"""Booking squares before you drive onto them.

PURE LOGIC ONLY. No web code, no ROS 2 code.

The rule that stops crashes
---------------------------
    A robot may not enter a square it has not booked.

If it cannot book the square ahead, it stops before it and waits.

Where the table lives
---------------------
Not on a server. 02_TECHNICAL_ARCHITECTURE section 3: the backend "must not be
the only component capable of preventing a robot collision." So EVERY robot
keeps its own copy of this table, built from PATH_RESERVATION messages it hears
on the radio.

Two robots claiming at once
---------------------------
Messages take time, so two robots really can claim the same square before
either hears the other. The fix is that nobody "decides" anything at write
time. Every claim is simply recorded, and the OWNER is worked out fresh when
asked, using a rule every robot applies identically:

    higher priority wins; if tied, the lower robot ID wins

04_DECENTRALIZED_FLEET_PROTOCOL section 7: "If scores are equal: lower robot_id
wins. This makes the system deterministic."

Because ownership is computed from the whole set rather than decided as claims
arrive, robots reach the same answer even if the messages reach them in a
different ORDER. That order-independence is the safety property.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from .grid import Cell
from .priority import bucket

# How many squares ahead a robot books. Booking a whole 40-square route would
# let one robot own half the warehouse while everyone else sat still.
DEFAULT_LOOKAHEAD = 3

# Reservations this far in the past are forgotten.
FORGET_AFTER = 2.0

# The priority a robot uses for the square it is physically standing on.
# This is not a preference, it is a fact: you cannot win a booking for a square
# that already has a robot parked in it. Nothing outranks it.
OCCUPANCY_PRIORITY = 100_000

ResourceKey = Tuple


def node_key(cell: Cell) -> ResourceKey:
    """The key for one square."""
    return ("N", cell.x, cell.y)


def edge_key(a: Cell, b: Cell) -> ResourceKey:
    """The key for one aisle segment.

    Sorted so that a->b and b->a are the SAME resource. That is what stops two
    robots booking opposite directions down the same aisle and driving into
    each other -- the head-on case from 05_PATH_PLANNING section 5.
    """
    p, q = (a.x, a.y), (b.x, b.y)
    if q < p:
        p, q = q, p
    return ("E", p[0], p[1], q[0], q[1])


@dataclass
class Reservation:
    """One booking. Fields follow the data model in 02_TECHNICAL_ARCHITECTURE §5."""

    robot_id: str
    resource: ResourceKey
    start: float          # absolute time, not "seconds from now"
    end: float
    priority: int = 5

    def overlaps(self, start: float, end: float) -> bool:
        return self.start < end and start < self.end

    @property
    def rank(self) -> Tuple[int, str]:
        """Lower is better. Higher priority band first, then lower robot ID.

        Compared in whole points, not tenths -- see PRIORITY_BUCKET in
        priority.py. Robots hold slightly different copies of this table, so a
        comparison that hinges on a tenth of a point would let two robots each
        decide they had won. Coarse bands plus the lower-ID fallback means both
        reach the same answer from slightly different information.
        """
        return (-bucket(self.priority), self.robot_id)

    def to_dict(self) -> Dict[str, object]:
        kind = "NODE" if self.resource[0] == "N" else "EDGE"
        if kind == "NODE":
            where = f"({self.resource[1]},{self.resource[2]})"
        else:
            where = f"({self.resource[1]},{self.resource[2]})-({self.resource[3]},{self.resource[4]})"
        return {
            "robot_id": self.robot_id, "kind": kind, "where": where,
            "start": round(self.start, 2), "end": round(self.end, 2),
            "priority": self.priority,
        }


class ReservationTable:
    """One robot's own copy of who has booked what, and when."""

    def __init__(self) -> None:
        self._by_resource: Dict[ResourceKey, List[Reservation]] = {}

    # ---------------------------------------------------------------- write

    def put(self, res: Reservation) -> None:
        """Record a booking. Replaces any earlier booking by the same robot on
        the same resource, because a robot only holds one slot per square."""
        slots = self._by_resource.setdefault(res.resource, [])
        for i, existing in enumerate(slots):
            if existing.robot_id == res.robot_id:
                slots[i] = res
                return
        slots.append(res)

    def release(self, robot_id: str, resource: Optional[ResourceKey] = None) -> int:
        """Give a booking back. With no resource, gives back everything."""
        removed = 0
        targets = [resource] if resource is not None else list(self._by_resource.keys())
        for key in targets:
            slots = self._by_resource.get(key)
            if not slots:
                continue
            keep = [r for r in slots if r.robot_id != robot_id]
            removed += len(slots) - len(keep)
            if keep:
                self._by_resource[key] = keep
            else:
                del self._by_resource[key]
        return removed

    def prune(self, now: float) -> None:
        """Forget bookings whose time has passed."""
        for key in list(self._by_resource.keys()):
            keep = [r for r in self._by_resource[key] if r.end > now - FORGET_AFTER]
            if keep:
                self._by_resource[key] = keep
            else:
                del self._by_resource[key]

    # ----------------------------------------------------------------- read

    def claims_on(self, resource: ResourceKey, start: float, end: float) -> List[Reservation]:
        return [r for r in self._by_resource.get(resource, []) if r.overlaps(start, end)]

    def owner(self, resource: ResourceKey, start: float, end: float) -> Optional[Reservation]:
        """Who actually holds this square for that stretch of time?

        Worked out from every claim on file, not from whoever asked first.
        That is what makes it order-independent.
        """
        claims = self.claims_on(resource, start, end)
        if not claims:
            return None
        return min(claims, key=lambda r: r.rank)

    def is_owner(self, robot_id: str, resource: ResourceKey, start: float, end: float) -> bool:
        holder = self.owner(resource, start, end)
        return holder is not None and holder.robot_id == robot_id

    def blocked_by(self, robot_id: str, resource: ResourceKey,
                   start: float, end: float) -> Optional[str]:
        """If this robot cannot have it, who is in the way?"""
        holder = self.owner(resource, start, end)
        if holder is None or holder.robot_id == robot_id:
            return None
        return holder.robot_id

    def rows(self, now: float, limit: int = 14) -> List[Dict[str, object]]:
        """The table as the dashboard shows it, soonest first."""
        out: List[Reservation] = []
        for slots in self._by_resource.values():
            out.extend(slots)
        out.sort(key=lambda r: (r.start, r.rank))
        rows = []
        for r in out:
            if r.end < now:
                continue
            row = r.to_dict()
            holder = self.owner(r.resource, r.start, r.end)
            row["granted"] = holder is not None and holder.robot_id == r.robot_id
            rows.append(row)
            if len(rows) >= limit:
                break
        return rows

    def held_nodes(self, robot_id: str, now: float, until: float) -> List[Cell]:
        """Squares this robot currently owns. Used to tint the map."""
        cells: List[Cell] = []
        for key, slots in self._by_resource.items():
            if key[0] != "N":
                continue
            for r in slots:
                if r.robot_id != robot_id or not r.overlaps(now, until):
                    continue
                if self.is_owner(robot_id, key, r.start, r.end):
                    cells.append(Cell(key[1], key[2]))
                break
        return cells

    def __len__(self) -> int:
        return sum(len(v) for v in self._by_resource.values())


def avoidance_cost(
    table: "ReservationTable",
    robot_id: str,
    now: float,
    lookahead: float = 6.0,
    booked_penalty: float = 6.0,
    avoid_robot: Optional[str] = None,
    avoid_penalty: float = 25.0,
    occupied: Optional[Iterable[Cell]] = None,
    occupied_penalty: float = 60.0,
):
    """A route-cost rule that steers around squares other robots have booked.

    05_PATH_PLANNING section 3:

        edge_cost = distance + congestion_penalty + blocked_penalty
                  + reservation_penalty + energy_penalty

    Booked squares are made EXPENSIVE, never forbidden. This is how a maps app
    handles traffic: a jammed road is not closed, it is just slow, so you get
    routed around it unless there is genuinely no alternative -- in which case
    you sit in the jam.

    Banning them instead would be worse: a robot in a one-way corridor would
    find no route at all and report BLOCKED, which is worse than waiting.
    """
    busy = set(occupied or ())

    def cost(_from_cell: Cell, to_cell: Cell) -> float:
        price = 1.0
        if to_cell in busy:
            price += occupied_penalty
        holder = table.owner(node_key(to_cell), now, now + lookahead)
        if holder is not None and holder.robot_id != robot_id:
            price += avoid_penalty if holder.robot_id == avoid_robot else booked_penalty
        return price

    return cost
