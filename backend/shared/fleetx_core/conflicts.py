"""Spotting a crash before it happens.

PURE LOGIC ONLY. No web code, no ROS 2 code.

The one idea
------------
A robot does not touch a square at an instant. It ARRIVES and later CLEARS.
So instead of "R1 is at (13,8) at 3.2s" we say:

    "R1 occupies (13,8) from 3.0s to 3.6s"

Now it is just double-booking a meeting room:

    square (13,8)
    R1   |========|            3.0 -> 3.6
    R2        |========|       3.4 -> 4.0
                  ^ overlap = conflict

Same resource AND overlapping time window means a conflict.

This same table of bookings becomes the reservation system in Phase 5. Here we
only READ it and complain. There, robots start booking slots in it.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Sequence, Tuple

from .grid import Cell, Grid

# How far ahead to look, in seconds. Longer than this and you get constant
# noise from plans that change before they ever matter.
DEFAULT_HORIZON = 8.0

# A robot has a body, so it needs a little clear space either side of its slot.
# You do not book a meeting room ending at 3.6 and the next one starting at 3.6.
DEFAULT_CLEARANCE = 0.35


class ConflictKind(str, Enum):
    SAME_SQUARE = "SAME_SQUARE"   # both want the same square at the same time
    HEAD_ON = "HEAD_ON"           # swapping down the same aisle, nose to nose
    JUNCTION = "JUNCTION"         # same square, but at a place aisles cross


@dataclass(frozen=True)
class Window:
    """One booking: who is on what, from when, to when."""

    cell: Cell
    start: float
    end: float

    def overlaps(self, other: "Window") -> bool:
        return self.start < other.end and other.start < self.end

    def overlap_start(self, other: "Window") -> float:
        return max(self.start, other.start)


@dataclass(frozen=True)
class EdgeWindow:
    """A robot moving along one aisle segment, from square a to square b."""

    a: Cell
    b: Cell
    start: float
    end: float

    def overlaps(self, other: "EdgeWindow") -> bool:
        return self.start < other.end and other.start < self.end


@dataclass
class Plan:
    """Everything one robot intends to do, as bookings."""

    robot_id: str
    nodes: List[Window]
    edges: List[EdgeWindow]


@dataclass
class Conflict:
    """Two robots want the same thing at the same time."""

    robot_a: str
    robot_b: str
    kind: ConflictKind
    cell: Cell
    lead_time: float          # seconds from now until the clash starts
    overlap_start: float
    overlap_end: float

    @property
    def pair(self) -> frozenset:
        return frozenset((self.robot_a, self.robot_b))

    def to_dict(self) -> Dict[str, object]:
        return {
            "robot_a": self.robot_a, "robot_b": self.robot_b,
            "kind": self.kind.value,
            "cell": [self.cell.x, self.cell.y],
            "lead_time": round(self.lead_time, 2),
            "overlap_start": round(self.overlap_start, 2),
            "overlap_end": round(self.overlap_end, 2),
        }


def build_plan(
    robot_id: str,
    current_cell: Cell,
    nodes: Sequence[Cell],
    etas: Sequence[float],
    speed: float,
    horizon: float = DEFAULT_HORIZON,
    clearance: float = DEFAULT_CLEARANCE,
    time_offset: float = 0.0,
) -> Plan:
    """Turn "I'll be at these squares at these times" into bookings.

    time_offset shifts everything earlier, for information that arrived late:
    a message that took 0.3s to reach us has ETAs that are 0.3s out of date.

    A robot with no route is parked. It occupies its square for the whole
    horizon -- which is exactly right, because driving into a parked robot is
    just as much a crash as driving into a moving one.
    """
    dwell = (1.0 / speed) if speed > 0 else 1.0
    shifted = [max(0.0, e - time_offset) for e in etas]

    windows: List[Window] = []
    edges: List[EdgeWindow] = []

    if not nodes:
        windows.append(Window(current_cell, 0.0, horizon))
        return Plan(robot_id, windows, edges)

    # The square it is standing on now, until it reaches the next one.
    windows.append(Window(current_cell, 0.0, shifted[0] + clearance))

    chain = [current_cell] + list(nodes)
    for i, node in enumerate(nodes):
        enter = shifted[i]
        # It leaves this square when it reaches the next one. If this is the
        # last square, it stops there and stays put.
        leave = shifted[i + 1] if i + 1 < len(shifted) else enter + dwell
        if enter - clearance > horizon:
            break
        windows.append(Window(node, max(0.0, enter - clearance), leave + clearance))

        prev = chain[i]
        if prev != node:
            edge_start = shifted[i - 1] if i > 0 else 0.0
            edges.append(EdgeWindow(
                prev, node, max(0.0, edge_start - clearance),
                enter + clearance,
            ))

    return Plan(robot_id, windows, edges)


def find_conflicts(
    own: Plan,
    others: Sequence[Plan],
    grid: Optional[Grid] = None,
    horizon: float = DEFAULT_HORIZON,
) -> List[Conflict]:
    """Compare one robot's bookings against everyone else's.

    Returns the soonest conflict per (other robot, square), soonest first.
    Only ever returns what THIS robot could work out from what it has heard --
    there is no god-view here. A robot whose messages were lost is invisible,
    and that is the honest behaviour for a decentralised fleet.
    """
    found: Dict[Tuple[str, Cell], Conflict] = {}

    for other in others:
        if other.robot_id == own.robot_id:
            continue

        # (1) and (3): both want the same square at overlapping times.
        for mine in own.nodes:
            if mine.start > horizon:
                continue
            for theirs in other.nodes:
                if mine.cell != theirs.cell or not mine.overlaps(theirs):
                    continue
                start = mine.overlap_start(theirs)
                if start > horizon:
                    continue
                junction = grid is not None and grid.is_junction(mine.cell)
                _keep(found, Conflict(
                    robot_a=own.robot_id, robot_b=other.robot_id,
                    kind=ConflictKind.JUNCTION if junction else ConflictKind.SAME_SQUARE,
                    cell=mine.cell, lead_time=start,
                    overlap_start=start, overlap_end=min(mine.end, theirs.end),
                ))

        # (2) head-on: same aisle segment, opposite directions, same time.
        # They never share a square, so the check above cannot see this.
        for mine_e in own.edges:
            if mine_e.start > horizon:
                continue
            for theirs_e in other.edges:
                if mine_e.a != theirs_e.b or mine_e.b != theirs_e.a:
                    continue
                if not mine_e.overlaps(theirs_e):
                    continue
                start = max(mine_e.start, theirs_e.start)
                if start > horizon:
                    continue
                _keep(found, Conflict(
                    robot_a=own.robot_id, robot_b=other.robot_id,
                    kind=ConflictKind.HEAD_ON, cell=mine_e.b, lead_time=start,
                    overlap_start=start, overlap_end=min(mine_e.end, theirs_e.end),
                ))

    return sorted(found.values(), key=lambda c: c.lead_time)


def _keep(found: Dict[Tuple[str, Cell], Conflict], candidate: Conflict) -> None:
    """Keep the soonest clash for each other-robot-and-square, and prefer a
    head-on label over a plain same-square one for the same spot."""
    key = (candidate.robot_b, candidate.cell)
    existing = found.get(key)
    if existing is None:
        found[key] = candidate
        return
    if candidate.kind is ConflictKind.HEAD_ON and existing.kind is not ConflictKind.HEAD_ON:
        found[key] = candidate
    elif candidate.lead_time < existing.lead_time and candidate.kind is existing.kind:
        found[key] = candidate
