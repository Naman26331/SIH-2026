"""Squares a robot believes are blocked by something that should not be there.

PURE LOGIC ONLY. No web code, no ROS 2 code.

A box falls off a pallet. Somebody leaves a trolley in an aisle. The map on
file is now wrong, and nobody told the robots.

05_PATH_PLANNING section 10:

    sensor detects obstacle
            -> local map update
            -> BLOCKED_AISLE broadcast
            -> robots invalidate affected paths
            -> A* replanning
            -> reservation update

    "Use TTL so temporary obstacles eventually expire."

Why blocks must expire
----------------------
Somebody picks the box up. If a block never faded, the warehouse would carry a
phantom wall for ever. Over a shift the map would silently fill with obstacles
that are not there and throughput would quietly collapse.

So: see it, and it is confirmed. Stop seeing it, and it fades.
"""

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set

from .grid import Cell

# How long a block survives without anybody confirming it again.
DEFAULT_TTL = 20.0

# How far a robot can see. Deliberately short: a robot has to GO somewhere to
# find out what is there. If every robot magically knew the instant a box
# landed, there would be nothing decentralised about any of this.
SENSOR_RANGE = 3.0


@dataclass
class Block:
    """One square somebody believes is blocked."""

    cell: Cell
    reported_by: str
    seen_at: float             # when it was last actually confirmed
    ttl: float = DEFAULT_TTL
    confidence: float = 1.0

    def expired(self, now: float) -> bool:
        return now - self.seen_at > self.ttl

    def remaining(self, now: float) -> float:
        return max(0.0, self.ttl - (now - self.seen_at))

    def to_dict(self, now: float) -> Dict[str, object]:
        return {
            "cell": [self.cell.x, self.cell.y],
            "reported_by": self.reported_by,
            "confidence": round(self.confidence, 2),
            "remaining": round(self.remaining(now), 1),
        }


class BlockedMap:
    """One robot's own picture of what is in the way.

    Like the fleet notebook and the booking table, this is NOT the truth. It is
    what this robot has seen for itself, plus what it has been told. Two robots
    can disagree, and a robot that has not been down an aisle recently may
    still believe a box is there that has long since been cleared.
    """

    def __init__(self, ttl: float = DEFAULT_TTL):
        self.ttl = ttl
        self._blocks: Dict[Cell, Block] = {}

    # ---------------------------------------------------------------- write

    def mark(self, cell: Cell, reported_by: str, now: float,
             ttl: Optional[float] = None, confidence: float = 1.0) -> bool:
        """Record (or re-confirm) a blocked square. True if this is news."""
        existing = self._blocks.get(cell)
        self._blocks[cell] = Block(
            cell=cell, reported_by=reported_by, seen_at=now,
            ttl=self.ttl if ttl is None else ttl, confidence=confidence,
        )
        return existing is None or existing.expired(now)

    def clear(self, cell: Cell) -> bool:
        """Somebody looked and there is nothing there after all."""
        return self._blocks.pop(cell, None) is not None

    def expire(self, now: float) -> List[Cell]:
        """Drop blocks nobody has confirmed lately. Returns what was forgotten."""
        gone = [c for c, b in self._blocks.items() if b.expired(now)]
        for cell in gone:
            del self._blocks[cell]
        return gone

    # ----------------------------------------------------------------- read

    def is_blocked(self, cell: Cell, now: float) -> bool:
        block = self._blocks.get(cell)
        return block is not None and not block.expired(now)

    def cells(self, now: float) -> Set[Cell]:
        return {c for c, b in self._blocks.items() if not b.expired(now)}

    def blocks_any(self, path: Iterable[Cell], now: float) -> Optional[Cell]:
        """Does this route run through anything we believe is blocked?"""
        for cell in path:
            if self.is_blocked(cell, now):
                return cell
        return None

    def get(self, cell: Cell) -> Optional[Block]:
        return self._blocks.get(cell)

    def to_rows(self, now: float) -> List[Dict[str, object]]:
        return [b.to_dict(now) for b in
                sorted(self._blocks.values(), key=lambda b: -b.remaining(now))
                if not b.expired(now)]

    def __len__(self) -> int:
        return len(self._blocks)


def within_range(rx: float, ry: float, cell: Cell,
                 sensor_range: float = SENSOR_RANGE) -> bool:
    """Can a robot at (rx, ry) see that square?

    Plain distance. A real robot's field of view is more complicated -- shelves
    block the view -- but distance is honest enough to make the point, and the
    ROS 2 version will get this from a real laser scanner anyway.
    """
    dx, dy = cell.x - rx, cell.y - ry
    return (dx * dx + dy * dy) <= sensor_range * sensor_range
