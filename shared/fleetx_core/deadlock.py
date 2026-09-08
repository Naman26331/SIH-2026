"""Spotting a jam that will never clear itself, and choosing who gives way.

PURE LOGIC ONLY. No web code, no ROS 2 code.

Two kinds of jam
----------------
1. A CIRCULAR WAIT. 05_PATH_PLANNING section 8 draws it:

       R1 -> R2 -> R3 -> R1

   Everybody is waiting for somebody, so nobody ever moves. Nothing short of
   one of them giving up will break it.

2. SOMEBODY PARKED IN THE WAY. A robot finished its job, stopped, and went
   idle -- on a square another robot needs. It is not waiting for anything, so
   it has no reason to ever move, and no amount of rerouting helps if that
   square IS the destination.

   This one is not in the roadmap, but it is the one that actually happened.

Choosing who gives way
----------------------
Whoever has waited least gives way; if it is close, the higher robot name does.
Names cannot go stale, which is what stops two robots arguing.

Unlike deciding who enters a square, a disagreement here is harmless: if two
robots both decide to give way, that is untidy, not dangerous. So this rule can
afford to be simpler than the one in priority.py.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .priority import bucket

# Waiting longer than this is not patience any more, it is a jam.
STUCK_SECONDS = 4.0


@dataclass
class Waiting:
    """One robot's report: who it is stuck behind."""

    robot_id: str
    blocked_by: str
    since: float
    priority: int
    is_moving: bool           # False = parked, so it will never clear on its own


class WaitForGraph:
    """Who is waiting for whom, as far as one robot has heard."""

    def __init__(self) -> None:
        self._blocked_by: Dict[str, str] = {}
        self._info: Dict[str, Waiting] = {}

    def add(self, entry: Waiting) -> None:
        self._blocked_by[entry.robot_id] = entry.blocked_by
        self._info[entry.robot_id] = entry

    def clear(self, robot_id: str) -> None:
        self._blocked_by.pop(robot_id, None)
        self._info.pop(robot_id, None)

    def blocker_of(self, robot_id: str) -> Optional[str]:
        return self._blocked_by.get(robot_id)

    def cycle_containing(self, robot_id: str) -> Optional[List[str]]:
        """Follow the "I am waiting for..." chain. If it comes back to where it
        started, that is a loop and it will never clear itself.

        Returns the members in order, or None if the chain just ends -- which
        means somebody at the far end is free to move and the jam should sort
        itself out.
        """
        seen: List[str] = [robot_id]
        current = robot_id
        for _ in range(len(self._blocked_by) + 1):
            nxt = self._blocked_by.get(current)
            if nxt is None:
                return None                    # chain ends: not a loop
            if nxt == robot_id:
                return seen                    # back to the start: a loop
            if nxt in seen:
                return None                    # a loop, but not one we are in
            seen.append(nxt)
            current = nxt
        return None

    def info(self, robot_id: str) -> Optional[Waiting]:
        return self._info.get(robot_id)

    def is_parked(self, robot_id: str) -> bool:
        """Is this robot stopped and NOT waiting for anybody?

        A robot like that will never move on its own. It has to be asked.
        """
        if robot_id in self._blocked_by:
            return False                       # it is waiting, so it is not parked
        entry = self._info.get(robot_id)
        return entry is not None and not entry.is_moving

    def to_rows(self) -> List[Dict[str, object]]:
        return [
            {"robot_id": w.robot_id, "blocked_by": w.blocked_by,
             "waiting": round(w.since, 1)}
            for w in sorted(self._info.values(), key=lambda w: -w.since)
            if w.blocked_by
        ]


def choose_victim(members: List[Tuple[str, int, float]]) -> Optional[str]:
    """Who gives way to break the loop?

    members is (robot_id, priority, seconds_waiting).

    The one who has waited least gives way -- which is the fair answer, because
    the robot that has been stuck longest gets to go first. Compared in whole
    points so a slightly stale reading does not change it, and settled by the
    higher name when it is close.
    """
    if not members:
        return None
    ranked = sorted(members, key=lambda m: (bucket(m[1]), _invert(m[0])))
    return ranked[0][0]


def _invert(robot_id: str) -> Tuple[int, ...]:
    """Sort keys so that the HIGHER name comes first."""
    return tuple(-ord(c) for c in robot_id)
