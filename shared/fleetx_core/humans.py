"""PART 1 and PART 2 -- people working the same floor as the robots.

19_HUMAN_ROBOT_COLLABORATION asks for exactly this: model where a person is
and make robots react conservatively around them.

A Human is deliberately much simpler than a Robot. It has no radio, no job, no
priority, no reservation table, no A* replanning of its own -- it just walks.
That simplicity is the point: a person does not coordinate with the fleet, so
nothing here should look like they do. Robots find out where a person is the
same way they find out about a dropped box -- by SENSING them locally, never
by a message -- because a real safety system built around people cannot
depend on a network staying up. See Robot.sense_humans() and the
PERSON_STOP_RADIUS / PERSON_SLOW_RADIUS checks in robot.py.

Pure Python. No web, no ROS 2.
"""

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Set

from .astar import find_path
from .grid import Cell, Grid


@dataclass
class Human:
    """One person, walking the floor.

    Movement mirrors Robot.advance() on purpose: fixed speed, one square's
    worth of progress accumulated per tick, never further than speed * dt in
    one step. The same teleport invariant the robots are held to applies here
    too -- a person cannot jump any more than a robot can.
    """

    human_id: str
    cell: Cell
    x: float = 0.0
    y: float = 0.0
    speed: float = 1.0              # squares per second -- a walking pace,
                                     # deliberately slower than a robot's drive
    goal: Optional[Cell] = None
    path: List[Cell] = field(default_factory=list)
    _progress: float = 0.0

    def __post_init__(self) -> None:
        self.x = float(self.cell.x)
        self.y = float(self.cell.y)

    def set_goal(self, grid: Grid, goal: Cell,
                blocked: Optional[Set[Cell]] = None) -> bool:
        """Head somewhere new. Returns whether a route was actually found.

        Unlike a robot, a person does not protect a half-finished step when
        redirected -- there is no reservation table to keep honest here, only
        somewhere to walk to next.
        """
        route = find_path(grid, self.cell, goal, blocked=blocked)
        if route is None:
            return False
        self.goal = goal
        self.path = route[1:]
        self._progress = 0.0
        return True

    # How close a robot has to be before a person simply stops rather than
    # keep closing the gap. The robot's own hard stop (Robot.PERSON_STOP_
    # RADIUS) protects a person from a robot that is still moving -- but a
    # PARKED robot never re-checks anything, because it has nowhere left to
    # go, and nothing stopped a person's own path from walking straight up to
    # one. A real person would not walk into a robot either. Same number as
    # the robot's own bubble, so neither side alone has to close a gap the
    # other is not equally respecting.
    ROBOT_STOP_RADIUS = 1.3

    def advance(self, dt: float, robots: Iterable[tuple] = ()) -> None:
        """Walk for dt seconds -- unless that would bring us in close to a
        robot, in which case simply wait where we are. `robots` is an
        iterable of (x, y) positions; passed fresh every tick, exactly the
        way a person would actually notice one nearby and hold back."""
        if not self.path:
            self.x, self.y = float(self.cell.x), float(self.cell.y)
            return

        trial = self._progress + self.speed * dt
        nxt = self.path[0]
        trial_x = self.cell.x + (nxt.x - self.cell.x) * min(trial, 1.0)
        trial_y = self.cell.y + (nxt.y - self.cell.y) * min(trial, 1.0)
        for rx, ry in robots:
            if ((rx - trial_x) ** 2 + (ry - trial_y) ** 2
                    < self.ROBOT_STOP_RADIUS ** 2):
                return                   # stay exactly where we are this tick

        self._progress = trial
        while self._progress >= 1.0 and self.path:
            self._progress -= 1.0
            self.cell = self.path.pop(0)

        if self.path:
            nxt = self.path[0]
            self.x = self.cell.x + (nxt.x - self.cell.x) * self._progress
            self.y = self.cell.y + (nxt.y - self.cell.y) * self._progress
        else:
            self._progress = 0.0
            self.x, self.y = float(self.cell.x), float(self.cell.y)
            self.goal = None

    def to_dict(self) -> Dict[str, object]:
        """Plain data about this person. Dicts and numbers only."""
        return {
            "human_id": self.human_id,
            "cell": [self.cell.x, self.cell.y],
            "x": round(self.x, 3), "y": round(self.y, 3),
            "goal": [self.goal.x, self.goal.y] if self.goal else None,
        }
