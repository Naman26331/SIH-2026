"""A* -- the route finder. Same idea as a maps app.

PURE LOGIC ONLY. No web code, no ROS 2 code.

How it works, in one paragraph:
  Start at the robot's square. Keep a list of squares worth exploring, always
  picking the one that looks most promising. "Most promising" = steps already
  walked + a guess of how many steps are still left. Shelves are simply never
  added to the list. When the goal comes off the list, walk the trail of
  "I came from here" markers backwards -- that trail is the route.

Why A* and not something cleverer: it is simple, it always finds the shortest
route if one exists, and it is easy to explain to a judge (see 11_SIH_DEMO).
"""

import heapq
from typing import Callable, Dict, List, Optional, Set, Tuple

from .grid import Cell, Grid

# A cost function scores one step from square A to square B.
# Phase 1 always returns 1.0 (every step costs the same).
# Later phases will make busy or reserved aisles cost more, which is how a
# robot ends up choosing a longer-but-faster route (see 05_PATH_PLANNING §3).
CostFn = Callable[[Cell, Cell], float]


def uniform_cost(_from_cell: Cell, _to_cell: Cell) -> float:
    """Every step costs the same. The Phase 1 default."""
    return 1.0


def manhattan(a: Cell, b: Cell) -> float:
    """Guess of the steps left: how far apart, walking only in straight lines.

    This never over-guesses, which is what makes A* trustworthy.
    """
    return float(abs(a.x - b.x) + abs(a.y - b.y))


def find_path(
    grid: Grid,
    start: Cell,
    goal: Cell,
    cost_fn: Optional[CostFn] = None,
    blocked: Optional[Set[Cell]] = None,
) -> Optional[List[Cell]]:
    """Work out a route from start to goal.

    Returns the full list of squares INCLUDING the start and the goal.
    Returns None if there is no way through (goal is inside a shelf, or walled
    off). Returns [start] if the robot is already standing on the goal.

    `blocked` is squares that are genuinely IMPASSABLE right now -- a fallen
    box, a broken-down robot. Note the difference from a busy square, which is
    merely expensive (see avoidance_cost in reservations.py): you can push
    through a busy aisle if you must, but you cannot drive through a pallet
    because you are in a hurry. If a block cuts off the only route, this
    honestly returns None and the robot reports BLOCKED.
    """
    if cost_fn is None:
        cost_fn = uniform_cost
    blocked = blocked or set()

    if not grid.is_walkable(start):
        return None
    if not grid.is_walkable(goal) or goal in blocked:
        return None
    if start == goal:
        return [start]

    # The "worth exploring" list, cheapest guess first.
    # Entries are (guess_of_total_cost, tie_breaker, square).
    frontier: List[Tuple[float, int, Cell]] = [(manhattan(start, goal), 0, start)]
    came_from: Dict[Cell, Cell] = {}
    cost_so_far: Dict[Cell, float] = {start: 0.0}
    visited: set = set()
    counter = 1

    while frontier:
        _, _, current = heapq.heappop(frontier)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return _rebuild(came_from, start, goal)

        for nxt in grid.neighbours(current):
            if nxt in blocked:
                continue                      # cannot drive through it, at all
            new_cost = cost_so_far[current] + cost_fn(current, nxt)
            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(frontier, (new_cost + manhattan(nxt, goal), counter, nxt))
                counter += 1

    # Explored everything reachable and never found the goal.
    return None


def _rebuild(came_from: Dict[Cell, Cell], start: Cell, goal: Cell) -> List[Cell]:
    """Walk the 'I came from here' markers backwards to get the route."""
    route = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        route.append(node)
    route.reverse()
    return route
