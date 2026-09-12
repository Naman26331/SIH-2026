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
from .reservations import ReservationTable, edge_key, node_key, target_key

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


def find_space_time_path(
    grid: Grid,
    start: Cell,
    goal: Cell,
    *,
    table: Optional[ReservationTable],
    robot_id: str,
    now: float,
    speed: float,
    cost_fn: Optional[CostFn] = None,
    blocked: Optional[Set[Cell]] = None,
    max_time_steps: int = 200,
    max_expansions: int = 12_000,
    clearance: float = 0.35,
) -> Optional[List[Cell]]:
    """Bounded A* over ``(cell, time)`` with a real WAIT action.

    Every candidate checks both its destination cell and traversed edge in the
    robot's local reservation table. No shared/global table is required.
    """
    cost_fn = cost_fn or uniform_cost
    blocked = blocked or set()
    if not grid.is_walkable(start) or not grid.is_walkable(goal) or goal in blocked:
        return None
    if start == goal:
        return [start]

    dwell = 1.0 / max(speed, 0.1)
    # Long enough for a useful detour/wait, but never let a small map create
    # hundreds of pointless time layers. The expansion cap remains the final
    # CPU guard for dense traffic.
    max_time_steps = min(max_time_steps,
                         max(int(manhattan(start, goal)) + 32, 48))
    State = Tuple[Cell, int]
    origin: State = (start, 0)
    frontier: List[Tuple[float, int, State]] = [(manhattan(start, goal), 0, origin)]
    came_from: Dict[State, State] = {}
    cost_so_far: Dict[State, float] = {origin: 0.0}
    counter = 1
    expansions = 0

    while frontier and expansions < max_expansions:
        _, _, state = heapq.heappop(frontier)
        current, step = state
        expansions += 1
        if current == goal:
            route = [current]
            while state != origin:
                state = came_from[state]
                route.append(state[0])
            route.reverse()
            return route
        if step >= max_time_steps:
            continue

        depart = now + step * dwell
        arrive = depart + dwell
        # Waiting is a first-class action, considered after movement so an
        # equally good clear route does not acquire pointless pauses.
        actions = list(grid.neighbours(current)) + [current]
        for nxt in actions:
            if nxt in blocked:
                continue
            if table is not None:
                node_owner = table.blocked_by(
                    robot_id, node_key(nxt), arrive - clearance,
                    arrive + dwell + clearance,
                )
                if node_owner is not None:
                    continue
                if nxt != current:
                    edge_owner = table.blocked_by(
                        robot_id, edge_key(current, nxt), depart - clearance,
                        arrive + clearance,
                    )
                    if edge_owner is not None:
                        continue
                if nxt == goal:
                    lease_owner = table.blocked_by(
                        robot_id, target_key(goal), arrive - clearance,
                        arrive + dwell + clearance,
                    )
                    if lease_owner is not None:
                        continue

            nxt_state = (nxt, step + 1)
            wait_cost = 1.1 if nxt == current else cost_fn(current, nxt)
            new_cost = cost_so_far[state] + wait_cost
            if new_cost >= cost_so_far.get(nxt_state, float("inf")):
                continue
            cost_so_far[nxt_state] = new_cost
            came_from[nxt_state] = state
            heapq.heappush(frontier, (
                new_cost + manhattan(nxt, goal), counter, nxt_state,
            ))
            counter += 1

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
