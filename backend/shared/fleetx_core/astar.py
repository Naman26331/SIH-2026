"""SIPP + A* -- the route finders. Same idea as a maps app.

PURE LOGIC ONLY. No web code, no ROS 2 code.

Two planners, both executed independently by every robot on its OWN local
knowledge (peer-to-peer reservations, never a global table):

  find_path      -- static spatial A* (no time). Used for map-only questions
                    such as a wandering person or a charger lookup.
  find_sipp_path -- THE fleet planner. Safe Interval Path Planning over the
                    robot's local reservation copy. Contiguous free timesteps
                    at a cell are one safe interval, so waits fall out of the
                    search instead of being explicit time layers.

Why SIPP and not space-time A*: same earliest-arrival routes, far fewer
states, and the interval form matches the reservation table's
``unavailable_intervals()`` directly. Pair with PIBT priority inheritance
(see robot.py) for decentralized coordination.
"""

import heapq
import math
from typing import Callable, Dict, List, Optional, Set, Tuple

from .grid import Cell, Grid
from .reservations import ReservationTable, edge_key, node_key, target_key

# A cost function scores one step from square A to square B.
# Phase 1 always returns 1.0 (every step costs the same).
# Later phases will make busy or reserved aisles cost more, which is how a
# robot ends up choosing a longer-but-faster route.
CostFn = Callable[[Cell, Cell], float]


def stop_capable_step_time(distance: float, cruise_speed: float,
                           acceleration: float, deceleration: float) -> float:
    """Conservative travel time with enough distance to accelerate and stop.

    Every reserved grid transition is treated as stop-capable. This is less
    optimistic than distance/cruise-speed and prevents SIPP timestamps from
    promising that physical wheels can clear a cell sooner than they can.
    """
    distance = max(0.0, distance)
    speed = max(0.1, cruise_speed)
    accel = max(0.1, acceleration)
    brake = max(0.1, deceleration)
    accelerate_distance = speed * speed / (2.0 * accel)
    brake_distance = speed * speed / (2.0 * brake)
    if accelerate_distance + brake_distance <= distance:
        cruise_distance = distance - accelerate_distance - brake_distance
        return speed / accel + cruise_distance / speed + speed / brake
    peak = math.sqrt(2.0 * distance / (1.0 / accel + 1.0 / brake))
    return peak / accel + peak / brake


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


def find_sipp_path(
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
    ignore_future_from: Optional[Set[str]] = None,
) -> Optional[List[Cell]]:
    """Safe Interval Path Planning using only this robot's local knowledge.

    Decentralized-only: ``table`` is the robot's OWN reservation copy built
    from peer-to-peer PATH_RESERVATION messages, never a shared/global
    service. Each robot runs this independently; coordination emerges from
    reservations + PIBT inheritance (see robot.py), not from a boss.
    """
    cost_fn = cost_fn or uniform_cost
    blocked = blocked or set()
    if not grid.is_walkable(start) or not grid.is_walkable(goal) or goal in blocked:
        return None
    if start == goal:
        return [start]

    dwell = 1.0 / max(speed, 0.1)
    max_time_steps = min(max_time_steps,
                         max(int(manhattan(start, goal)) + 32, 48))
    horizon = now + max_time_steps * dwell
    interval_cache: Dict[Cell, List[Tuple[float, float]]] = {}

    def safe_intervals(cell: Cell) -> List[Tuple[float, float]]:
        cached = interval_cache.get(cell)
        if cached is not None:
            return cached
        denied: List[Tuple[float, float]] = []
        if table is not None:
            resources = [node_key(cell)]
            if cell == goal:
                resources.append(target_key(cell))
            for resource in resources:
                for left, right in table.unavailable_intervals(
                        robot_id, resource, now - clearance, horizon + clearance,
                        ignore_future_from=ignore_future_from):
                    denied.append((left, right))
        denied.sort()
        merged: List[Tuple[float, float]] = []
        for left, right in denied:
            left, right = max(now, left), min(horizon, right)
            if right <= left:
                continue
            if merged and left <= merged[-1][1]:
                merged[-1] = (merged[-1][0], max(merged[-1][1], right))
            else:
                merged.append((left, right))
        safe: List[Tuple[float, float]] = []
        cursor = now
        for left, right in merged:
            if cursor < left:
                safe.append((cursor, left))
            cursor = max(cursor, right)
        if cursor < horizon:
            safe.append((cursor, horizon))
        interval_cache[cell] = safe
        return safe

    start_intervals = safe_intervals(start)
    start_index = next((i for i, (left, right) in enumerate(start_intervals)
                        if left <= now < right), None)
    if start_index is None:
        return None

    State = Tuple[Cell, int]
    origin: State = (start, start_index)
    frontier: List[Tuple[float, int, float, State]] = [
        (now + manhattan(start, goal) * dwell, 0, now, origin)
    ]
    arrival: Dict[State, float] = {origin: now}
    came_from: Dict[State, Tuple[State, float]] = {}
    counter = 1
    expansions = 0

    def align_to_tick(value: float) -> float:
        slots = max(0, math.ceil((value - now) / dwell - 1e-9))
        return now + slots * dwell

    def edge_delay(a: Cell, b: Cell, depart: float) -> float:
        if table is None:
            return align_to_tick(depart)
        probe = align_to_tick(depart)
        while probe + dwell <= horizon:
            conflicts = table.unavailable_intervals(
                robot_id, edge_key(a, b), probe - clearance,
                probe + dwell + clearance,
                ignore_future_from=ignore_future_from,
            )
            if not conflicts:
                return probe
            nxt = align_to_tick(max(end for _, end in conflicts) + clearance)
            # Never re-probe the slot just proven blocked. A claim can
            # overlap this window by float dust, and align_to_tick's 1e-9
            # snap then lands back on the SAME slot -- without this guard
            # that is an infinite loop (the sim thread holds the server
            # lock, so the whole backend freezes and /map never answers).
            # Both branches are tick-aligned, so the probe strictly
            # advances and this always terminates at the horizon.
            probe = nxt if nxt > probe else probe + dwell
        return float("inf")

    goal_state: Optional[State] = None
    while frontier and expansions < max_expansions:
        _, _, queued_arrival, state = heapq.heappop(frontier)
        if queued_arrival != arrival.get(state):
            continue
        current, interval_index = state
        here_at = queued_arrival
        expansions += 1
        if current == goal:
            goal_state = state
            break

        _, current_end = safe_intervals(current)[interval_index]
        for nxt in grid.neighbours(current):
            if nxt in blocked:
                continue
            for nxt_index, (safe_start, safe_end) in enumerate(safe_intervals(nxt)):
                candidate = max(here_at, safe_start + clearance - dwell)
                depart = edge_delay(current, nxt, candidate)
                reach = depart + dwell
                if depart + clearance > current_end:
                    break
                if reach - clearance < safe_start:
                    depart = edge_delay(
                        current, nxt, safe_start + clearance - dwell)
                    reach = depart + dwell
                if reach + dwell + clearance > safe_end:
                    continue
                nxt_state = (nxt, nxt_index)
                if reach >= arrival.get(nxt_state, float("inf")):
                    continue
                arrival[nxt_state] = reach
                came_from[nxt_state] = (state, depart)
                # Time is primary. Tiny local cost tie-break retains aisle
                # preference without invalidating SIPP's earliest-arrival state.
                tie_cost = max(0.0, cost_fn(current, nxt) - 1.0) * 1e-4
                heapq.heappush(frontier, (
                    reach + manhattan(nxt, goal) * dwell + tie_cost,
                    counter, reach, nxt_state,
                ))
                counter += 1

    if goal_state is None:
        return None

    timed: List[Tuple[Cell, float]] = [(goal_state[0], arrival[goal_state])]
    state = goal_state
    while state != origin:
        previous, _ = came_from[state]
        timed.append((previous[0], arrival[previous]))
        state = previous
    timed.reverse()

    route = [timed[0][0]]
    for (previous, previous_at), (cell, cell_at) in zip(timed, timed[1:]):
        waits = max(0, int(round((cell_at - previous_at) / dwell)) - 1)
        route.extend([previous] * waits)
        route.append(cell)
    return route


def _rebuild(came_from: Dict[Cell, Cell], start: Cell, goal: Cell) -> List[Cell]:
    """Walk the 'I came from here' markers backwards to get the route."""
    route = [goal]
    node = goal
    while node != start:
        node = came_from[node]
        route.append(node)
    route.reverse()
    return route
