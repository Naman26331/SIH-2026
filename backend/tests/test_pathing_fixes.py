"""Regression tests for decentralized pathing safety and fairness."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))

from fleetx_core import (Cell, InMemoryBus, PoseUpdate, Reservation, Robot,
                         RobotStatus, Task, World, default_grid, find_path,
                         node_key)


def test_missing_reservation_stops_robot_fail_closed():
    robot = Robot("R1", Cell(0, 0))
    robot.path = [Cell(1, 0)]
    robot.goal = Cell(2, 0)
    robot.status = RobotStatus.MOVING

    robot.check_clearance(now=0.0, dt=0.05)

    assert robot.hold
    assert robot.emergency
    assert robot.blocked_by == "unreserved"


def test_initial_astar_uses_local_reservations():
    grid = default_grid()
    robot = Robot("R1", Cell(0, 0))
    robot.table.put(Reservation(
        "R2", node_key(Cell(1, 0)), 0.0, 10.0, priority=50))
    robot.set_goal(Cell(2, 0))

    robot.decide(grid, now=0.0, traffic_aware=True)

    assert robot.path[0] != Cell(1, 0)
    assert len(robot.path) > 2


def test_baseline_can_still_request_uniform_shortest_path():
    grid = default_grid()
    robot = Robot("R1", Cell(0, 0))
    robot.table.put(Reservation(
        "R2", node_key(Cell(1, 0)), 0.0, 10.0, priority=50))
    robot.set_goal(Cell(2, 0))

    robot.decide(grid, now=0.0, traffic_aware=False)

    assert robot.path == [Cell(1, 0), Cell(2, 0)]


def test_communication_batch_removes_robot_iteration_bias():
    world = World(bus=InMemoryBus())
    first = world.add_robot(Robot("R1", Cell(0, 0)))
    second = world.add_robot(Robot("R2", Cell(2, 0)))

    world.tick(0.05)
    assert first.messages_heard == second.messages_heard == 0

    world.tick(0.05)
    assert first.messages_heard == second.messages_heard
    assert first.messages_heard > 0


def _robot_with_peer(peer_cell):
    robot = Robot("R1", Cell(0, 0))
    robot.fleet.ingest(PoseUpdate(
        robot_id="R2", timestamp=0.0, seq=1,
        x=float(peer_cell.x), y=float(peer_cell.y),
        cell=(peer_cell.x, peer_cell.y), velocity=0.0,
    ), now=0.0)
    return robot


def test_task_congestion_counts_only_local_route_intersections():
    grid = default_grid()
    task = Task("T1", Cell(2, 0), Cell(2, 2))
    to_pickup = find_path(grid, Cell(0, 0), task.pickup)
    to_dropoff = find_path(grid, task.pickup, task.dropoff)

    far = _robot_with_peer(Cell(27, 15))
    on_route = _robot_with_peer(Cell(1, 0))

    assert far.local_route_congestion(to_pickup, to_dropoff, 0.0) == 0.0
    assert on_route.local_route_congestion(to_pickup, to_dropoff, 0.0) == 1.0
    assert on_route.cost_of(task, grid, 0.0) > far.cost_of(task, grid, 0.0)
