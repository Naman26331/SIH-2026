"""Regression tests for backend safety fixes."""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))

from fleetx_core import (Cell, Grid, Robot, RobotStatus, Task, TaskClaim,
                         TaskStatus, World)


class TestFailedRobotRemainsPhysical(unittest.TestCase):
    def test_failure_freezes_robot_on_nearest_cell(self):
        world = World(grid=Grid((".....", ".....")))
        failed = world.add_robot(Robot("R1", Cell(1, 0)))
        failed.x = 1.75
        failed._progress = 0.75
        failed.path = [Cell(2, 0)]

        world.fail_robot("R1")

        self.assertIs(failed.status, RobotStatus.FAILED)
        self.assertEqual(failed.cell, Cell(2, 0))
        self.assertEqual((failed.x, failed.y), (2.0, 0.0))
        self.assertEqual(failed.path, [])

    def test_planner_avoids_failed_robot_seen_on_floor(self):
        world = World(grid=Grid((".....", ".....")))
        moving = world.add_robot(Robot("R1", Cell(0, 0)))
        failed = world.add_robot(Robot("R2", Cell(2, 0)))
        failed.status = RobotStatus.FAILED
        moving.set_goal(Cell(4, 0))

        world.tick(0.05)

        self.assertIn(failed.cell, moving.known_occupied(world.sim_time))
        self.assertNotIn(failed.cell, moving.path)

    def test_mid_edge_failure_blocks_its_whole_physical_footprint(self):
        world = World(grid=Grid((".....", ".....")))
        moving = world.add_robot(Robot("R1", Cell(0, 0)))
        failed = world.add_robot(Robot("R2", Cell(2, 0)))
        failed.status = RobotStatus.FAILED
        failed.x = 1.875
        moving.sense_robots([(failed.x, failed.y, failed.cell)], now=0.0)
        moving.sense_robots([(failed.x, failed.y, failed.cell)], now=1.1)

        occupied = moving.known_occupied(now=1.1)

        self.assertIn(Cell(2, 0), occupied)
        self.assertIn(Cell(1, 0), occupied)


class TestLateAuctionClaim(unittest.TestCase):
    def test_collected_parcel_is_not_abandoned_for_late_better_bid(self):
        robot = Robot("R1", Cell(0, 0))
        task = Task("T-001", Cell(0, 0), Cell(2, 0),
                    assigned_robot="R1", status=TaskStatus.CARRYING)
        robot.board.add(task)
        robot.task = task
        robot._my_bids[task.task_id] = 10.0

        robot._ingest_task_news(
            TaskClaim("R2", 1.0, 5, task_id=task.task_id,
                      action="CLAIM", cost=1.0),
            now=1.0,
        )

        self.assertIs(robot.task, task)
        self.assertIs(task.status, TaskStatus.CARRYING)
        self.assertEqual(task.assigned_robot, "R1")

    def test_late_claim_does_not_regress_peer_board(self):
        observer = Robot("R3", Cell(4, 0))
        task = Task("T-001", Cell(0, 0), Cell(2, 0),
                    assigned_robot="R1", status=TaskStatus.CARRYING)
        observer.board.add(task)

        observer._ingest_task_news(
            TaskClaim("R2", 1.0, 5, task_id=task.task_id,
                      action="CLAIM", cost=1.0),
            now=1.0,
        )

        self.assertIs(task.status, TaskStatus.CARRYING)
        self.assertEqual(task.assigned_robot, "R1")


class TestSensorOcclusion(unittest.TestCase):
    def test_shelf_blocks_obstacle_detection(self):
        world = World(grid=Grid(("....", ".#..")))
        robot = world.add_robot(Robot("R1", Cell(0, 1)))
        hidden = Cell(3, 1)
        world.add_obstacle(hidden)

        world._run_sensors()

        self.assertNotIn(hidden, robot.blocked_cells(world.sim_time))

    def test_clear_aisle_keeps_obstacle_visible(self):
        world = World(grid=Grid(("....", "....")))
        robot = world.add_robot(Robot("R1", Cell(0, 1)))
        visible = Cell(3, 1)
        world.add_obstacle(visible)

        world._run_sensors()

        self.assertIn(visible, robot.blocked_cells(world.sim_time))


if __name__ == "__main__":
    unittest.main()
