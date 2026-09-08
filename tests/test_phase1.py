"""Phase 1 tests.

Run with:   python3 -m unittest discover -s tests -v

Uses unittest, which is built into Python. (08_SIMULATION suggests pytest --
that needs installing, so unittest is the zero-install stand-in. Same job.)
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "shared"))

from fleetx_core import Cell, Grid, Robot, RobotStatus, World, default_grid, find_path
from fleetx_core.grid import CellKind


class TestGrid(unittest.TestCase):
    def setUp(self):
        self.g = default_grid()

    def test_size(self):
        self.assertEqual((self.g.width, self.g.height), (28, 16))

    def test_shelves_block_robots(self):
        self.assertFalse(self.g.is_walkable(Cell(3, 1)))
        self.assertTrue(self.g.is_walkable(Cell(0, 0)))

    def test_off_the_map_is_not_walkable(self):
        for bad in (Cell(-1, 0), Cell(0, -1), Cell(28, 0), Cell(0, 16)):
            self.assertFalse(self.g.is_walkable(bad))

    def test_neighbours_are_never_diagonal(self):
        for n in self.g.neighbours(Cell(5, 4)):
            self.assertEqual(abs(n.x - 5) + abs(n.y - 4), 1)

    def test_stations_exist(self):
        for kind in (CellKind.PICK, CellKind.DROP, CellKind.CHARGER):
            self.assertTrue(self.g.cells_of_kind(kind), f"no {kind} on the map")

    def test_ragged_map_is_rejected(self):
        with self.assertRaises(ValueError):
            Grid(["...", "...."])


class TestAStar(unittest.TestCase):
    def setUp(self):
        self.g = default_grid()

    def test_route_never_crosses_a_shelf(self):
        route = find_path(self.g, Cell(0, 14), Cell(27, 0))
        self.assertIsNotNone(route)
        for c in route:
            self.assertTrue(self.g.is_walkable(c), f"route goes through blocked square {c}")

    def test_route_has_no_teleporting(self):
        route = find_path(self.g, Cell(0, 14), Cell(27, 0))
        for a, b in zip(route, route[1:]):
            self.assertEqual(abs(a.x - b.x) + abs(a.y - b.y), 1)

    def test_route_starts_and_ends_correctly(self):
        route = find_path(self.g, Cell(2, 0), Cell(20, 12))
        self.assertEqual(route[0], Cell(2, 0))
        self.assertEqual(route[-1], Cell(20, 12))

    def test_shortest_route_in_an_empty_corridor(self):
        route = find_path(self.g, Cell(0, 0), Cell(27, 0))
        self.assertEqual(len(route), 28)

    def test_impossible_goal_returns_none(self):
        self.assertIsNone(find_path(self.g, Cell(0, 0), Cell(3, 1)))

    def test_already_at_goal(self):
        self.assertEqual(find_path(self.g, Cell(5, 0), Cell(5, 0)), [Cell(5, 0)])

    def test_walled_off_goal_returns_none(self):
        tiny = Grid(["..#..", "..#..", "..#.."])
        self.assertIsNone(find_path(tiny, Cell(0, 0), Cell(4, 0)))


class TestRobot(unittest.TestCase):
    def setUp(self):
        self.g = default_grid()
        self.r = Robot("R1", Cell(0, 14))

    def _run_until_idle(self, limit_seconds=200.0):
        dt, t = 0.05, 0.0
        while t < limit_seconds:
            self.r.decide(self.g)
            self.r.advance(dt)
            t += dt
            if self.r.goal is None and self.r.status is RobotStatus.IDLE:
                return t
        self.fail("robot never arrived")

    def test_starts_idle(self):
        self.assertIs(self.r.status, RobotStatus.IDLE)

    def test_reaches_its_goal(self):
        goal = Cell(27, 0)
        self.r.set_goal(goal)
        self._run_until_idle()
        self.assertEqual(self.r.cell, goal)

    def test_never_stands_on_a_shelf_along_the_way(self):
        self.r.set_goal(Cell(27, 0))
        dt = 0.05
        for _ in range(4000):
            self.r.decide(self.g)
            self.r.advance(dt)
            self.assertTrue(self.g.is_walkable(self.r.cell), f"robot stood on {self.r.cell}")
            if self.r.goal is None:
                break

    def test_unreachable_goal_marks_blocked(self):
        self.r.set_goal(Cell(3, 1))
        self.r.decide(self.g)
        self.assertIs(self.r.status, RobotStatus.BLOCKED)

    def test_new_goal_mid_journey_is_obeyed(self):
        self.r.set_goal(Cell(27, 0))
        for _ in range(40):
            self.r.decide(self.g)
            self.r.advance(0.05)
        self.r.set_goal(Cell(13, 15))
        self._run_until_idle()
        self.assertEqual(self.r.cell, Cell(13, 15))

    def test_battery_only_falls(self):
        self.r.set_goal(Cell(27, 0))
        last = self.r.battery
        for _ in range(200):
            self.r.decide(self.g)
            self.r.advance(0.05)
            self.assertLessEqual(self.r.battery, last)
            last = self.r.battery

    def test_ros2_pose_update_keeps_the_plan_honest(self):
        """A real robot reporting back should trim the squares it has passed."""
        self.r.set_goal(Cell(27, 0))
        self.r.decide(self.g)
        third = self.r.path[2]
        before = len(self.r.path)
        self.r.update_pose(float(third.x), float(third.y), third)
        self.assertEqual(self.r.cell, third)
        self.assertEqual(len(self.r.path), before - 3)


class TestWorld(unittest.TestCase):
    def test_phase1_goal_reached_with_zero_collisions(self):
        from fleetx_core import phase1_world
        w = phase1_world()
        r = w.get("R1")
        r.set_goal(Cell(13, 15))
        for _ in range(4000):
            w.tick(0.05)
            if r.goal is None:
                break
        self.assertEqual(r.cell, Cell(13, 15))
        self.assertEqual(w.kpis()["collisions"], 0)

    def test_robot_cannot_start_on_a_shelf(self):
        w = World()
        with self.assertRaises(ValueError):
            w.add_robot(Robot("Rx", Cell(3, 1)))

    def test_duplicate_robot_id_rejected(self):
        w = World()
        w.add_robot(Robot("R1", Cell(0, 0)))
        with self.assertRaises(ValueError):
            w.add_robot(Robot("R1", Cell(1, 0)))

    def test_snapshot_is_plain_json_safe_data(self):
        import json
        from fleetx_core import phase1_world
        w = phase1_world()
        w.tick(0.05)
        json.dumps(w.snapshot())     # must not raise
        json.dumps(w.grid.to_dict())


if __name__ == "__main__":
    unittest.main(verbosity=2)
