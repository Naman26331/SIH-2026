"""Phase 8 tests -- things that should not be there.

Run with:   python3 -m unittest discover -s tests -v

The important one is test_nobody_knows_until_a_robot_gets_close. If robots
magically knew the instant an obstacle appeared, the broadcast would be
pointless and none of this would be decentralised.
"""

import json
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (BlockedAisle, BlockedMap, Cell, RobotStatus,
                         default_grid, find_path, from_dict, phase2_world,
                         within_range)
from fleetx_core.grid import CellKind
from scenarios import Scenarios

TOUCHING = 0.7


def run(world, seconds, sc=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)


def posed_world():
    """Three robots, nobody patrolling, so a test can place them by hand."""
    w = phase2_world()
    sc = Scenarios(w)
    sc.auto = False
    for r in w.robots.values():
        r.clear_goal()
    return w, sc


class TestTheBlockedMap(unittest.TestCase):
    def test_a_marked_square_is_blocked(self):
        m = BlockedMap()
        m.mark(Cell(5, 5), "R1", now=0.0)
        self.assertTrue(m.is_blocked(Cell(5, 5), 0.0))

    def test_a_block_fades_if_nobody_confirms_it(self):
        """Somebody picks the box up. Without this the warehouse would carry a
        phantom wall for ever."""
        m = BlockedMap(ttl=10.0)
        m.mark(Cell(5, 5), "R1", now=0.0)
        self.assertTrue(m.is_blocked(Cell(5, 5), 9.0))
        self.assertFalse(m.is_blocked(Cell(5, 5), 11.0))

    def test_seeing_it_again_keeps_it_alive(self):
        m = BlockedMap(ttl=10.0)
        m.mark(Cell(5, 5), "R1", now=0.0)
        m.mark(Cell(5, 5), "R1", now=9.0)
        self.assertTrue(m.is_blocked(Cell(5, 5), 15.0))

    def test_expire_reports_what_it_forgot(self):
        m = BlockedMap(ttl=5.0)
        m.mark(Cell(1, 1), "R1", now=0.0)
        self.assertEqual(m.expire(now=10.0), [Cell(1, 1)])
        self.assertEqual(len(m), 0)

    def test_it_can_say_whether_a_route_is_affected(self):
        m = BlockedMap()
        m.mark(Cell(3, 0), "R1", now=0.0)
        route = [Cell(1, 0), Cell(2, 0), Cell(3, 0), Cell(4, 0)]
        self.assertEqual(m.blocks_any(route, 0.0), Cell(3, 0))
        self.assertIsNone(m.blocks_any([Cell(9, 9)], 0.0))

    def test_sensor_range_is_short(self):
        self.assertTrue(within_range(10.0, 8.0, Cell(12, 8), 3.0))
        self.assertFalse(within_range(10.0, 8.0, Cell(20, 8), 3.0))


class TestRoutingRoundThings(unittest.TestCase):
    def test_an_obstacle_is_impassable_not_merely_expensive(self):
        """Different from a busy square. You can push through a busy aisle if
        you must; you cannot drive through a pallet because you are in a hurry."""
        g = default_grid()
        wall = {Cell(13, 8), Cell(14, 8)}
        route = find_path(g, Cell(2, 8), Cell(26, 8), blocked=wall)
        self.assertIsNotNone(route)
        self.assertFalse(wall & set(route), "route went through the obstacle")

    def test_going_round_costs_more(self):
        g = default_grid()
        direct = find_path(g, Cell(2, 8), Cell(26, 8))
        around = find_path(g, Cell(2, 8), Cell(26, 8), blocked={Cell(13, 8)})
        self.assertGreater(len(around), len(direct))

    def test_a_walled_off_goal_returns_no_route(self):
        g = default_grid()
        blocked = {Cell(1, 0), Cell(0, 1), Cell(1, 1)}
        self.assertIsNone(find_path(g, Cell(3, 0), Cell(0, 0), blocked=blocked))

    def test_a_goal_with_something_on_it_returns_no_route(self):
        g = default_grid()
        self.assertIsNone(find_path(g, Cell(2, 8), Cell(13, 8),
                                    blocked={Cell(13, 8)}))


class TestFindingThings(unittest.TestCase):
    def test_nobody_knows_until_a_robot_gets_close(self):
        """THE point of the phase. If everyone magically knew, the broadcast
        would be pointless and none of this would be decentralised."""
        w, sc = posed_world()
        w.get("R1").place(Cell(2, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        w.add_obstacle(Cell(20, 8))
        run(w, 1.0, sc)
        for r in w.robots.values():
            self.assertEqual(len(r.blocked_cells(w.sim_time)), 0,
                             f"{r.robot_id} knew about it without going near")

    def test_a_robot_finds_it_by_driving_near(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(2, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        w.add_obstacle(Cell(13, 8))
        w.get("R1").set_goal(Cell(26, 8))
        run(w, 30, sc)
        self.assertGreater(w.get("R1").obstacles_found, 0)

    def test_finding_it_warns_everybody(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(11, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        w.add_obstacle(Cell(13, 8))
        run(w, 2.0, sc)
        for r in w.robots.values():
            self.assertIn(Cell(13, 8), r.blocked_cells(w.sim_time),
                          f"{r.robot_id} was never told")

    def test_it_gets_there_anyway(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(2, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        w.add_obstacle(Cell(13, 8))
        w.get("R1").set_goal(Cell(26, 8))
        run(w, 90, sc)
        self.assertEqual(w.get("R1").cell, Cell(26, 8))
        self.assertEqual(w.collisions, 0)

    def test_a_robot_never_drives_onto_an_obstacle(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(2, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        for cell in (Cell(13, 8), Cell(19, 8), Cell(8, 4)):
            w.add_obstacle(cell)
        w.get("R1").set_goal(Cell(26, 8))
        w.get("R2").set_goal(Cell(26, 4))
        for _ in range(2400):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                self.assertNotIn(r.cell, w.obstacles,
                                 f"{r.robot_id} drove onto an obstacle "
                                 f"at t={w.sim_time:.2f}")

    def test_seeing_it_gone_tells_everybody_too(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(11, 8))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 0))
        w.add_obstacle(Cell(13, 8))
        run(w, 1.0, sc)
        self.assertIn(Cell(13, 8), w.get("R3").blocked_cells(w.sim_time))
        w.remove_obstacle(Cell(13, 8))
        run(w, 1.0, sc)
        self.assertNotIn(Cell(13, 8), w.get("R3").blocked_cells(w.sim_time),
                         "R3 was never told the box had gone")

    def test_a_walled_in_robot_says_blocked_rather_than_pretending(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(2, 0))
        w.get("R2").place(Cell(2, 12))
        w.get("R3").place(Cell(2, 4))
        for c in (Cell(0, 12), Cell(1, 12), Cell(2, 13), Cell(2, 14),
                  Cell(2, 15), Cell(1, 13), Cell(1, 15)):
            w.add_obstacle(c)
        w.get("R1").set_goal(Cell(0, 14))
        # Give it time. Since Phase 7 a robot first tries asking whatever is in
        # its way to move, so it takes longer to conclude there is genuinely no
        # route at all -- which is the right order to try things in.
        run(w, 200, sc)
        self.assertIs(w.get("R1").status, RobotStatus.BLOCKED)
        self.assertNotEqual(w.get("R1").cell, Cell(0, 14),
                            "claimed to reach a goal that is walled off")
        self.assertEqual(w.collisions, 0)

    def test_you_cannot_drop_a_box_on_a_robot(self):
        w, _ = posed_world()
        w.get("R1").place(Cell(5, 8))
        result = w.add_obstacle(Cell(5, 8))
        self.assertFalse(result["ok"])
        self.assertIn("standing there", result["message"])

    def test_you_cannot_drop_a_box_into_a_shelf(self):
        w, _ = posed_world()
        self.assertFalse(w.add_obstacle(Cell(3, 1))["ok"])


class TestTheMessage(unittest.TestCase):
    def test_blocked_aisle_survives_a_round_trip(self):
        m = BlockedAisle("R1", 1.0, 3, cell=(13, 8), confidence=0.96, ttl=15.0)
        back = from_dict(json.loads(json.dumps(m.to_dict())))
        self.assertEqual(back.cell, (13, 8))
        self.assertAlmostEqual(back.confidence, 0.96)
        self.assertAlmostEqual(back.ttl, 15.0)

    def test_it_goes_out_on_the_radio(self):
        """Caught early: bus.recent is a short ring buffer for the dashboard
        ticker, and position updates flush it within a second."""
        w, sc = posed_world()
        w.get("R1").place(Cell(11, 8))
        w.add_obstacle(Cell(13, 8))
        alerts = []
        for _ in range(8):
            sc.keep_busy()
            w.tick(0.05)
            alerts += [m for m in w.bus.recent if isinstance(m, BlockedAisle)]
        self.assertTrue(alerts, "no BLOCKED_AISLE was ever broadcast")
        self.assertEqual(alerts[0].cell, (13, 8))
        self.assertFalse(alerts[0].cleared)


class TestNothingElseBroke(unittest.TestCase):
    def test_no_collisions_with_obstacles_scattered_about(self):
        rng = random.Random(7)
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        floor = w.grid.cells_of_kind(CellKind.FLOOR)
        worst = 99.0
        nxt = 5.0
        for _ in range(4800):
            if w.sim_time >= nxt:
                nxt = w.sim_time + rng.uniform(6, 14)
                if rng.random() < 0.5:
                    w.add_obstacle(rng.choice(floor))
                else:
                    w.clear_obstacles()
            sc.keep_busy()
            w.tick(0.05)
            rs = list(w.robots.values())
            for i in range(len(rs)):
                for j in range(i + 1, len(rs)):
                    a, b = rs[i], rs[j]
                    worst = min(worst, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)
            self.assertGreater(worst, TOUCHING,
                               f"robots overlapped at t={w.sim_time:.2f}")
        self.assertEqual(w.collisions, 0)
        self.assertEqual(w.kpis()["deadlocked"], 0)

    def test_free_roam_is_unaffected_when_the_floor_is_clear(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 120, sc)
        k = w.kpis()
        self.assertEqual(k["collisions"], 0)
        self.assertEqual(k["obstacles"], 0)
        self.assertGreater(k["total_distance"], 700)

    def test_snapshot_still_json_safe(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        w.add_obstacle(Cell(13, 8))
        run(w, 20, sc)
        json.dumps(w.snapshot())

    def test_reset_clears_what_robots_believed(self):
        w, sc = posed_world()
        w.get("R1").place(Cell(11, 8))
        w.add_obstacle(Cell(13, 8))
        run(w, 2, sc)
        w.reset_counters()
        for r in w.robots.values():
            self.assertEqual(len(r.blocked_cells(0.0)), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
