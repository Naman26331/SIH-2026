"""Phase 2 tests -- three robots, and an honest collision counter.

Run with:   python3 -m unittest discover -s tests -v

The collision counter is the single most important number in this project
(00_README: "0 inter-robot collisions"). If it lies, the whole benchmark is
worthless. These tests exist to stop it lying.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, Robot, RobotStatus, World, phase2_world
from scenarios import Scenarios



# Phase 5 added square booking, which is what stops the crashes. Tests below
# that are about the EARLIER behaviour (robots driving blind) now switch it
# off explicitly with reservations_enabled=False. That is not a workaround --
# Phase 15 runs the "before" side of the benchmark exactly the same way.


def blind(world):
    """Turn booking off, to test the pre-Phase-5 behaviour."""
    world.reservations_enabled = False
    return world


def run(world, seconds, dt=0.05, scenarios=None):
    for _ in range(int(seconds / dt)):
        if scenarios:
            scenarios.keep_busy()
        world.tick(dt)


class TestThreeRobots(unittest.TestCase):
    def test_phase2_world_has_three_robots_apart(self):
        w = phase2_world()
        self.assertEqual(len(w.robots), 3)
        cells = [r.cell for r in w.robots.values()]
        self.assertEqual(len(set(cells)), 3, "robots must start on different squares")
        for r in w.robots.values():
            self.assertTrue(w.grid.is_walkable(r.cell))

    def test_all_three_can_move_at_once(self):
        w = phase2_world()
        goals = {"R1": Cell(26, 12), "R2": Cell(2, 12), "R3": Cell(13, 12)}
        for rid, g in goals.items():
            w.get(rid).set_goal(g)
        w.tick(0.05)
        self.assertEqual(sum(1 for r in w.robots.values() if r.status is RobotStatus.MOVING), 3)

    def test_each_robot_reaches_its_own_goal(self):
        """Phase 2's success test: three robots navigating simultaneously."""
        w = blind(phase2_world())
        goals = {"R1": Cell(26, 12), "R2": Cell(2, 12), "R3": Cell(13, 12)}
        for rid, g in goals.items():
            w.get(rid).set_goal(g)
        run(w, 60)
        for rid, g in goals.items():
            self.assertEqual(w.get(rid).cell, g, f"{rid} never arrived")


class TestCollisionCounting(unittest.TestCase):
    """The counter must not over-count, under-count, or invent crashes."""

    def test_one_crash_counts_once_not_once_per_tick(self):
        """The world ticks 20x a second. A crash must still count as 1."""
        w = blind(World())
        a = w.add_robot(Robot("R1", Cell(10, 8), speed=2.0))
        b = w.add_robot(Robot("R2", Cell(16, 8), speed=2.0))
        a.set_goal(Cell(16, 8))
        b.set_goal(Cell(10, 8))
        run(w, 15)
        self.assertEqual(w.collisions, 1, "one head-on crash should count exactly once")

    def test_head_on_swap_is_caught(self):
        """Two robots trading places never share a square.

        Checking squares alone would miss the most obvious crash there is.
        See 05_PATH_PLANNING section 5 -- "Do not check only nodes."
        """
        w = blind(World())
        a = w.add_robot(Robot("R1", Cell(10, 8), speed=1.0))
        b = w.add_robot(Robot("R2", Cell(11, 8), speed=1.0))
        a.set_goal(Cell(11, 8))
        b.set_goal(Cell(10, 8))
        w.tick(1.0)                       # one big step: they swap in one go
        self.assertEqual(a.cell, Cell(11, 8))
        self.assertEqual(b.cell, Cell(10, 8))
        self.assertEqual(w.collisions, 1)
        self.assertEqual(w.collision_events[0]["reason"], "head-on swap")

    def test_same_square_is_caught(self):
        w = blind(World())
        w.add_robot(Robot("R1", Cell(10, 8)))
        w.add_robot(Robot("R2", Cell(10, 8)))   # placed on top of each other
        w.tick(0.05)
        self.assertEqual(w.collisions, 1)

    def test_robots_far_apart_never_collide(self):
        """No phantom crashes. Two robots on opposite sides, both busy."""
        w = World()
        a = w.add_robot(Robot("R1", Cell(2, 0)))
        b = w.add_robot(Robot("R2", Cell(2, 12)))
        a.set_goal(Cell(26, 0))
        b.set_goal(Cell(26, 12))
        run(w, 40)
        self.assertEqual(w.collisions, 0)

    def test_one_robot_alone_can_never_collide(self):
        w = World()
        r = w.add_robot(Robot("R1", Cell(2, 8)))
        r.set_goal(Cell(26, 8))
        run(w, 40)
        self.assertEqual(w.collisions, 0)

    def test_separating_then_crashing_again_counts_twice(self):
        """Two separate crashes are two crashes, not one."""
        w = blind(World())
        a = w.add_robot(Robot("R1", Cell(10, 8), speed=2.0))
        b = w.add_robot(Robot("R2", Cell(16, 8), speed=2.0))
        a.set_goal(Cell(16, 8)); b.set_goal(Cell(10, 8))
        run(w, 15)
        first = w.collisions
        a.place(Cell(10, 8)); b.place(Cell(16, 8))      # reset them apart
        run(w, 2)
        a.set_goal(Cell(16, 8)); b.set_goal(Cell(10, 8))
        run(w, 15)
        self.assertEqual(w.collisions, first + 1)

    def test_collision_event_records_who_where_and_why(self):
        w = blind(World())
        a = w.add_robot(Robot("R1", Cell(10, 8), speed=2.0))
        b = w.add_robot(Robot("R2", Cell(16, 8), speed=2.0))
        a.set_goal(Cell(16, 8)); b.set_goal(Cell(10, 8))
        run(w, 15)
        e = w.collision_events[0]
        self.assertEqual(e["robots"], ["R1", "R2"])
        self.assertIn(e["reason"], ("touching", "same square", "head-on swap"))
        self.assertAlmostEqual(e["y"], 8.0, places=1)

    def test_failed_robot_still_has_a_physical_body(self):
        w = blind(World())
        dead = w.add_robot(Robot("R1", Cell(10, 8)))
        dead.status = RobotStatus.FAILED
        w.add_robot(Robot("R2", Cell(10, 8)))
        w.tick(0.05)
        self.assertEqual(w.collisions, 1)

    def test_reset_counters_clears_everything(self):
        w = World()
        w.add_robot(Robot("R1", Cell(10, 8)))
        w.add_robot(Robot("R2", Cell(10, 8)))
        w.tick(0.05)
        self.assertGreater(w.collisions, 0)
        w.reset_counters()
        self.assertEqual(w.collisions, 0)
        self.assertEqual(w.collision_events, [])
        self.assertEqual(w.sim_time, 0.0)


class TestScenarios(unittest.TestCase):
    """The demo setups must actually produce the crash they promise."""

    def test_head_on_produces_a_collision(self):
        w = blind(phase2_world()); sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, scenarios=sc)
        self.assertGreaterEqual(w.collisions, 1)

    def test_intersection_makes_all_three_pairs_collide(self):
        w = blind(phase2_world()); sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 25, scenarios=sc)
        pairs = {tuple(e["robots"]) for e in w.collision_events}
        self.assertEqual(len(pairs), 3, f"expected all 3 pairs to crash, got {pairs}")

    def test_free_roam_eventually_crashes(self):
        """This is the number Phase 5 drives to zero."""
        w = blind(phase2_world()); sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 60, scenarios=sc)
        self.assertGreater(w.collisions, 0)

    def test_stop_parks_everyone(self):
        """Stop all must actually stop them.

        This used to assert that every robot's goal was None, which sounds
        right and guaranteed nothing: clearing the destination never stopped a
        robot that was carrying something, because it hands itself the same
        destination straight back. The button said "all robots parked" while
        the fleet drove on. So ask the question that matters instead -- did
        anything move? -- which the old wording could not catch.

        A stopped robot now KEEPS its goal on purpose, so you can see what it
        was in the middle of and so it can pick the job back up on release.
        """
        w = phase2_world(); sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 5, scenarios=sc)
        sc.apply("stop")
        where = {rid: (r.x, r.y) for rid, r in w.robots.items()}
        run(w, 5, scenarios=sc)
        for rid, r in w.robots.items():
            self.assertTrue(r.halted, f"{rid} was not halted")
            self.assertEqual((r.x, r.y), where[rid], f"{rid} moved after Stop all")

    def test_unknown_scenario_is_refused_politely(self):
        w = phase2_world(); sc = Scenarios(w)
        self.assertIn("no scenario", sc.apply("nonsense"))

    def test_snapshot_still_json_safe(self):
        import json
        w = phase2_world(); sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 10, scenarios=sc)
        json.dumps(w.snapshot())


if __name__ == "__main__":
    unittest.main(verbosity=2)
