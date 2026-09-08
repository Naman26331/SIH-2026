"""The win condition: ZERO collisions, with robots still doing their work.

Run with:   python3 -m unittest discover -s tests -v

00_README: "0 inter-robot collisions". Every test here exists because of a bug
that actually let two robots touch. They are named after the cause so nobody
quietly reintroduces one.

Note these tests check the GAP between robots, not just the collision counter.
Checking the counter alone would pass if the counter itself were broken.
"""

import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (Cell, InMemoryBus, Robot, RobotStatus, World,
                         fleet_world, phase2_world, yields_to)
from fleetx_core.grid import CellKind
from scenarios import Scenarios

# Two robots are about 0.68 squares wide together. Anything under this and
# they are overlapping on screen.
TOUCHING = 0.7


def closest_gap(world, worst=None):
    robots = list(world.robots.values())
    smallest = worst if worst is not None else 99.0
    for i in range(len(robots)):
        for j in range(i + 1, len(robots)):
            a, b = robots[i], robots[j]
            smallest = min(smallest, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)
    return smallest


class TestTheYieldingRule(unittest.TestCase):
    """The rule two robots use to decide who goes. It must never say
    'both go' -- and must never say 'both stop' either."""

    def test_two_robots_never_both_go(self):
        for pa in range(40, 120, 3):
            for pb in range(40, 120, 3):
                a = yields_to(pa, "R1", pb, "R2")
                b = yields_to(pb, "R2", pa, "R1")
                self.assertNotEqual(a, b,
                                    f"both go or both stop at {pa} vs {pb}")

    def test_a_stale_reading_cannot_flip_the_answer(self):
        """Each robot reads the other's score a fraction of a second late, so
        the numbers they compare are never quite the same. The answer must not
        depend on that."""
        for drift in range(0, 10):          # up to 0.9 of a point out of date
            self.assertFalse(yields_to(50, "R1", 50 + drift, "R2"),
                             f"a drift of {drift} tenths flipped the answer")

    def test_a_clear_lead_does_decide_it(self):
        self.assertTrue(yields_to(50, "R1", 80, "R2"),
                        "a robot 3 points ahead should win despite the name")


class TestNoTeleporting(unittest.TestCase):
    """A robot may never move further in one tick than its wheels allow.

    This is THE invariant. All three collisions this project has had were
    teleports: a robot jumped across a gap and through another robot, and
    nothing predicted it because nothing predicts teleportation. Checking
    positions every tick catches the whole class in one test, instead of
    finding them one crash at a time.
    """

    def _assert_no_jumps(self, world, sc, ticks, label):
        for _ in range(ticks):
            before = {r.robot_id: (r.x, r.y) for r in world.robots.values()}
            if sc:
                sc.keep_busy()
            world.tick(0.05)
            for r in world.robots.values():
                px, py = before[r.robot_id]
                moved = ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5
                limit = r.speed * 0.05 + 1e-9
                self.assertLessEqual(
                    moved, limit,
                    f"{label}: {r.robot_id} moved {moved:.3f} squares in one tick "
                    f"at t={world.sim_time:.2f} (wheels allow {limit:.3f})")

    def test_no_robot_ever_jumps_while_running_orders(self):
        for robots in (3, 10, 20):
            w = fleet_world(robots)
            sc = Scenarios(w)
            sc.apply("orders", seed=7, every=4.0, limit=None)
            self._assert_no_jumps(w, sc, 2000, f"{robots} robots")

    def test_no_robot_ever_jumps_while_being_sent_about(self):
        rng = random.Random(11)
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        floor = w.grid.cells_of_kind(CellKind.FLOOR)
        for _ in range(2000):
            if rng.random() < 0.04:
                w.get(rng.choice(list(w.robots))).set_goal(rng.choice(floor))
            before = {r.robot_id: (r.x, r.y) for r in w.robots.values()}
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                px, py = before[r.robot_id]
                moved = ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5
                self.assertLessEqual(moved, r.speed * 0.05 + 1e-9,
                                     f"{r.robot_id} jumped at t={w.sim_time:.2f}")

    def test_no_robot_ever_jumps_with_obstacles_appearing(self):
        rng = random.Random(12)
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", seed=3, every=4.0, limit=None)
        floor = w.grid.cells_of_kind(CellKind.FLOOR)
        for _ in range(2000):
            if rng.random() < 0.01:
                w.add_obstacle(rng.choice(floor))
            if rng.random() < 0.005:
                w.clear_obstacles()
            before = {r.robot_id: (r.x, r.y) for r in w.robots.values()}
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                px, py = before[r.robot_id]
                moved = ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5
                self.assertLessEqual(moved, r.speed * 0.05 + 1e-9,
                                     f"{r.robot_id} jumped at t={w.sim_time:.2f}")


    def test_changing_destination_mid_move_does_not_jump_backwards(self):
        """A robot part way along a segment cannot be snapped back onto the
        square behind it -- that jumped it straight through safety checks."""
        w = World(bus=InMemoryBus(seed=1))
        r = w.add_robot(Robot("R1", Cell(10, 8)))
        r.set_goal(Cell(20, 8))
        for _ in range(9):
            w.tick(0.05)
        before = r.x
        r.set_goal(Cell(10, 12))
        for _ in range(4):
            w.tick(0.05)
            self.assertGreaterEqual(r.x + 1e-9, before,
                                    "robot teleported backwards")

    def test_restarting_the_clock_clears_every_booking(self):
        """Bookings hold absolute times. Any that survive a clock reset claim
        squares for a time that will never come again, and the warehouse
        gums up solid."""
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
        w.reset_counters()
        for r in w.robots.values():
            self.assertEqual(len(r.table), 0, f"{r.robot_id} kept stale bookings")
        for _ in range(1200):
            sc.keep_busy()
            w.tick(0.05)
        self.assertGreater(w.kpis()["total_distance"], 100,
                           "the fleet did not recover after a clock reset")

    def test_staging_a_scenario_never_stacks_two_robots(self):
        w = phase2_world()
        sc = Scenarios(w)
        for name in ("head_on", "intersection", "patrol", "head_on", "intersection"):
            sc.apply(name)
            cells = [r.cell for r in w.robots.values()]
            self.assertEqual(len(set(cells)), len(cells),
                             f"{name} put two robots on one square")


class TestAStoppedRobotSaysSo(unittest.TestCase):
    def test_a_held_robot_reports_zero_speed(self):
        """It used to report full speed whenever it had a route. Everyone else
        then dead-reckoned it creeping into squares it was not entering, and
        two stopped robots blocked each other for ever."""
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        seen_stopped = False
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.hold:
                    seen_stopped = True
                    self.assertEqual(r.current_velocity(), 0.0,
                                     f"{r.robot_id} is held but claims to be moving")
        self.assertTrue(seen_stopped, "nobody was ever held, test proves nothing")


class TestNobodyEverTouches(unittest.TestCase):
    """The headline. Gaps are measured every single tick."""

    def _assert_clean(self, world, sc, ticks, label):
        worst = 99.0
        for _ in range(ticks):
            sc.keep_busy()
            world.tick(0.05)
            worst = closest_gap(world, worst)
            self.assertGreater(worst, TOUCHING,
                               f"{label}: robots overlapped at t={world.sim_time:.2f}")
        self.assertEqual(world.collisions, 0, f"{label}: collisions counted")
        return worst

    def test_head_on_ten_times(self):
        for run in range(10):
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("head_on")
            self._assert_clean(w, sc, 1200, f"head_on run {run}")

    def test_intersection_ten_times(self):
        for run in range(10):
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("intersection")
            self._assert_clean(w, sc, 1200, f"intersection run {run}")

    def test_free_roam_for_five_minutes(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        self._assert_clean(w, sc, 6000, "free roam")
        self.assertGreater(w.kpis()["total_distance"], 1500,
                           "no collisions, but the robots barely moved")

    def test_free_roam_while_somebody_keeps_clicking(self):
        """Sending robots to new places at random, which is what a person
        does to a demo. This is how the last two collisions were found."""
        for seed in range(4):
            rng = random.Random(300 + seed)
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("patrol")
            floor = w.grid.cells_of_kind(CellKind.FLOOR)
            nxt = 0.0
            worst = 99.0
            for _ in range(3600):
                if w.sim_time >= nxt:
                    nxt = w.sim_time + rng.choice([0.8, 1.5, 3.0])
                    w.get(rng.choice(list(w.robots))).set_goal(rng.choice(floor))
                sc.keep_busy()
                w.tick(0.05)
                worst = closest_gap(w, worst)
                self.assertGreater(worst, TOUCHING,
                                   f"seed {seed}: overlapped at t={w.sim_time:.2f}")
            self.assertEqual(w.collisions, 0, f"seed {seed}")
            # Typical range over 12 seeds is 785-1295 squares. This floor is
            # a "did they keep working at all" check, not a target.
            self.assertGreater(w.kpis()["total_distance"], 600,
                               f"seed {seed}: robots stopped working")

    def test_switching_scenarios_mid_run(self):
        """Somebody pressing the demo buttons at random moments."""
        for seed in range(4):
            rng = random.Random(400 + seed)
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("patrol")
            nxt = 0.0
            worst = 99.0
            for _ in range(2400):
                if w.sim_time >= nxt:
                    nxt = w.sim_time + rng.uniform(3, 9)
                    sc.apply(rng.choice(["patrol", "head_on", "intersection", "patrol"]))
                sc.keep_busy()
                w.tick(0.05)
                worst = closest_gap(w, worst)
                self.assertGreater(worst, TOUCHING,
                                   f"seed {seed}: overlapped at t={w.sim_time:.2f}")
            self.assertEqual(w.collisions, 0, f"seed {seed}")

    def test_still_clean_on_a_lossy_network(self):
        """Up to a quarter of messages thrown away. Beyond that, robots cannot
        respect bookings they never heard -- see the note in PROGRESS_NOTES."""
        for loss in (0.1, 0.25):
            w = phase2_world()
            w.bus = InMemoryBus(packet_loss=loss, latency=0.06, jitter=0.04, seed=5)
            for rid in w.robots:
                w.bus.register(rid)
            sc = Scenarios(w)
            sc.apply("patrol")
            self._assert_clean(w, sc, 3600, f"{int(loss*100)}% packet loss")
            self.assertGreater(w.bus.dropped, 0, "nothing was actually lost")


if __name__ == "__main__":
    unittest.main(verbosity=2)
