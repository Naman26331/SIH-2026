"""Phase 10 tests -- the side-by-side comparison screen.

Run with:   python3 -m unittest discover -s tests -v

The point of this screen is a claim a judge can check: both fleets are doing
the SAME work. So most of these tests are about fairness, not about pictures.
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from comparison import LEFT, RIGHT, Comparison


def run(c, seconds, dt=0.05):
    """Step the comparison forward.

    Note the guard. tick() does nothing while the comparison is paused, so a
    plain "while sim_time < target" loop never finishes if nobody pressed
    Start -- which is exactly what one of the tests below is checking. That
    hung the whole suite for 87 minutes before it was noticed. A test that can
    hang forever is worse than a test that fails.
    """
    steps = int(seconds / dt)
    for _ in range(steps + 10):
        c.tick(dt)
        if c.left.sim_time >= steps * dt:
            return
    if c.running:
        raise AssertionError("comparison did not advance while running")


class TestItIsAFairFight(unittest.TestCase):
    def setUp(self):
        self.c = Comparison(robots=5, every=2.5, seed=1)
        self.c.start()

    def test_the_two_fleets_differ_only_in_coordination(self):
        self.assertEqual(self.c.left.coordination, LEFT)
        self.assertEqual(self.c.right.coordination, RIGHT)
        self.assertFalse(self.c.left.reservations_enabled)
        self.assertTrue(self.c.right.reservations_enabled)

    def test_same_map(self):
        self.assertEqual(self.c.left.grid.to_dict(), self.c.right.grid.to_dict())

    def test_same_start_positions_and_speeds(self):
        self.assertEqual([r.cell for r in self.c.left.robots.values()],
                         [r.cell for r in self.c.right.robots.values()])
        self.assertEqual([r.speed for r in self.c.left.robots.values()],
                         [r.speed for r in self.c.right.robots.values()])

    def test_both_fleets_get_exactly_the_same_orders(self):
        """The whole claim rests on this."""
        run(self.c, 90)
        left = [(t.task_id, t.pickup, t.dropoff, t.product, t.priority)
                for t in self.c.left.board.tasks.values()]
        right = [(t.task_id, t.pickup, t.dropoff, t.product, t.priority)
                 for t in self.c.right.board.tasks.values()]
        self.assertTrue(left, "no orders were issued at all")
        self.assertEqual(left, right)

    def test_an_order_reaches_both_in_the_same_instant(self):
        for _ in range(600):
            self.c.tick(0.05)
            self.assertEqual(len(self.c.left.board), len(self.c.right.board),
                             f"boards differ at t={self.c.left.sim_time:.2f}")

    def test_the_clocks_never_drift(self):
        run(self.c, 60)
        self.assertEqual(self.c.left.sim_time, self.c.right.sim_time)
        self.assertTrue(self.c.snapshot()["in_step"])

    def test_the_order_count_matches_what_was_offered(self):
        run(self.c, 60)
        s = self.c.snapshot()
        self.assertEqual(len(self.c.left.board), s["orders_offered"])
        self.assertEqual(len(self.c.right.board), s["orders_offered"])


class TestNeitherFleetCrashes(unittest.TestCase):
    def test_no_collisions_on_either_side(self):
        c = Comparison(robots=10, every=2.0, seed=3)
        c.start()
        run(c, 200)
        self.assertEqual(c.left.collisions, 0, "stop-and-wait crashed")
        self.assertEqual(c.right.collisions, 0, "FLEET-X crashed")

    def test_no_robot_ever_jumps_on_either_side(self):
        c = Comparison(robots=5, every=2.5, seed=2)
        c.start()
        for _ in range(1200):
            before = {("L", r.robot_id): (r.x, r.y) for r in c.left.robots.values()}
            before.update({("R", r.robot_id): (r.x, r.y) for r in c.right.robots.values()})
            c.tick(0.05)
            for tag, world in (("L", c.left), ("R", c.right)):
                for r in world.robots.values():
                    px, py = before[(tag, r.robot_id)]
                    moved = ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5
                    self.assertLessEqual(moved, r.speed * 0.05 + 1e-9,
                                         f"{tag} {r.robot_id} jumped")


class TestTheStoryItTells(unittest.TestCase):
    def test_fleetx_pulls_ahead_under_load(self):
        """If this ever stops being true, the screen is not worth showing."""
        c = Comparison(robots=10, every=2.0, seed=1)
        c.start()
        run(c, 150)
        s = c.snapshot()
        self.assertGreater(s["right"]["delivered"], s["left"]["delivered"] * 2,
                           f"only {s['right']['delivered']} vs {s['left']['delivered']}")

    def test_nothing_happens_until_you_press_start(self):
        """A fixed number of ticks, NOT a run-until-the-clock-reaches-N loop.
        Paused means the clock never moves, so that loop would never end."""
        c = Comparison(robots=5, every=2.5, seed=1)
        for _ in range(600):
            c.tick(0.05)
        self.assertEqual(c.orders_offered, 0)
        self.assertEqual(len(c.left.board), 0)
        self.assertEqual(c.left.sim_time, 0.0)

    def test_pause_stops_both(self):
        c = Comparison(robots=5, every=2.5, seed=1)
        c.start()
        run(c, 20)
        c.pause()
        before = (c.left.sim_time, c.orders_offered)
        for _ in range(200):
            c.tick(0.05)
        self.assertEqual((c.left.sim_time, c.orders_offered), before)

    def test_reset_clears_both_sides(self):
        c = Comparison(robots=5, every=2.5, seed=1)
        c.start()
        run(c, 40)
        c.reset()
        self.assertEqual(c.orders_offered, 0)
        self.assertEqual(len(c.left.board), 0)
        self.assertEqual(len(c.right.board), 0)
        self.assertEqual(c.left.sim_time, 0.0)

    def test_changing_the_fleet_size_rebuilds_both_sides_equally(self):
        c = Comparison(robots=5, every=2.5, seed=1)
        c.reset(robots=10)
        self.assertEqual(len(c.left.robots), 10)
        self.assertEqual(len(c.right.robots), 10)
        self.assertEqual([r.cell for r in c.left.robots.values()],
                         [r.cell for r in c.right.robots.values()])

    def test_speed_only_takes_sensible_values(self):
        c = Comparison()
        for bad in (0, 3, 99, -1):
            c.set_speed(bad)
            self.assertEqual(c.speed, 1)
        c.set_speed(4)
        self.assertEqual(c.speed, 4)

    def test_snapshot_is_json_safe(self):
        c = Comparison(robots=5, every=2.5, seed=1)
        c.start()
        run(c, 30)
        json.dumps(c.snapshot())


if __name__ == "__main__":
    unittest.main(verbosity=2)
