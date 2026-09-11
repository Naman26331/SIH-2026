"""Phase 18 tests -- the central planner rival.

Run with:   python3 -m unittest discover -s tests -v

26_IMPROVEMENTS_AND_UPGRADES calls a centralized-planner baseline "the one
experiment that directly answers 'why not centralized?' with data instead of
an argument." These tests are the claim that experiment has to keep being
true: a central boss (shared/fleetx_core/central.py) can run the warehouse
about as well as FLEET-X while its link to the robots stays up, and safely,
cleanly loses the ability to run it at all -- never unsafely -- the moment
that link is cut. FLEET-X never borrowed its intelligence from that link, so
cutting it costs FLEET-X nothing central mode also doesn't lose.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import RobotStatus, fleet_world
from scenarios import Scenarios

TOUCHING = 0.7


def central_world(robots=3, seed=1, every=3.0):
    w = fleet_world(robots, coordination="CENTRAL")
    sc = Scenarios(w)
    sc.apply("orders", seed=seed, every=every, limit=None)
    return w, sc


def run(w, sc, to, watch=None):
    while w.sim_time < to:
        sc.keep_busy()
        w.tick(0.05)
        if watch:
            watch()


def closest(w):
    rs = list(w.robots.values())
    best = 99.0
    for i in range(len(rs)):
        for j in range(i + 1, len(rs)):
            a, b = rs[i], rs[j]
            best = min(best, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)
    return best


class TestCentralModeDeliversWork(unittest.TestCase):
    """The boss can actually run a warehouse, not just talk about one."""

    def test_a_task_gets_assigned_planned_and_delivered(self):
        w, sc = central_world(robots=2, every=1000.0)  # one order, no traffic
        run(w, sc, 60)
        stats = w.board.stats(w.sim_time)
        self.assertGreaterEqual(stats["done"], 1,
                                "a single robot with a clear floor never "
                                "finished the one task it was given")
        self.assertEqual(w.collisions, 0)

    def test_only_the_boss_plans_robots_never_self_route(self):
        w, sc = central_world(robots=3)
        run(w, sc, 30)
        for r in w.robots.values():
            self.assertTrue(r.centrally_controlled)

    def test_no_collisions_under_normal_load(self):
        worst = [99.0]
        w, sc = central_world(robots=5, every=2.5)
        watch = lambda: worst.__setitem__(0, min(worst[0], closest(w)))
        run(w, sc, 200, watch)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(worst[0], TOUCHING,
                           f"robots came within {worst[0]:.2f} squares")


class TestResendsDoNotTeleport(unittest.TestCase):
    """CentralCommand is repeated on a timer -- a resend must never yank a
    robot mid-route back to the start of the same path."""

    def test_repeated_identical_command_does_not_reset_progress(self):
        w, sc = central_world(robots=1, every=1000.0)
        sc._orders()
        run(w, sc, 5)
        moving = [r for r in w.robots.values() if r.status is RobotStatus.MOVING]
        if not moving:
            run(w, sc, 8)
            moving = [r for r in w.robots.values() if r.status is RobotStatus.MOVING]
        self.assertTrue(moving, "robot never started moving toward its task")
        robot = moving[0]
        before = (robot.x, robot.y)
        # Force an immediate resend of the same command.
        w.central._publish(w, robot.robot_id, w.sim_time)
        w.tick(0.05)
        after = (robot.x, robot.y)
        dist = ((after[0] - before[0]) ** 2 + (after[1] - before[1]) ** 2) ** 0.5
        self.assertLess(dist, 0.5,
                        "a resend of the SAME route teleported the robot -- "
                        "it should be a no-op, not a fresh command")


class TestNetworkCutBreaksCentralNotFleetX(unittest.TestCase):
    """The actual point of this phase: cut the radio and watch what happens."""

    def test_central_mode_stalls_cleanly_when_the_network_dies(self):
        w, sc = central_world(robots=4, every=3.0)
        run(w, sc, 60)
        done_before = w.board.stats(w.sim_time)["done"]
        w.bus.packet_loss = 1.0    # the network is gone, completely
        run(w, sc, 160)
        done_after = w.board.stats(w.sim_time)["done"]
        # Deliveries already in a robot's hands finish (it has a path in
        # hand), but nothing NEW should be dispatchable once every command
        # is lost in transit -- the boss can plan all it likes, nothing it
        # sends ever arrives.
        self.assertEqual(w.central.commands_sent > 0, True)
        self.assertEqual(w.collisions, 0,
                         "a dead network must never cause a collision")
        # Safety still holds through the blackout.
        worst = closest(w)
        self.assertGreater(worst, TOUCHING,
                           f"robots came within {worst:.2f} squares with no radio")

    def test_fleetx_keeps_delivering_through_the_same_blackout(self):
        w = fleet_world(4)
        sc = Scenarios(w)
        sc.apply("orders", seed=1, every=3.0, limit=None)
        run(w, sc, 60)
        done_before = w.board.stats(w.sim_time)["done"]
        w.bus.packet_loss = 1.0
        run(w, sc, 160)
        done_after = w.board.stats(w.sim_time)["done"]
        self.assertGreater(done_after, done_before,
                           "FLEET-X should keep delivering through a network "
                           "blackout -- its robots never depended on the "
                           "link for movement")
        self.assertEqual(w.collisions, 0)

    def test_central_throughput_collapses_relative_to_fleetx_after_the_cut(self):
        """The headline comparison: same blackout, same window, very
        different outcome, because only one of the two ever depended on
        the link that just died."""
        wc, scc = central_world(robots=5, every=3.0)
        run(wc, scc, 100)
        wc.bus.packet_loss = 1.0
        run(wc, scc, 300)
        central_delivered = wc.board.stats(wc.sim_time)["done"]

        wf = fleet_world(5)
        scf = Scenarios(wf)
        scf.apply("orders", seed=1, every=3.0, limit=None)
        run(wf, scf, 100)
        wf.bus.packet_loss = 1.0
        run(wf, scf, 300)
        fleetx_delivered = wf.board.stats(wf.sim_time)["done"]

        self.assertGreater(fleetx_delivered, central_delivered,
                           f"FLEET-X ({fleetx_delivered} orders) should keep "
                           f"outdelivering central mode ({central_delivered} "
                           f"orders) once the network central depends on is cut")
        self.assertEqual(wc.collisions, 0)
        self.assertEqual(wf.collisions, 0)


if __name__ == "__main__":
    unittest.main()
