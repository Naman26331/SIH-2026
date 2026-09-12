"""Phase 19 tests -- the geometry safety layer.

Run with:   python3 -m unittest discover -s tests -v

Square booking only reasons about which NODE or EDGE a robot holds. It has
nothing to say about the open floor near a shared corner, where two robots
each transiting a DIFFERENT edge that happens to meet at the same node can
swing closer to each other than either edge or node reservation alone would
ever catch. World._enforce_geometry_gap() is the backstop UNDER that: a flat,
unnegotiated "too close -- stop", checked on real (x, y), same as the
person-proximity hard stop it mirrors, for every coordination mode alike.

The claim these tests hold: it engages exactly when two robots' actual bodies
are closer than GEOMETRY_SAFE_GAP, never at the normal 1.0-square operating
floor, never leaves a pair stuck forever, and never fires at all across
ordinary simulated running -- it is a backstop, not a change to how things
normally behave.
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import (Cell, GEOMETRY_SAFE_GAP, Robot, RobotStatus, World,
                         fleet_world)
from scenarios import Scenarios

TOUCHING = 0.7


def close_pair(ax, ay, bx, by):
    """A fresh world with two robots placed at exact continuous positions,
    with nothing else going on -- isolates the geometry check by itself."""
    w = World()
    a = Robot(robot_id="R1", cell=Cell(0, 0))
    b = Robot(robot_id="R2", cell=Cell(1, 0))
    w.add_robot(a)
    w.add_robot(b)
    a.x, a.y = ax, ay
    b.x, b.y = bx, by
    return w, a, b


class TestTheBackstopTriggers(unittest.TestCase):

    def test_closer_than_the_safe_gap_forces_a_hard_stop(self):
        w, a, b = close_pair(0.0, 0.0, GEOMETRY_SAFE_GAP - 0.05, 0.0)
        w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertTrue(a.hold and a.emergency)
        self.assertTrue(b.hold and b.emergency)
        self.assertEqual(a.blocked_by, "R2")
        self.assertEqual(b.blocked_by, "R1")
        self.assertEqual(w.geometry_interventions, 1)

    def test_one_intervention_is_counted_once_not_once_per_tick(self):
        w, a, b = close_pair(0.0, 0.0, GEOMETRY_SAFE_GAP - 0.05, 0.0)
        for _ in range(10):
            w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertEqual(w.geometry_interventions, 1,
                         "the SAME encroachment held ten ticks running up "
                         "the count -- it should count the episode, not the "
                         "tick, the same way _detect_collisions does")

    def test_almost_touching_still_triggers(self):
        w, a, b = close_pair(0.0, 0.0, TOUCHING + 0.02, 0.0)
        w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertTrue(a.hold and b.hold)


class TestTheBackstopStaysOutOfTheWay(unittest.TestCase):
    """The normal operating floor on this grid -- robots resting on their
    own squares, no diagonal movement -- never puts two of them closer than
    1.0 squares apart. The backstop must never fire there."""

    def test_exactly_one_square_apart_does_not_trigger(self):
        w, a, b = close_pair(0.0, 0.0, 1.0, 0.0)
        w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertFalse(a.hold)
        self.assertFalse(b.hold)
        self.assertEqual(w.geometry_interventions, 0)

    def test_diagonal_neighbours_do_not_trigger(self):
        w, a, b = close_pair(0.0, 0.0, 1.0, 1.0)
        w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertFalse(a.hold)
        self.assertFalse(b.hold)
        self.assertEqual(w.geometry_interventions, 0)

    def test_does_not_touch_a_hold_already_set_for_another_reason(self):
        w, a, b = close_pair(0.0, 0.0, 1.0, 0.0)
        a._set_hold(True, w.sim_time, 0.05, emergency=False)
        a.blocked_by = "obstacle"
        w._enforce_geometry_gap(w.sim_time, 0.05)
        self.assertEqual(a.blocked_by, "obstacle",
                         "a robot held for an unrelated reason, far enough "
                         "away, must be left exactly as the real reason set it")

    def test_a_long_normal_run_never_needs_the_backstop(self):
        """The actual claim: existing behaviour stays. Run real orders, real
        traffic, and confirm this backstop is invisible the whole time."""
        for robots in (3, 5, 10):
            w = fleet_world(robots)
            sc = Scenarios(w)
            sc.apply("orders", seed=1, every=3.0, limit=None)
            while w.sim_time < 120:
                sc.keep_busy()
                w.tick(0.05)
            self.assertEqual(w.collisions, 0)
            self.assertEqual(w.geometry_interventions, 0,
                             f"the geometry backstop fired during ordinary "
                             f"{robots}-robot traffic -- it should stay "
                             f"invisible unless something else already let "
                             f"two robots get closer than normal")


class TestNobodyGetsStuckForever(unittest.TestCase):
    """A hard stop is not a plan. Whichever robot is mid-square when this
    catches a pair is exactly the case back_out_if_wedged() already knows
    how to recover -- the same machinery any other emergency hold already
    relies on, not something new this layer had to invent."""

    def test_a_wedged_robot_backs_away_and_the_gap_reopens(self):
        w = World()
        a = Robot(robot_id="R1", cell=Cell(0, 0))
        b = Robot(robot_id="R2", cell=Cell(1, 0))
        w.add_robot(a)
        w.add_robot(b)
        a.status = RobotStatus.MOVING
        a.goal = Cell(2, 0)
        a.path = [Cell(1, 0)]
        a._progress = 0.85                      # mid-square, close to R2
        a.x, a.y = a.cell.x + 0.85, 0.0
        b.x, b.y = 1.0, 0.0

        started_reversing = False
        for _ in range(200):
            w._enforce_geometry_gap(w.sim_time, 0.05)
            a.back_out_if_wedged(w.sim_time)
            if a._reversing:
                started_reversing = True
            a.advance(0.05)
            w.sim_time += 0.05
            if started_reversing and not a._reversing:
                break                            # finished backing out

        self.assertTrue(started_reversing,
                        "a robot held on emergency, wedged mid-square, "
                        "never triggered its own recovery")
        gap = math.hypot(a.x - b.x, a.y - b.y)
        self.assertGreaterEqual(gap, GEOMETRY_SAFE_GAP,
                                "backing out should have reopened the gap")


class TestTheBackstopAppliesToEveryMode(unittest.TestCase):
    """'Sits under the existing square booking' means every mode -- not a
    special case for one coordination scheme."""

    def test_fleetx_stop_and_wait_and_central_all_get_the_same_check(self):
        for mode in ("FLEETX", "STOP_AND_WAIT", "CENTRAL"):
            w = fleet_world(2, coordination=mode)
            robots = list(w.robots.values())
            a, b = robots[0], robots[1]
            a.x, a.y = 0.0, 0.0
            b.x, b.y = GEOMETRY_SAFE_GAP - 0.1, 0.0
            w._enforce_geometry_gap(w.sim_time, 0.05)
            self.assertTrue(a.hold, f"{mode}: backstop did not engage")
            self.assertTrue(b.hold, f"{mode}: backstop did not engage")


if __name__ == "__main__":
    unittest.main()
