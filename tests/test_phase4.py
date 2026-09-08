"""Phase 4 tests -- seeing the crash coming.

Run with:   python3 -m unittest discover -s tests -v

The roadmap asks for four things to be detected: same node, same edge,
intersection, time overlap. There is a test for each.

The headline test is test_every_crash_is_predicted_in_advance. If that ever
drops below 100%, there is no point building reservations on top of this.
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (Cell, ConflictAlert, ConflictKind, InMemoryBus, Robot,
                         RobotStatus, Window, World, build_plan, default_grid,
                         find_conflicts, from_dict, phase2_world)
from scenarios import Scenarios



# Phase 5 added square booking, which is what stops the crashes. Tests below
# that are about the EARLIER behaviour (robots driving blind) now switch it
# off explicitly with reservations_enabled=False. That is not a workaround --
# Phase 15 runs the "before" side of the benchmark exactly the same way.


def blind(world):
    """Turn booking off, to test detection on its own."""
    world.reservations_enabled = False
    return world


def run(world, seconds, dt=0.05, sc=None):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)


class TestTimeSlots(unittest.TestCase):
    """The meeting-room booking maths."""

    def test_two_slots_overlap_or_do_not(self):
        a = Window(Cell(1, 1), 3.0, 3.6)
        self.assertTrue(a.overlaps(Window(Cell(1, 1), 3.4, 4.0)))
        self.assertFalse(a.overlaps(Window(Cell(1, 1), 3.6, 4.2)), "touching is not overlapping")
        self.assertFalse(a.overlaps(Window(Cell(1, 1), 0.0, 3.0)))

    def test_a_parked_robot_holds_its_square_the_whole_time(self):
        """Driving into a stationary robot is just as much a crash."""
        plan = build_plan("R1", Cell(5, 5), [], [], speed=2.5, horizon=8.0)
        self.assertEqual(len(plan.nodes), 1)
        self.assertEqual(plan.nodes[0].cell, Cell(5, 5))
        self.assertEqual(plan.nodes[0].end, 8.0)

    def test_a_moving_robot_books_each_square_for_a_span_not_an_instant(self):
        plan = build_plan("R1", Cell(0, 0), [Cell(1, 0), Cell(2, 0)],
                          [0.4, 0.8], speed=2.5, horizon=8.0)
        booked = {w.cell: w for w in plan.nodes}
        self.assertIn(Cell(1, 0), booked)
        w = booked[Cell(1, 0)]
        self.assertLess(w.start, w.end, "a booking must have a duration")
        self.assertLessEqual(w.start, 0.4)
        self.assertGreaterEqual(w.end, 0.8)

    def test_late_information_is_shifted_earlier(self):
        """A message that took 0.3s to arrive has ETAs 0.3s out of date."""
        fresh = build_plan("R2", Cell(0, 0), [Cell(1, 0)], [1.0], 2.5, time_offset=0.0)
        late = build_plan("R2", Cell(0, 0), [Cell(1, 0)], [1.0], 2.5, time_offset=0.3)
        f = [w for w in fresh.nodes if w.cell == Cell(1, 0)][0]
        l = [w for w in late.nodes if w.cell == Cell(1, 0)][0]
        self.assertAlmostEqual(f.start - l.start, 0.3, places=2)


class TestTheFourDetections(unittest.TestCase):
    """The roadmap's list: same node, same edge, intersection, time overlap."""

    def setUp(self):
        self.grid = default_grid()

    def test_same_square(self):
        a = build_plan("R1", Cell(10, 0), [Cell(11, 0), Cell(12, 0)], [0.4, 0.8], 2.5)
        b = build_plan("R2", Cell(12, 1), [Cell(12, 0), Cell(12, 1)], [0.5, 0.9], 2.5)
        found = find_conflicts(a, [b], self.grid)
        self.assertTrue(found)
        self.assertEqual(found[0].cell, Cell(12, 0))

    def test_same_edge_head_on(self):
        """They swap places and never share a square, so the square check alone
        would miss it. 05_PATH_PLANNING section 5."""
        a = build_plan("R1", Cell(10, 0), [Cell(11, 0)], [0.4], 2.5)
        b = build_plan("R2", Cell(11, 0), [Cell(10, 0)], [0.4], 2.5)
        found = find_conflicts(a, [b], self.grid)
        self.assertTrue(any(c.kind is ConflictKind.HEAD_ON for c in found))

    def test_junction_is_labelled_separately(self):
        junction = Cell(13, 8)
        self.assertTrue(self.grid.is_junction(junction))
        a = build_plan("R1", Cell(11, 8), [Cell(12, 8), junction], [0.4, 0.8], 2.5)
        b = build_plan("R2", junction, [Cell(13, 9)], [0.8], 2.5)
        found = find_conflicts(a, [b], self.grid)
        self.assertTrue(any(c.kind is ConflictKind.JUNCTION for c in found))

    def test_time_overlap_is_required(self):
        """Same square, but hours apart, is NOT a conflict."""
        a = build_plan("R1", Cell(10, 0), [Cell(11, 0)], [0.4], 2.5, horizon=60)
        b = build_plan("R2", Cell(11, 1), [Cell(11, 0)], [30.0], 2.5, horizon=60)
        self.assertEqual(find_conflicts(a, [b], self.grid, horizon=60), [])

    def test_different_squares_is_not_a_conflict(self):
        a = build_plan("R1", Cell(2, 0), [Cell(3, 0), Cell(4, 0)], [0.4, 0.8], 2.5)
        b = build_plan("R2", Cell(2, 12), [Cell(3, 12), Cell(4, 12)], [0.4, 0.8], 2.5)
        self.assertEqual(find_conflicts(a, [b], self.grid), [])

    def test_a_robot_never_conflicts_with_itself(self):
        a = build_plan("R1", Cell(2, 0), [Cell(3, 0)], [0.4], 2.5)
        self.assertEqual(find_conflicts(a, [a], self.grid), [])

    def test_nothing_beyond_the_horizon(self):
        a = build_plan("R1", Cell(0, 0), [Cell(1, 0)], [0.4], 2.5, horizon=60)
        b = build_plan("R2", Cell(1, 1), [Cell(1, 0)], [20.0], 2.5, horizon=60)
        self.assertEqual(find_conflicts(a, [b], self.grid, horizon=8.0), [])


class TestRobotsSpotTheirOwnTrouble(unittest.TestCase):
    def test_a_robot_finds_a_conflict_from_what_it_heard(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.0, sc=sc)
        self.assertTrue(w.get("R1").conflicts or w.get("R2").conflicts,
                        "neither robot noticed a head-on coming")

    def test_a_robot_it_cannot_hear_is_invisible_to_it(self):
        """No god-view. If the messages did not arrive, there is no conflict
        to be seen -- which is exactly why Phase 6 needs real negotiation."""
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.0, sc=sc)
        w.bus.silence("R2", True)
        run(w, 3.0, sc=sc)
        self.assertEqual(w.get("R1").conflicts, [],
                         "R1 should not 'see' a conflict with a robot it stopped hearing")

    def test_conflict_alerts_go_out_on_the_radio(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 1.5, sc=sc)
        alerts = [m for m in w.bus.recent if isinstance(m, ConflictAlert)]
        self.assertTrue(alerts, "no CONFLICT_ALERT was ever broadcast")
        a = alerts[0]
        self.assertIn(a.kind, [k.value for k in ConflictKind])
        self.assertEqual(len(a.resource), 2)

    def test_alerts_are_not_spammed_every_tick(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 2.0, sc=sc)
        alerts = [m for m in w.bus.recent if isinstance(m, ConflictAlert)]
        self.assertLess(len(alerts), 20, "alerts should be rate limited")

    def test_conflict_alert_survives_a_round_trip(self):
        msg = ConflictAlert("R1", 1.0, 7, robot_a="R1", robot_b="R2",
                            resource=(13, 8), kind="JUNCTION", estimated_time=3.2)
        back = from_dict(json.loads(json.dumps(msg.to_dict())))
        self.assertEqual(back.resource, (13, 8))
        self.assertEqual(back.robot_b, "R2")

    def test_a_failed_robot_reports_nothing(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.0, sc=sc)
        w.get("R1").status = RobotStatus.FAILED
        run(w, 0.5, sc=sc)
        self.assertEqual(w.get("R1").conflicts, [])


class TestDidWeActuallySeeItComing(unittest.TestCase):
    """The measurable result. This is what goes in front of a judge."""

    def test_every_crash_is_predicted_in_advance(self):
        w = blind(phase2_world())
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 90, sc=sc)
        k = w.kpis()
        self.assertGreater(k["collisions"], 0, "no crashes means this proves nothing")
        self.assertEqual(k["collisions_unpredicted"], 0,
                         f"{k['collisions_unpredicted']} crashes came out of nowhere")
        self.assertEqual(k["collisions_predicted"], k["collisions"])

    def test_the_warning_is_long_enough_to_be_useful(self):
        w = blind(phase2_world())
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 90, sc=sc)
        self.assertGreaterEqual(w.kpis()["avg_warning"], 1.5,
                                "under 1.5s warning is too late to act on")

    def test_warning_is_measured_from_the_first_alarm_not_the_last(self):
        """A live conflict counts down to zero as robots close in. Reporting
        the latest reading would say 'we predicted it 0.05s ahead' every time."""
        w = blind(phase2_world())
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 25, sc=sc)
        for e in w.collision_events:
            if e["predicted"]:
                self.assertGreater(e["warning"], 1.0,
                                   "warning time collapsed to ~0, the metric is broken")

    def test_it_does_not_cry_wolf_constantly(self):
        w = blind(phase2_world())
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 90, sc=sc)
        k = w.kpis()
        self.assertLess(k["conflicts_raised"], 200, "far too twitchy")

    def test_one_robot_alone_predicts_nothing(self):
        w = World(bus=InMemoryBus(seed=1))
        r = w.add_robot(Robot("R1", Cell(2, 8)))
        r.set_goal(Cell(26, 8))
        run(w, 30)
        self.assertEqual(w.kpis()["conflicts_raised"], 0)
        self.assertEqual(w.kpis()["collisions"], 0)


class TestNothingElseBroke(unittest.TestCase):
    def test_snapshot_still_json_safe(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 10, sc=sc)
        json.dumps(w.snapshot())

    def test_detection_alone_avoids_nothing(self):
        """Phase 4 only LOOKS. Announcing a crash does not prevent it --
        booking the square (Phase 5) is what prevents it."""
        w = blind(phase2_world())
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 25, sc=sc)
        self.assertGreater(w.collisions, 0)

    def test_reset_clears_the_prediction_scoreboard(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 20, sc=sc)
        w.reset_counters()
        k = w.kpis()
        self.assertEqual(k["collisions_predicted"], 0)
        self.assertEqual(k["conflicts_raised"], 0)
        self.assertEqual(k["avg_warning"], 0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
