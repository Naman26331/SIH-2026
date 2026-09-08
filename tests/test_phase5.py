"""Phase 5 tests -- booking squares.

Run with:   python3 -m unittest discover -s tests -v

The headline test is test_no_collisions_in_free_roam. That is the number the
whole project is judged on (00_README: "0 inter-robot collisions").

There are also tests asserting deadlocks DO appear. Phase 5 trades crashes for
jams on purpose, and hiding that would make Phases 6 and 7 look pointless.
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (Cell, InMemoryBus, PathReservation, Reservation,
                         ReservationTable, Robot, RobotStatus, World,
                         edge_key, from_dict, node_key, phase2_world)
from fleetx_core.reservations import OCCUPANCY_PRIORITY
from scenarios import Scenarios


def run(world, seconds, dt=0.05, sc=None):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)


def fresh_world(**bus_kwargs):
    w = phase2_world()
    w.bus = InMemoryBus(seed=13, **bus_kwargs)
    for rid in w.robots:
        w.bus.register(rid)
    return w


class TestTheTable(unittest.TestCase):
    def test_an_empty_square_has_no_owner(self):
        t = ReservationTable()
        self.assertIsNone(t.owner(node_key(Cell(1, 1)), 0.0, 1.0))

    def test_one_claim_makes_you_the_owner(self):
        t = ReservationTable()
        t.put(Reservation("R1", node_key(Cell(1, 1)), 0.0, 1.0))
        self.assertTrue(t.is_owner("R1", node_key(Cell(1, 1)), 0.0, 1.0))

    def test_non_overlapping_bookings_both_stand(self):
        """05_PATH_PLANNING section 4: N7 10-12 R1, N7 12-14 R2 is fine."""
        t = ReservationTable()
        key = node_key(Cell(7, 7))
        t.put(Reservation("R1", key, 10.0, 12.0))
        t.put(Reservation("R2", key, 12.0, 14.0))
        self.assertTrue(t.is_owner("R1", key, 10.0, 12.0))
        self.assertTrue(t.is_owner("R2", key, 12.0, 14.0))

    def test_lower_robot_id_wins_a_tie(self):
        """04_PROTOCOL section 7 -- makes the system deterministic."""
        t = ReservationTable()
        key = node_key(Cell(3, 3))
        t.put(Reservation("R2", key, 0.0, 1.0, priority=5))
        t.put(Reservation("R1", key, 0.0, 1.0, priority=5))
        self.assertEqual(t.owner(key, 0.0, 1.0).robot_id, "R1")

    def test_higher_priority_beats_a_lower_id(self):
        """Priorities are stored in tenths, and compared in whole points,
        so a real gap is 50 vs 90 -- not 5 vs 9."""
        t = ReservationTable()
        key = node_key(Cell(3, 3))
        t.put(Reservation("R1", key, 0.0, 1.0, priority=50))
        t.put(Reservation("R9", key, 0.0, 1.0, priority=90))
        self.assertEqual(t.owner(key, 0.0, 1.0).robot_id, "R9")

    def test_a_tenth_of_a_point_does_not_change_the_answer(self):
        """Robots hold slightly different copies of the table. If a tenth of a
        point could decide ownership, two robots could each decide they won."""
        t = ReservationTable()
        key = node_key(Cell(3, 3))
        t.put(Reservation("R1", key, 0.0, 1.0, priority=50))
        t.put(Reservation("R2", key, 0.0, 1.0, priority=59))
        self.assertEqual(t.owner(key, 0.0, 1.0).robot_id, "R1",
                         "same whole point, so the lower ID must still win")

    def test_the_answer_does_not_depend_on_arrival_order(self):
        """The safety property. Messages reach different robots in different
        orders, and every robot must still reach the same conclusion."""
        key = node_key(Cell(4, 4))
        claims = [
            Reservation("R3", key, 0.0, 1.0, priority=5),
            Reservation("R1", key, 0.2, 1.2, priority=5),
            Reservation("R2", key, 0.1, 1.1, priority=7),
        ]
        answers = set()
        import itertools
        for order in itertools.permutations(claims):
            t = ReservationTable()
            for c in order:
                t.put(c)
            answers.add(t.owner(key, 0.0, 1.2).robot_id)
        self.assertEqual(len(answers), 1, f"robots would disagree: {answers}")

    def test_an_edge_is_the_same_resource_in_both_directions(self):
        """This is what stops a head-on. 05_PATH_PLANNING section 5."""
        self.assertEqual(edge_key(Cell(1, 1), Cell(2, 1)),
                         edge_key(Cell(2, 1), Cell(1, 1)))

    def test_a_robot_holds_only_one_slot_per_square(self):
        t = ReservationTable()
        key = node_key(Cell(1, 1))
        t.put(Reservation("R1", key, 0.0, 1.0))
        t.put(Reservation("R1", key, 5.0, 6.0))
        self.assertEqual(len(t.claims_on(key, 0.0, 10.0)), 1)

    def test_releasing_gives_the_square_back(self):
        t = ReservationTable()
        key = node_key(Cell(1, 1))
        t.put(Reservation("R1", key, 0.0, 1.0))
        t.release("R1", key)
        self.assertIsNone(t.owner(key, 0.0, 1.0))

    def test_old_bookings_are_forgotten(self):
        t = ReservationTable()
        t.put(Reservation("R1", node_key(Cell(1, 1)), 0.0, 1.0))
        t.prune(now=100.0)
        self.assertEqual(len(t), 0)

    def test_standing_on_a_square_outranks_everything(self):
        """A robot cannot be booked out of the square its body is in."""
        t = ReservationTable()
        key = node_key(Cell(5, 5))
        t.put(Reservation("R9", key, 0.0, 2.0, priority=OCCUPANCY_PRIORITY))
        t.put(Reservation("R1", key, 0.0, 2.0, priority=99))
        self.assertEqual(t.owner(key, 0.0, 2.0).robot_id, "R9")


class TestTheMessage(unittest.TestCase):
    def test_reservation_survives_a_round_trip(self):
        msg = PathReservation("R1", 1.0, 4, action="CLAIM", kind="NODE",
                              cells=[(13, 8)], start=3.0, end=3.6, priority=5)
        back = from_dict(json.loads(json.dumps(msg.to_dict())))
        self.assertEqual(back.cells, [(13, 8)])
        self.assertEqual(back.action, "CLAIM")

    def test_bookings_actually_go_out_on_the_radio(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.0, sc=sc)
        claims = [m for m in w.bus.recent if isinstance(m, PathReservation)]
        self.assertTrue(claims, "no PATH_RESERVATION was ever broadcast")

    def test_a_robot_learns_of_other_bookings_from_the_radio(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.5, sc=sc)
        others = [r for slots in w.get("R1").table._by_resource.values()
                  for r in slots if r.robot_id != "R1"]
        self.assertTrue(others, "R1 never heard anybody else's booking")


class TestTheRuleThatStopsCrashes(unittest.TestCase):
    def test_a_robot_stops_short_when_refused(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 4.0, sc=sc)
        waiting = [r for r in w.robots.values() if r.status is RobotStatus.WAITING]
        self.assertTrue(waiting, "nobody yielded in a head-on")
        self.assertTrue(all(r.blocked_by for r in waiting))

    def test_an_ordinary_hold_stops_tidily_on_a_square(self):
        """Refused a booking, but nothing dangerous nearby: finish the segment
        already begun and wait neatly on a square.

        An EMERGENCY stop is different -- see the next test."""
        w = fresh_world()
        w.negotiation_enabled = False
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 6.0, sc=sc)
        for r in w.robots.values():
            if r.status is RobotStatus.WAITING and not r.emergency:
                self.assertAlmostEqual(r.x, float(r.cell.x), places=6)
                self.assertAlmostEqual(r.y, float(r.cell.y), places=6)

    def test_an_emergency_stop_is_immediate(self):
        """Something in the space ahead means stop NOW, even mid-aisle.

        Robots have brakes. Insisting on always finishing the segment already
        begun is what let two robots -- each committed a fraction of a second
        before hearing about the other -- drive into each other.
        05_PATH_PLANNING section 12: the safety controller takes precedence.
        """
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        last = {}
        checked = 0
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.emergency and r.robot_id in last:
                    px, py = last[r.robot_id]
                    moved = ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5
                    self.assertLess(moved, 1e-9,
                                    f"{r.robot_id} kept moving during an "
                                    f"emergency stop at t={w.sim_time:.2f}")
                    checked += 1
                last[r.robot_id] = (r.x, r.y)
        self.assertGreater(checked, 0, "no emergency stop ever happened")

    def test_never_drives_onto_an_occupied_square(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
            cells = [r.cell for r in w.robots.values()]
            self.assertEqual(len(set(cells)), len(cells),
                             f"two robots on one square at t={w.sim_time:.2f}")

    def test_no_collisions_in_the_intersection(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 30, sc=sc)
        self.assertEqual(w.collisions, 0)

    def test_no_collisions_in_a_head_on(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 30, sc=sc)
        self.assertEqual(w.collisions, 0)

    def test_no_collisions_in_free_roam(self):
        """THE headline number. 00_README: "0 inter-robot collisions"."""
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 120, sc=sc)
        self.assertEqual(w.collisions, 0, "the whole project is judged on this")

    def test_a_lone_robot_is_not_slowed_down_by_any_of_this(self):
        w = World(bus=InMemoryBus(seed=2))
        r = w.add_robot(Robot("R1", Cell(2, 8)))
        r.set_goal(Cell(26, 8))
        run(w, 30)
        self.assertEqual(r.cell, Cell(26, 8))
        self.assertEqual(r.wait_time, 0.0, "nobody to wait for, so it must not wait")


class TestTheJamsWeJustCreated(unittest.TestCase):
    """Phase 5 trades crashes for jams. These tests assert the jams are real.

    Phase 6 (going around instead of waiting) is what clears them, so these
    run with negotiation switched off -- they are about Phase 5 on its own."""

    @staticmethod
    def _no_negotiation(w):
        """Phase 5 on its own: no going around (Phase 6) and no asking anybody
        to move (Phase 7). Both of those are what clear these jams."""
        w.negotiation_enabled = False
        w.deadlock_enabled = False
        return w

    def test_a_head_on_now_deadlocks_instead_of_crashing(self):
        w = self._no_negotiation(fresh_world())
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 30, sc=sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(w.kpis()["deadlocked"], 0,
                           "expected a deadlock -- Phase 6/7 exist to fix it")

    def test_stuck_robots_are_named(self):
        w = self._no_negotiation(fresh_world())
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 30, sc=sc)
        self.assertTrue(w.stuck_robots())

    def test_waiting_time_is_counted(self):
        w = self._no_negotiation(fresh_world())
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, sc=sc)
        self.assertGreater(w.kpis()["total_wait"], 0)


class TestNothingElseBroke(unittest.TestCase):
    def test_snapshot_still_json_safe(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 10, sc=sc)
        json.dumps(w.snapshot())

    def test_reset_clears_the_waiting_counters(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, sc=sc)
        w.reset_counters()
        k = w.kpis()
        self.assertEqual(k["total_wait"], 0)
        self.assertEqual(k["deadlocks_seen"], 0)

    def test_conflicts_are_still_predicted(self):
        w = fresh_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 5, sc=sc)
        self.assertGreater(w.kpis()["conflicts_raised"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
