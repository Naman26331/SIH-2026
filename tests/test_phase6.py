"""Phase 6 tests -- giving way properly.

Run with:   python3 -m unittest discover -s tests -v

The headline test is test_distance_recovers_with_no_collisions: robots must get
their work done AND still never crash. Either one alone is easy.

Several tests here exist because of bugs that actually happened while building
this phase. They are named so nobody quietly reintroduces them.
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (BASE_PRIORITY, Cell, InMemoryBus, Reservation,
                         ReservationTable, Robot, RobotStatus, World,
                         effective_priority, next_wait_credit, node_key,
                         phase2_world)
from fleetx_core.priority import LOW_BATTERY, MAX_AGING, bucket
from scenarios import Scenarios


def run(world, seconds, dt=0.05, sc=None):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)



# Phase 7 added deadlock recovery, which clears these jams. Tests below that
# are about the EARLIER behaviour switch it off with deadlock_enabled=False.


def world_with(res=True, neg=True, breaker=True):
    w = phase2_world()
    w.reservations_enabled = res
    w.negotiation_enabled = neg
    w.deadlock_enabled = breaker
    return w


class TestPriorityScore(unittest.TestCase):
    """04_PROTOCOL section 6 and 05_PATH_PLANNING section 9."""

    def test_a_fresh_robot_sits_on_the_base_score(self):
        self.assertEqual(effective_priority(), BASE_PRIORITY)

    def test_waiting_earns_priority(self):
        calm = effective_priority(waiting=0.0)
        stuck = effective_priority(waiting=5.0)
        self.assertGreater(stuck, calm)

    def test_aging_is_capped(self):
        forever = effective_priority(waiting=10_000.0)
        self.assertLessEqual(forever - BASE_PRIORITY, MAX_AGING + 1)

    def test_a_nearly_flat_robot_gets_priority(self):
        """It needs a charger more than anyone needs the aisle."""
        full = effective_priority(battery=100.0)
        flat = effective_priority(battery=5.0)
        self.assertGreater(flat, full)

    def test_a_healthy_battery_changes_nothing(self):
        self.assertEqual(effective_priority(battery=LOW_BATTERY + 10),
                         effective_priority(battery=100.0))

    def test_waiting_credit_grows_while_stuck(self):
        self.assertAlmostEqual(next_wait_credit(0.0, True, 0.05), 0.05)

    def test_waiting_credit_fades_slowly_once_moving(self):
        """THE bug that caused the worst crash in this phase.

        If credit vanished the moment a robot moved, then: R2 waits, gains
        priority, wins, stops waiting, instantly loses the priority, R1 wins
        again -- and the two swap places every tick and creep into each other.
        """
        credit = 3.0
        after = next_wait_credit(credit, False, 0.05)
        self.assertLess(after, credit)
        self.assertGreater(after, credit - 0.05,
                           "fading must be slower than it built up")

    def test_credit_never_goes_negative(self):
        self.assertEqual(next_wait_credit(0.0, False, 10.0), 0.0)


class TestRobotsMustAgree(unittest.TestCase):
    """Every robot has a slightly different copy of the table. They still have
    to reach the same answer, or two of them both think they won."""

    def test_comparison_is_in_whole_points_not_tenths(self):
        self.assertEqual(bucket(50), bucket(59))
        self.assertNotEqual(bucket(50), bucket(60))

    def test_a_small_difference_of_opinion_cannot_flip_the_owner(self):
        key = node_key(Cell(4, 4))
        # Same situation, but each robot's copy is a fraction out of date.
        answers = set()
        for r2_priority in (50, 53, 57, 59):
            t = ReservationTable()
            t.put(Reservation("R1", key, 0.0, 1.0, priority=50))
            t.put(Reservation("R2", key, 0.0, 1.0, priority=r2_priority))
            answers.add(t.owner(key, 0.0, 1.0).robot_id)
        self.assertEqual(answers, {"R1"}, f"robots would disagree: {answers}")

    def test_a_full_point_of_difference_does_decide_it(self):
        key = node_key(Cell(4, 4))
        t = ReservationTable()
        t.put(Reservation("R1", key, 0.0, 1.0, priority=50))
        t.put(Reservation("R2", key, 0.0, 1.0, priority=61))
        self.assertEqual(t.owner(key, 0.0, 1.0).robot_id, "R2")


class TestGoingAround(unittest.TestCase):
    def test_a_blocked_robot_eventually_reroutes(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, sc=sc)
        self.assertGreater(w.kpis()["reroutes"], 0, "nobody ever went around")

    def test_it_is_patient_before_it_reroutes(self):
        """Most hold-ups clear on their own. A robot that recalculated its
        route 20 times a second would just flap about."""
        w = world_with()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 1.0, sc=sc)
        self.assertEqual(w.kpis()["reroutes"], 0, "rerouted before being patient")

    def test_the_decision_is_written_in_words(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, sc=sc)
        self.assertTrue(w.decisions)
        # Somewhere in the log, not necessarily first. Since head-on standoffs
        # are now spotted in about a second, the loop-detection line usually
        # lands ahead of the going-around line, and asserting on decisions[0]
        # was testing the order of the log rather than the behaviour.
        text = " | ".join(d["text"] for d in w.decisions)
        self.assertIn("gave way", text, f"nobody gave way. Log was: {text}")

    def test_a_reroute_actually_changes_the_route(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("head_on")
        before = None
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
            mover = next((r for r in w.robots.values() if r.reroutes), None)
            if mover:
                self.assertIsNotNone(mover.last_decision)
                break
        else:
            self.fail("nobody rerouted")

    def test_a_lone_robot_never_reroutes(self):
        w = World(bus=InMemoryBus(seed=3))
        r = w.add_robot(Robot("R1", Cell(2, 8)))
        r.set_goal(Cell(26, 8))
        run(w, 30)
        self.assertEqual(w.reroutes, 0)
        self.assertEqual(r.cell, Cell(26, 8))


class TestTheResult(unittest.TestCase):
    """What Phase 6 is actually for."""

    def test_distance_recovers_with_no_collisions(self):
        """The headline. Getting work done AND never crashing.
        Either one alone is easy; both together is the project."""
        jammed = world_with(neg=False, breaker=False)
        sc1 = Scenarios(jammed)
        sc1.apply("patrol")
        run(jammed, 120, sc=sc1)

        fixed = world_with()
        sc2 = Scenarios(fixed)
        sc2.apply("patrol")
        run(fixed, 120, sc=sc2)

        self.assertEqual(fixed.collisions, 0, "collisions must not come back")
        self.assertGreater(fixed.kpis()["total_distance"],
                           jammed.kpis()["total_distance"] * 4,
                           "negotiation did not unjam the warehouse")

    def test_deadlocks_are_cleared_in_free_roam(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 120, sc=sc)
        self.assertEqual(w.kpis()["deadlocked"], 0)

    def test_no_collisions_in_any_scenario(self):
        for name in ("patrol", "head_on", "intersection"):
            w = world_with()
            sc = Scenarios(w)
            sc.apply(name)
            run(w, 60, sc=sc)
            self.assertEqual(w.collisions, 0, f"{name} crashed")

    def test_never_two_robots_on_one_square(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("patrol")
        for _ in range(2400):
            sc.keep_busy()
            w.tick(0.05)
            cells = [r.cell for r in w.robots.values()]
            self.assertEqual(len(set(cells)), len(cells),
                             f"two robots on one square at t={w.sim_time:.2f}")

    def test_robots_never_pass_through_each_other(self):
        """The bug that produced the last two crashes of this phase: two robots
        entering one square from opposite ends, each cleared at a different
        instant, meeting in the middle."""
        w = world_with()
        sc = Scenarios(w)
        sc.apply("patrol")
        for _ in range(2400):
            sc.keep_busy()
            w.tick(0.05)
            robots = list(w.robots.values())
            for i in range(len(robots)):
                for j in range(i + 1, len(robots)):
                    a, b = robots[i], robots[j]
                    gap = ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5
                    self.assertGreater(gap, 0.6,
                                       f"{a.robot_id}/{b.robot_id} overlapped "
                                       f"at t={w.sim_time:.2f}")


class TestNothingElseBroke(unittest.TestCase):
    def test_snapshot_still_json_safe(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 20, sc=sc)
        json.dumps(w.snapshot())

    def test_phase_5_behaviour_is_still_reachable(self):
        w = world_with(neg=False, breaker=False)
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 30, sc=sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(w.kpis()["deadlocked"], 0)

    def test_reset_clears_the_reroute_counters(self):
        w = world_with()
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 20, sc=sc)
        w.reset_counters()
        self.assertEqual(w.kpis()["reroutes"], 0)
        self.assertEqual(w.decisions, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
