"""Phase 7 tests -- breaking jams that would never clear themselves.

Run with:   python3 -m unittest discover -s tests -v

The first test is Anushka's own bug, found by hammering the live dashboard:
a robot finished its job, parked, went idle -- on the exact square another
robot was heading for. It sat there for 365 seconds. No rerouting can help,
because the blocked square IS the destination.
"""

import json
import os
import random
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (Cell, InMemoryBus, RobotStatus, WaitForGraph, Waiting,
                         WaitReport, YieldRequest, choose_victim, from_dict,
                         phase2_world)
from fleetx_core.grid import CellKind
from scenarios import Scenarios

TOUCHING = 0.7


def run(world, seconds, sc=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)


def quiet_world():
    """Three robots, nobody patrolling, so a test can pose them by hand."""
    w = phase2_world()
    sc = Scenarios(w)
    sc.auto = False
    for r in w.robots.values():
        r.clear_goal()
    return w, sc


class TestTheWaitForGraph(unittest.TestCase):
    """05_PATH_PLANNING section 8."""

    def test_a_three_way_loop_is_found(self):
        g = WaitForGraph()
        for a, b in (("R1", "R2"), ("R2", "R3"), ("R3", "R1")):
            g.add(Waiting(a, b, since=5.0, priority=50, is_moving=False))
        self.assertEqual(g.cycle_containing("R1"), ["R1", "R2", "R3"])

    def test_a_two_way_standoff_is_a_loop(self):
        g = WaitForGraph()
        g.add(Waiting("R1", "R2", 5.0, 50, False))
        g.add(Waiting("R2", "R1", 5.0, 50, False))
        self.assertEqual(g.cycle_containing("R1"), ["R1", "R2"])

    def test_a_chain_that_ends_is_not_a_loop(self):
        """Somebody at the far end is free, so it will sort itself out."""
        g = WaitForGraph()
        g.add(Waiting("R1", "R2", 5.0, 50, False))
        g.add(Waiting("R2", "R3", 5.0, 50, False))
        self.assertIsNone(g.cycle_containing("R1"))

    def test_a_loop_we_are_not_part_of_is_not_ours_to_break(self):
        g = WaitForGraph()
        g.add(Waiting("R1", "R2", 5.0, 50, False))
        g.add(Waiting("R2", "R3", 5.0, 50, False))
        g.add(Waiting("R3", "R2", 5.0, 50, False))
        self.assertIsNone(g.cycle_containing("R1"))

    def test_a_parked_robot_is_recognised(self):
        g = WaitForGraph()
        g.add(Waiting("R3", "", 0.0, 50, is_moving=False))
        g._blocked_by.pop("R3", None)
        self.assertTrue(g.is_parked("R3"))

    def test_a_robot_that_is_itself_waiting_is_not_parked(self):
        g = WaitForGraph()
        g.add(Waiting("R3", "R1", 5.0, 50, False))
        self.assertFalse(g.is_parked("R3"))

    def test_the_least_stuck_robot_gives_way(self):
        """Fair: whoever has been stuck longest gets to go first."""
        self.assertEqual(
            choose_victim([("R1", 50, 1.0), ("R2", 60, 9.0), ("R3", 80, 20.0)]),
            "R1")

    def test_when_it_is_close_the_higher_name_gives_way(self):
        self.assertEqual(
            choose_victim([("R1", 50, 5.0), ("R2", 50, 5.0), ("R3", 50, 5.0)]),
            "R3")

    def test_everyone_picks_the_same_victim(self):
        """Each robot has a slightly different reading. They must still agree."""
        answers = set()
        for drift in (0, 3, 6, 9):
            answers.add(choose_victim([("R1", 50, 5.0),
                                       ("R2", 50 + drift, 5.0),
                                       ("R3", 50, 5.0)]))
        self.assertEqual(len(answers), 1, f"robots would disagree: {answers}")


class TestTheMessages(unittest.TestCase):
    def test_wait_report_survives_a_round_trip(self):
        m = WaitReport("R2", 1.0, 5, blocked_by="R3", waiting=6.0,
                       priority=60, is_moving=False)
        back = from_dict(json.loads(json.dumps(m.to_dict())))
        self.assertEqual(back.blocked_by, "R3")
        self.assertFalse(back.is_moving)

    def test_yield_request_survives_a_round_trip(self):
        m = YieldRequest("R2", 1.0, 6, target="R3", resource=(26, 0),
                         reason="PARKED")
        back = from_dict(json.loads(json.dumps(m.to_dict())))
        self.assertEqual(back.target, "R3")
        self.assertEqual(back.resource, (26, 0))


class TestAnushkasDeadlock(unittest.TestCase):
    """The real one, off the live dashboard.

    R3 finished its job, parked on (26,0) and went idle. R2's destination WAS
    (26,0). R2 waited 365 seconds. Rerouting cannot help -- every route to
    (26,0) ends at (26,0). Somebody has to ask R3 to move.
    """

    def _pose(self):
        w, sc = quiet_world()
        w.get("R3").place(Cell(26, 0))       # parked, idle, no goal
        w.get("R2").place(Cell(22, 0))
        w.get("R1").place(Cell(2, 12))
        w.get("R2").set_goal(Cell(26, 0))    # destination IS R3's parking spot
        return w, sc

    def test_without_the_fix_it_never_clears(self):
        w, sc = self._pose()
        w.deadlock_enabled = False
        run(w, 60, sc)
        self.assertNotEqual(w.get("R2").cell, Cell(26, 0))
        self.assertGreater(w.kpis()["deadlocked"], 0)

    def test_with_the_fix_r2_gets_there(self):
        w, sc = self._pose()
        run(w, 40, sc)
        self.assertEqual(w.get("R2").cell, Cell(26, 0), "R2 never arrived")
        self.assertEqual(w.kpis()["deadlocked"], 0)
        self.assertEqual(w.collisions, 0)

    def test_it_clears_quickly_not_eventually(self):
        w, sc = self._pose()
        for _ in range(400):
            sc.keep_busy()
            w.tick(0.05)
            if w.get("R2").cell == Cell(26, 0):
                self.assertLess(w.sim_time, 15.0,
                                "took too long to notice and recover")
                return
        self.fail("R2 never arrived")

    def test_the_parked_robot_is_asked_in_words(self):
        w, sc = self._pose()
        run(w, 40, sc)
        text = " ".join(d["text"] for d in w.decisions)
        self.assertIn("asked R3 to move", text)
        self.assertIn("stepped aside", text)


class TestGivingWayAndComingBack(unittest.TestCase):
    def test_a_robot_remembers_the_job_it_was_doing(self):
        """Breaking a jam by wandering off and forgetting the job would just
        swap one problem for another."""
        w, sc = quiet_world()
        w.get("R1").place(Cell(20, 8))
        w.get("R2").place(Cell(18, 8))
        w.get("R3").place(Cell(2, 12))
        w.get("R1").set_goal(Cell(2, 8))      # R1 must pass through R2
        w.get("R2").set_goal(Cell(26, 8))     # and R2 through R1
        run(w, 60, sc)
        self.assertEqual(w.get("R1").cell, Cell(2, 8), "R1 lost its job")
        self.assertEqual(w.get("R2").cell, Cell(26, 8), "R2 lost its job")
        self.assertEqual(w.collisions, 0)

    def test_a_two_way_standoff_is_broken_even_without_rerouting(self):
        w = phase2_world()
        w.negotiation_enabled = False          # no going around; only yielding
        sc = Scenarios(w)
        sc.apply("head_on")
        run(w, 80, sc)
        self.assertEqual(w.kpis()["deadlocked"], 0)
        self.assertGreater(w.kpis()["deadlocks_broken"], 0)
        self.assertEqual(w.collisions, 0)

    def test_a_boxed_in_robot_passes_the_request_along(self):
        """Its own ways out are blocked, so refusing would leave the queue
        stuck for ever."""
        w, sc = quiet_world()
        w.get("R1").place(Cell(0, 0))
        w.get("R2").place(Cell(0, 1))
        w.get("R3").place(Cell(1, 0))
        w.get("R2").set_goal(Cell(0, 0))
        w.get("R3").set_goal(Cell(0, 0))
        run(w, 80, sc)
        self.assertEqual(w.kpis()["deadlocked"], 0)
        self.assertEqual(w.collisions, 0)
        text = " ".join(d["text"] for d in w.decisions)
        self.assertIn("boxed in", text)

    def test_recoveries_are_only_counted_when_somebody_moves(self):
        """Counting every request would report dozens of 'recoveries' for one
        jam that never cleared."""
        w, sc = quiet_world()
        w.get("R3").place(Cell(26, 0))
        w.get("R2").place(Cell(22, 0))
        w.get("R1").place(Cell(2, 12))
        w.get("R2").set_goal(Cell(26, 0))
        run(w, 40, sc)
        moved = sum(1 for d in w.decisions
                    if "stepped aside" in d["text"] or "moving aside" in d["text"])
        self.assertEqual(w.kpis()["deadlocks_broken"], moved)


class TestNothingElseBroke(unittest.TestCase):
    def test_no_collisions_anywhere(self):
        for name in ("patrol", "head_on", "intersection"):
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply(name)
            worst = 99.0
            for _ in range(2400):
                sc.keep_busy()
                w.tick(0.05)
                rs = list(w.robots.values())
                for i in range(len(rs)):
                    for j in range(i + 1, len(rs)):
                        a, b = rs[i], rs[j]
                        worst = min(worst, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)
                self.assertGreater(worst, TOUCHING,
                                   f"{name}: overlapped at t={w.sim_time:.2f}")
            self.assertEqual(w.collisions, 0, name)

    def test_free_roam_never_gets_stuck(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 300, sc)
        k = w.kpis()
        self.assertEqual(k["deadlocked"], 0)
        self.assertEqual(k["collisions"], 0)
        self.assertGreater(k["total_distance"], 1500)

    def test_nobody_gets_stuck_while_somebody_keeps_clicking(self):
        """Checks the whole run, not one arbitrary final tick.

        Asserting the deadlock counter is zero at the last tick is a weaker
        AND flakier test: a robot six seconds into a recovery that always
        clears would fail it, while a robot wedged for two minutes in the
        middle of the run would pass. Watching how long any robot actually
        goes nowhere catches the real thing and tolerates normal recovery.
        """
        for seed in range(3):
            rng = random.Random(500 + seed)
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("patrol")
            floor = w.grid.cells_of_kind(CellKind.FLOOR)
            nxt = 0.0
            worst_stall = 0.0
            for _ in range(3600):
                if w.sim_time >= nxt:
                    nxt = w.sim_time + rng.choice([0.8, 1.5, 3.0])
                    w.get(rng.choice(list(w.robots))).set_goal(rng.choice(floor))
                sc.keep_busy()
                w.tick(0.05)
                for r in w.robots.values():
                    worst_stall = max(worst_stall, r.stalled_for(w.sim_time))
            self.assertEqual(w.kpis()["collisions"], 0, f"seed {seed}")
            # Typical worst case is 5-8s while a jam is being broken.
            self.assertLess(worst_stall, 30.0,
                            f"seed {seed}: a robot went nowhere for "
                            f"{worst_stall:.0f}s -- that is a real jam")

    def test_snapshot_still_json_safe(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("intersection")
        run(w, 20, sc)
        json.dumps(w.snapshot())

    def test_reset_clears_the_recovery_counters(self):
        w, sc = quiet_world()
        w.get("R3").place(Cell(26, 0))
        w.get("R2").place(Cell(22, 0))
        w.get("R2").set_goal(Cell(26, 0))
        run(w, 30, sc)
        w.reset_counters()
        self.assertEqual(w.kpis()["deadlocks_broken"], 0)
        self.assertEqual(w.kpis()["yields"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
