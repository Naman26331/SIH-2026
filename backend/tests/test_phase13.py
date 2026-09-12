"""Phase 13 tests -- battery and charging.

Run with:   python3 -m unittest discover -s tests -v

The interesting question is not "does the number go down". It is whether a
robot decides to charge at the right moment, whether four bays can serve twenty
robots without them all setting off at once, and whether any of it can cause a
crash. That last one is the only answer that is allowed to be no.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, RobotStatus, World, fleet_world
from fleetx_core.grid import CellKind
from scenarios import Scenarios


def run(world, seconds, scenarios=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if scenarios is not None:
            scenarios.keep_busy()
        world.tick(dt)


class TestTheMapHasChargers(unittest.TestCase):
    def test_four_bays_and_none_of_them_in_a_corner(self):
        g = fleet_world(3).grid
        bays = g.cells_of_kind(CellKind.CHARGER)
        self.assertEqual(len(bays), 4)
        for bay in bays:
            sides = [n for n in (Cell(bay.x + 1, bay.y), Cell(bay.x - 1, bay.y),
                                 Cell(bay.x, bay.y + 1), Cell(bay.x, bay.y - 1))
                     if g.is_walkable(n)]
            self.assertGreaterEqual(
                len(sides), 3,
                f"charger {bay} is in a pocket -- robots will wedge trying to reach it")

    def test_bays_are_spread_out(self):
        """They used to be a 2x2 block in the corner. Drain ten batteries and
        all ten converged on one corner, wedged, and sat there until flat."""
        bays = fleet_world(3).grid.cells_of_kind(CellKind.CHARGER)
        for a in bays:
            for b in bays:
                if a is b:
                    continue
                self.assertGreater(abs(a.x - b.x) + abs(a.y - b.y), 2,
                                   f"{a} and {b} are close enough to jam each other")


class TestTheChargingDecision(unittest.TestCase):
    def test_a_full_robot_does_not_want_a_charger(self):
        w = fleet_world(3)
        r = w.robots["R1"]
        self.assertTrue(r.can_finish_and_still_reach_a_charger(
            w.grid, 0.0, [Cell(13, 0), Cell(5, 13)]))

    def test_an_empty_robot_knows_it_cannot_finish(self):
        w = fleet_world(3)
        r = w.robots["R1"]
        r.battery = 4.0
        self.assertFalse(r.can_finish_and_still_reach_a_charger(
            w.grid, 0.0, [Cell(13, 0), Cell(5, 13)]))

    def test_the_line_moves_with_the_job_not_a_fixed_percent(self):
        """The whole point of asking "can I finish THIS job" rather than "am I
        under 20%": a short job is fine on a charge that a long one is not."""
        w = fleet_world(3)
        r = w.robots["R1"]
        r.battery = 16.0
        near = r.can_finish_and_still_reach_a_charger(w.grid, 0.0, [Cell(3, 14)])
        far = r.can_finish_and_still_reach_a_charger(
            w.grid, 0.0, [Cell(27, 2), Cell(0, 10), Cell(27, 10)])
        self.assertTrue(near, "a short hop should still be on")
        self.assertFalse(far, "a long haul should not be")

    def test_a_flat_robot_will_not_bid_for_work(self):
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 6, sc)
        task = next(iter(w.board.tasks.values()))
        r = w.robots["R1"]
        r.battery = 100.0
        self.assertIsNotNone(r.cost_of(task, w.grid, w.sim_time))
        r.battery = 3.0
        self.assertIsNone(r.cost_of(task, w.grid, w.sim_time),
                          "a nearly flat robot bid for a job it cannot finish")


class TestChargingActuallyHappens(unittest.TestCase):
    def test_a_low_robot_books_a_bay_and_fills_up(self):
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=4.0)
        run(w, 10, sc)
        r = w.robots["R1"]
        r.battery = 9.0
        r._charge_checked_at = -99.0
        # Watch the PEAK, not the reading at the end: once it is full it goes
        # back to work and starts draining again, so sampling at one arbitrary
        # moment says nothing about whether it ever charged.
        peak = r.battery
        for _ in range(2400):
            sc.keep_busy()
            w.tick(0.05)
            peak = max(peak, r.battery)
        self.assertGreater(peak, 90.0,
                           f"R1 never charged, the most it ever had was {peak:.0f}%")
        self.assertEqual(w.collisions, 0)

    def test_it_sits_on_the_bay_rather_than_being_shooed_off_it(self):
        """An idle robot is normally moved off a station. A charging one is
        there on purpose, and used to be sent away mid-charge."""
        w = fleet_world(3)
        r = w.robots["R1"]
        r.battery = 10.0
        r._charge_checked_at = -99.0
        run(w, 60)
        seen = [r.status is RobotStatus.CHARGING]
        run(w, 3)
        self.assertTrue(any(seen) or r.battery > 50.0)

    def test_charging_status_survives_a_whole_tick(self):
        """Deciding runs after the battery pass, so it used to see "no
        destination", call the robot idle and undo charging every tick. Ten
        robots sat on four chargers and drained to nothing."""
        w = fleet_world(3)
        r = w.robots["R1"]
        r.battery = 10.0
        r._charge_checked_at = -99.0
        charged = False
        for _ in range(1600):
            w.tick(0.05)
            if r.status is RobotStatus.CHARGING:
                charged = True
                break
        self.assertTrue(charged, "the robot never reached CHARGING status")
        before = r.battery
        w.tick(0.05)
        self.assertIs(r.status, RobotStatus.CHARGING, "charging lasted one tick")
        self.assertGreater(r.battery, before, "it was not actually charging")


class TestTheyTakeTurns(unittest.TestCase):
    def test_twenty_robots_four_bays_nobody_crashes(self):
        w = fleet_world(20)
        sc = Scenarios(w)
        sc.apply("orders", every=4.0)
        run(w, 20, sc)
        w.drain_batteries()
        run(w, 200, sc)
        self.assertEqual(w.collisions, 0)
        low = [r.robot_id for r in w.robots.values() if r.battery < 25.0]
        self.assertLessEqual(len(low), 4,
                             f"still flat after 200 seconds: {low}")

    def test_never_more_robots_on_bays_than_there_are_bays(self):
        w = fleet_world(20)
        sc = Scenarios(w)
        sc.apply("orders", every=4.0)
        run(w, 20, sc)
        w.drain_batteries()
        bays = set(w.grid.cells_of_kind(CellKind.CHARGER))
        worst = 0
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
            on = [r for r in w.robots.values()
                  if r.status is RobotStatus.CHARGING]
            worst = max(worst, len(on))
            self.assertEqual(len(on), len({r.cell for r in on}),
                             "two robots charging on the same square")
            for r in on:
                self.assertIn(r.cell, bays, f"{r.robot_id} charging off a bay")
        self.assertLessEqual(worst, len(bays))

    def test_the_emptiest_robot_gets_the_bay(self):
        """Two robots, one bay's worth of urgency apart. The emptier must win,
        and both must reach the SAME answer from their own copy of the table."""
        w = fleet_world(3)
        a, b = w.robots["R1"], w.robots["R2"]
        a.battery, b.battery = 40.0, 8.0
        self.assertGreater(b.charge_urgency(), a.charge_urgency())

    def test_urgency_is_banded_so_stale_readings_matter_less(self):
        """Every robot reads everyone else's charge a fraction of a second
        late, so the answer must not turn on a hair.

        Bucketing cannot remove the problem at a band edge -- 30.0% and 30.4%
        land either side of one -- and claiming otherwise would be a test that
        lies. What it does is make a hair's difference irrelevant almost
        everywhere, and leave a tie broken by robot ID, which cannot go stale.
        The stand-down in manage_battery covers the edge case: whoever hears
        that somebody emptier wanted the bay gives it up.
        """
        w = fleet_world(3)
        a, b = w.robots["R1"], w.robots["R2"]
        a.battery, b.battery = 31.0, 34.0          # well inside one band
        self.assertEqual(a.charge_urgency(), b.charge_urgency(),
                         "a 3% difference inside a band should not decide it")
        a.battery, b.battery = 30.0, 12.0          # clearly emptier
        self.assertGreater(b.charge_urgency(), a.charge_urgency())


class TestChargingNeverCausesACrash(unittest.TestCase):
    def test_repeated_drains_stay_clean(self):
        w = fleet_world(15)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        for _ in range(4):
            run(w, 30, sc)
            w.drain_batteries()
            run(w, 60, sc)
        self.assertEqual(w.collisions, 0)

    def test_nobody_teleports_while_the_fleet_charges(self):
        w = fleet_world(12)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 20, sc)
        w.drain_batteries()
        limit = max(r.speed for r in w.robots.values()) * 0.05 + 1e-6
        last = {rid: (r.x, r.y) for rid, r in w.robots.items()}
        for _ in range(3000):
            sc.keep_busy()
            w.tick(0.05)
            for rid, r in w.robots.items():
                moved = abs(r.x - last[rid][0]) + abs(r.y - last[rid][1])
                self.assertLessEqual(moved, limit,
                                     f"{rid} jumped {moved:.3f} squares in one tick")
                last[rid] = (r.x, r.y)
        self.assertEqual(w.collisions, 0)

    def test_a_failed_robot_gives_its_bay_back(self):
        """Otherwise one of four bays is lost for the rest of the run."""
        w = fleet_world(5)
        r = w.robots["R1"]
        r.battery = 9.0
        r._charge_checked_at = -99.0
        run(w, 20)
        w.fail_robot("R1")
        self.assertIsNone(r.charger)


if __name__ == "__main__":
    unittest.main()
