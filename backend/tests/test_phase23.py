"""Phase 23 tests -- robot health.

Run with:   python3 -m unittest discover -s tests -v

The point is to spot a struggling robot BEFORE it fails, from signals it
genuinely measures about itself -- not to invent telemetry a grid simulator
does not have. Most of these tests check that the score means what it says,
and that a low score never turns into a new way to cause a collision.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, RobotStatus, fleet_world, phase2_world
from fleetx_core.health import CRITICAL, HEALTHY, SERVICE_SOON, WARNING, compute
from scenarios import Scenarios


def run(world, seconds, scenarios=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if scenarios is not None:
            scenarios.keep_busy()
        world.tick(dt)


class TestTheScoreMeansWhatItSays(unittest.TestCase):
    def test_a_perfect_robot_is_fully_healthy(self):
        r = compute(battery=100, stalled_for=0, comms_silence=0,
                   reroutes_recent=0, backouts_recent=0)
        self.assertEqual(r.band, HEALTHY)
        self.assertEqual(r.score, 100.0)
        self.assertEqual(r.reasons, [])

    def test_a_robot_stuck_a_long_time_is_not_healthy(self):
        r = compute(battery=100, stalled_for=30, comms_silence=0,
                   reroutes_recent=0, backouts_recent=0)
        self.assertNotEqual(r.band, HEALTHY)
        self.assertIn("stuck", r.sentence())

    def test_a_quiet_radio_is_not_healthy(self):
        r = compute(battery=100, stalled_for=0, comms_silence=10,
                   reroutes_recent=0, backouts_recent=0)
        self.assertNotEqual(r.band, HEALTHY)
        self.assertIn("not heard from", r.sentence())

    def test_everything_wrong_at_once_is_critical(self):
        r = compute(battery=5, stalled_for=30, comms_silence=10,
                   reroutes_recent=6, backouts_recent=3)
        self.assertEqual(r.band, CRITICAL)

    def test_a_wedge_and_reverse_counts_for_more_than_an_ordinary_reroute(self):
        """Getting wedged mid-square and having to back out is a much louder
        signal than an ordinary replan -- it means the robot could not even
        finish the step it had already committed to. One wedge is worth
        THREE ordinary reroutes by design (see the 3x in compute()); compare
        a single reroute against a single wedge, which must not be equal."""
        ordinary = compute(battery=100, stalled_for=0, comms_silence=0,
                           reroutes_recent=1, backouts_recent=0)
        wedged = compute(battery=100, stalled_for=0, comms_silence=0,
                         reroutes_recent=0, backouts_recent=1)
        self.assertLess(wedged.score, ordinary.score)
        # And the design ratio itself: three ordinary reroutes should land at
        # the same score as one wedge, because that is the 3x weighting.
        three_reroutes = compute(battery=100, stalled_for=0, comms_silence=0,
                                 reroutes_recent=3, backouts_recent=0)
        self.assertEqual(three_reroutes.score, wedged.score)

    def test_a_short_wait_is_not_reported_as_stuck(self):
        """Waiting your turn at a junction is normal, not a health problem.
        Only long, genuine stalls should ever show up."""
        r = compute(battery=100, stalled_for=1.5, comms_silence=0,
                   reroutes_recent=0, backouts_recent=0)
        self.assertEqual(r.band, HEALTHY)
        self.assertEqual(r.reasons, [])

    def test_bands_are_ordered_sensibly(self):
        scores = [compute(battery=100, stalled_for=s, comms_silence=0,
                          reroutes_recent=0, backouts_recent=0).score
                 for s in (0, 8, 16, 24)]
        self.assertEqual(scores, sorted(scores, reverse=True))


class TestRobotsTrackTheirOwnHealth(unittest.TestCase):
    def test_a_healthy_fleet_stays_healthy(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 100, sc)
        bands = {r.robot_id: r.health_band for r in w.robots.values()}
        self.assertTrue(all(b == HEALTHY for b in bands.values()), bands)

    def test_a_walled_in_robot_is_flagged(self):
        """Phase 8's own scenario: wall a robot in completely. It cannot move
        no matter how long it waits, and health should say so."""
        w = phase2_world()
        r = w.robots["R1"]
        from fleetx_core.grid import CellKind
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            w.add_obstacle(Cell(r.cell.x + dx, r.cell.y + dy))
        r.set_goal(Cell(r.cell.x + 5, r.cell.y))
        for _ in range(700):        # 35s, comfortably past STALL_CEILING
            w.tick(0.05)
        self.assertIn(r.health_band, (WARNING, SERVICE_SOON, CRITICAL))
        self.assertTrue(r.health_reasons)

    def test_a_robot_that_cannot_hear_anyone_shows_a_quiet_radio(self):
        """Silencing a robot's TRANSMITTER (the "Silence" button) stops
        others hearing IT -- it says nothing about whether IT can hear THEM,
        which is what this health signal actually measures (the same one
        Phase 14's safe mode already uses). A real blackout is what makes a
        robot's own reception go quiet: the order system keeps announcing on
        its own schedule too, so silencing individual robots is not enough --
        anything at all arriving, from anyone, resets the clock."""
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 10, sc)
        w.bus.packet_loss = 1.0
        run(w, 10, sc)
        r2 = w.robots["R2"]
        self.assertNotEqual(r2.health_band, HEALTHY)
        self.assertTrue(any("not heard" in reason for reason in r2.health_reasons),
                        r2.health_reasons)

    def test_recovering_returns_it_to_healthy(self):
        """The whole point of a summary score: it must go back up once the
        problem clears, not stay flagged forever."""
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 10, sc)
        w.bus.packet_loss = 1.0
        run(w, 10, sc)
        w.bus.packet_loss = 0.0
        run(w, 15, sc)
        self.assertEqual(w.robots["R2"].health_band, HEALTHY)

    def test_a_heartbeat_carries_the_real_band(self):
        """The Heartbeat message had a health field from early on, defaulted
        to the placeholder "OK". Genuinely degrade a robot and check the next
        heartbeat it actually PUBLISHES carries the real, computed band --
        not a value poked in by the test, which the next tick's
        update_health() would just recompute over anyway."""
        w = phase2_world()
        r1 = w.robots["R1"]
        from fleetx_core.grid import CellKind
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            w.add_obstacle(Cell(r1.cell.x + dx, r1.cell.y + dy))
        r1.set_goal(Cell(r1.cell.x + 5, r1.cell.y))
        for _ in range(700):
            w.tick(0.05)
        self.assertNotEqual(r1.health_band, HEALTHY, "test needs a genuinely unhealthy robot")

        import fleetx_core.messages as messages_mod
        seen = []
        orig_publish = type(r1)._publish
        def spy(self, bus, message):
            if self is r1 and isinstance(message, messages_mod.Heartbeat):
                seen.append(message.health)
            orig_publish(self, bus, message)
        type(r1)._publish = spy
        try:
            for _ in range(20):
                w.tick(0.05)
                if seen:
                    break
        finally:
            type(r1)._publish = orig_publish

        self.assertTrue(seen, "R1 never published a heartbeat")
        self.assertEqual(seen[-1], r1.health_band)
        self.assertNotEqual(seen[-1], "OK", "still showing the old placeholder")


class TestACriticalRobotStopsTakingNewWork(unittest.TestCase):
    def test_it_does_not_bid_while_critical(self):
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 6, sc)
        task = next((t for t in w.board.tasks.values() if not t.finished), None)
        self.assertIsNotNone(task)
        r = w.robots["R1"]
        r.health_band = "CRITICAL"
        self.assertIsNone(r.cost_of(task, w.grid, w.sim_time))

    def test_it_still_finishes_a_job_it_is_already_carrying(self):
        """Doc 23's own policy: reduce NEW assignments, finish what is already
        in hand. Dropping a parcel mid-aisle because health dipped would be
        worse than the problem it is trying to avoid."""
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        r = w.robots["R1"]
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
            if r.task is not None and r.task.status.value == "CARRYING":
                break
        self.assertIsNotNone(r.task, "R1 never picked anything up, test proves nothing")
        r.health_band = "CRITICAL"
        held = r.task.task_id
        for _ in range(600):
            sc.keep_busy()
            w.tick(0.05)
            if r.task is None:
                break
        # It may have finished (task is None because done) or still be
        # carrying the SAME job -- either is fine. Picking up something else
        # entirely, or abandoning it, is not.
        if r.task is not None:
            self.assertEqual(r.task.task_id, held)

    def test_turning_critical_never_causes_a_collision(self):
        w = fleet_world(10)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 20, sc)
        for r in list(w.robots.values())[:3]:
            r.health_band = "CRITICAL"
        run(w, 100, sc)
        self.assertEqual(w.collisions, 0)


class TestTheDashboardGetsHonestNumbers(unittest.TestCase):
    def test_kpis_count_struggling_robots(self):
        w = fleet_world(3)
        w.robots["R1"].health_band = "CRITICAL"
        w.robots["R2"].health_band = "SERVICE_SOON"
        k = w.kpis()
        self.assertEqual(k["health_critical"], 1)
        self.assertEqual(k["health_service_soon"], 1)

    def test_a_robots_own_dict_carries_its_health(self):
        w = fleet_world(3)
        d = w.robots["R1"].to_dict(0.0)
        self.assertIn("health_score", d)
        self.assertIn("health_band", d)
        self.assertIn("health_reasons", d)


if __name__ == "__main__":
    unittest.main()
