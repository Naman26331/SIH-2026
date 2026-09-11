"""Phase 12 tests -- demand prediction and pre-positioning.

Run with:   python3 -m unittest discover -s tests -v

Two things are being checked, and the second matters far more than the first:

  1. the model learns something real, and says "no idea" when there is nothing
     to learn -- a predictor that always has an opinion is not a predictor; and
  2. acting on it can never hurt. A wrong guess must cost one short empty drive
     and nothing else: no collision, no job delayed, no robot in the way.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import Cell, RobotStatus, fleet_world, phase2_world
from fleetx_core.demand import MIN_ORDERS, DemandModel
from fleetx_core.grid import CellKind
from scenarios import Scenarios


def run(world, seconds, scenarios=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if scenarios is not None:
            scenarios.keep_busy()
        world.tick(dt)


class TestTheModelIsHonest(unittest.TestCase):
    def test_it_says_nothing_before_it_has_seen_anything(self):
        self.assertIsNone(DemandModel().predict(0.0))

    def test_it_says_nothing_after_only_a_couple_of_orders(self):
        m = DemandModel()
        for i in range(MIN_ORDERS - 1):
            m.record("A", float(i))
        self.assertIsNone(m.predict(10.0), "it guessed from almost no data")

    def test_it_says_nothing_when_orders_are_spread_evenly(self):
        """The important one. With no pattern the honest answer is "no idea",
        and a model that names a winner anyway is just amplifying noise."""
        m = DemandModel()
        for i in range(80):
            m.record("ABCD"[i % 4], float(i))
        self.assertIsNone(m.predict(80.0))

    def test_it_finds_a_real_pattern(self):
        m = DemandModel()
        for i in range(80):
            m.record("C" if i % 4 else "ABD"[i % 3], float(i))
        pred = m.predict(80.0)
        self.assertIsNotNone(pred)
        self.assertEqual(pred.zone, "C")
        self.assertGreater(pred.lift, 1.0)

    def test_it_can_explain_itself_in_one_sentence(self):
        m = DemandModel()
        for i in range(60):
            m.record("C" if i % 4 else "A", float(i))
        pred = m.predict(60.0)
        self.assertIn("aisle C", pred.reason)
        self.assertIn("%", pred.reason)
        self.assertIn("of the last", pred.reason)

    def test_old_orders_stop_counting(self):
        """Otherwise it describes where the work used to be, for ever."""
        m = DemandModel(half_life=30.0)
        for i in range(40):
            m.record("A", float(i))
        self.assertEqual(m.predict(40.0).zone if m.predict(40.0) else "A", "A")
        for i in range(40):
            m.record("C", 200.0 + i)
        pred = m.predict(240.0)
        self.assertIsNotNone(pred)
        self.assertEqual(pred.zone, "C", "it was still living in the past")

    def test_the_answer_does_not_depend_on_how_often_you_ask(self):
        """Fading happens on the clock, not per call. A model that drifts when
        you look at it twice is not a model."""
        a, b = DemandModel(), DemandModel()
        for i in range(40):
            a.record("C" if i % 3 else "A", float(i))
            b.record("C" if i % 3 else "A", float(i))
            for _ in range(5):
                b.predict(float(i))              # pestered constantly
        self.assertAlmostEqual(a.ranked(60.0)[0][1], b.ranked(60.0)[0][1], places=6)


class TestRobotsLearnWithoutBeingTold(unittest.TestCase):
    def test_every_robot_builds_its_own_model_from_the_radio(self):
        """No server holds this. Each robot learns from the announcements it
        hears, the same way it learns the job board."""
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 120, sc)
        seen = [r.demand.total for r in w.robots.values()]
        self.assertTrue(all(n > 10 for n in seen),
                        f"some robot learned nothing: {seen}")

    def test_robots_broadly_agree_with_each_other(self):
        w = fleet_world(6)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 200, sc)
        opinions = {r.robot_id: (r.demand.predict(w.sim_time).zone
                                 if r.demand.predict(w.sim_time) else None)
                    for r in w.robots.values()}
        top = max(set(opinions.values()), key=list(opinions.values()).count)
        agree = list(opinions.values()).count(top)
        self.assertGreaterEqual(agree, len(opinions) - 1,
                                f"robots disagree about where the work is: {opinions}")


class TestGuessingWrongIsHarmless(unittest.TestCase):
    """The tests that actually matter."""

    def test_a_busy_robot_never_pre_positions(self):
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.task is not None:
                    self.assertIsNone(
                        r.staging, f"{r.robot_id} was staging while holding a job")

    def test_a_job_immediately_beats_a_hunch(self):
        w = fleet_world(6)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        saw_staging = False
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.staging is not None:
                    saw_staging = True
                if r.task is not None and r.task.status.value == "CARRYING":
                    self.assertIsNone(r.staging)
        self.assertTrue(saw_staging, "nothing ever pre-positioned, test proves nothing")

    def test_it_never_waits_on_a_square_somebody_needs(self):
        """A robot guessing wrong must not be standing on a picking face, a
        packing bay or a charger."""
        w = fleet_world(10)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        faces = {s.face for s in w.inventory.by_name.values()}
        for _ in range(5000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.staging is None:
                    continue
                self.assertIs(w.grid.kind(r.staging), CellKind.FLOOR)
                self.assertNotIn(r.staging, faces,
                                 f"{r.robot_id} parked on a picking face")

    def test_charging_always_wins_over_a_hunch(self):
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 30, sc)
        w.drain_batteries()
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.charger is not None or r.status is RobotStatus.CHARGING:
                    self.assertIsNone(
                        r.staging, f"{r.robot_id} went sightseeing on {r.battery:.0f}%")

    def test_a_stopped_fleet_does_not_wander_off(self):
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 40, sc)
        sc.apply("stop")
        where = {rid: (r.x, r.y) for rid, r in w.robots.items()}
        run(w, 30, sc)
        for rid, r in w.robots.items():
            self.assertEqual((r.x, r.y), where[rid],
                             f"{rid} pre-positioned after Stop all")

    def test_no_collisions_with_prediction_on(self):
        w = fleet_world(15)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 200, sc)
        self.assertEqual(w.collisions, 0)

    def test_nobody_teleports_while_pre_positioning(self):
        w = fleet_world(12)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        limit = max(r.speed for r in w.robots.values()) * 0.05 + 1e-6
        last = {rid: (r.x, r.y) for rid, r in w.robots.items()}
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
            for rid, r in w.robots.items():
                moved = abs(r.x - last[rid][0]) + abs(r.y - last[rid][1])
                self.assertLessEqual(moved, limit,
                                     f"{rid} jumped {moved:.3f} squares")
                last[rid] = (r.x, r.y)
        self.assertEqual(w.collisions, 0)

    def test_they_do_not_all_pile_onto_the_same_square(self):
        """They did. Four robots picked the identical waiting square and held
        for each other for two minutes.

        The first version asked "is anybody already closer?" -- which reads the
        radio. Cut the network and every robot believes it is alone, so all of
        them answered no. Exactly the mistake Phase 14 was about. They now
        scatter using their own names, which need no messages and cannot go
        stale.
        """
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=6.0)          # slow, so robots are often idle
        clashes = 0
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            spots = [r.staging for r in w.robots.values() if r.staging is not None]
            if len(spots) != len(set(spots)):
                clashes += 1
        self.assertEqual(clashes, 0, "two robots aimed at the same waiting square")

    def test_a_blackout_does_not_leave_robots_wedged(self):
        """With no radio there is no new work arriving, so there is nothing to
        get ahead of -- and a robot that cannot hear the others has no idea
        where they are going. Guessing is a luxury for when the radio works."""
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 40, sc)
        w.bus.packet_loss = 1.0
        run(w, 160, sc)
        for r in w.robots.values():
            self.assertIsNone(r.staging, f"{r.robot_id} was guessing with no radio")
            self.assertLess(r.stalled_for(w.sim_time), 45.0,
                            f"{r.robot_id} went nowhere for "
                            f"{r.stalled_for(w.sim_time):.0f}s")
        self.assertEqual(w.collisions, 0)

    def test_a_hunch_is_given_up_if_it_does_not_work_out(self):
        """A guess must never be defended. If getting there is not working,
        stand where you are and wait for real work instead."""
        w = fleet_world(6)
        sc = Scenarios(w)
        sc.apply("orders", every=6.0)
        worst = 0.0
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.staging is not None:
                    worst = max(worst, r._still_for)
        self.assertLess(worst, 20.0,
                        f"a robot spent {worst:.0f}s stuck chasing a guess")

    def test_turning_it_off_changes_nothing_else(self):
        """With the switch off the fleet must behave exactly as it did before
        this phase existed -- that is what makes it safe to ship."""
        w = fleet_world(8)
        w.prepositioning = False
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        run(w, 200, sc)
        self.assertEqual(w.collisions, 0)
        for r in w.robots.values():
            self.assertIsNone(r.staging)
            self.assertEqual(r.prepositions, 0)


class TestItExplainsItself(unittest.TestCase):
    def test_the_reason_names_a_shelf_and_says_why(self):
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        reasons = []
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.staging_why:
                    reasons.append(r.staging_why)
        self.assertTrue(reasons, "no robot ever explained a move")
        for why in reasons[:20]:
            self.assertIn("pre-positioned near", why)
            self.assertIn("busy zone", why)
            self.assertIn("% of the last", why)

    def test_the_dashboard_gets_the_numbers_behind_it(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 150, sc)
        view = w.demand_view()
        self.assertIn("zones", view)
        self.assertIn("orders_seen", view)
        self.assertTrue(view["zones"])
        self.assertEqual(sum(1 for z in view["zones"] if z["zone"] == "A"), 1)


if __name__ == "__main__":
    unittest.main()
