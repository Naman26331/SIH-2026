"""Phase 14 tests -- carrying on when the radio dies.

Run with:   python3 -m unittest discover -s tests -v

The claim being tested is one Anushka wanted to be able to say to a judge:
"the warehouse degrades, it doesn't stop, and it never becomes unsafe."

Before this phase that claim was FALSE. The "local safety reflex" read the
notebook of robots heard on the RADIO, so cutting the network made every robot
believe it was alone in the warehouse. Nothing collided only because nothing
was moving. These tests exist so it can never quietly become false again.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, InMemoryBus, Robot, RobotStatus, World, fleet_world
from scenarios import Scenarios

TOUCHING = 0.7


def busy_world(robots=5, seed=1, every=3.0):
    w = fleet_world(robots)
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


class TestSafetyDoesNotNeedTheRadio(unittest.TestCase):
    """The one that matters."""

    def test_robots_still_see_each_other_with_the_radio_dead(self):
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 50)
        someone_sees_someone = any(r.local_contacts for r in w.robots.values())
        self.assertTrue(someone_sees_someone,
                        "with the radio dead, no robot can see any other -- "
                        "the safety reflex is not local at all")

    def test_no_collisions_through_a_total_blackout(self):
        worst = [99.0]
        w, sc = busy_world()
        watch = lambda: worst.__setitem__(0, min(worst[0], closest(w)))
        run(w, sc, 40, watch)
        w.bus.packet_loss = 1.0
        run(w, sc, 160, watch)
        w.bus.packet_loss = 0.0
        run(w, sc, 260, watch)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(worst[0], TOUCHING,
                           f"robots came within {worst[0]:.2f} squares")

    def test_safety_holds_with_no_radio_at_all_from_the_start(self):
        """Not one message, ever. Sensors only."""
        worst = [99.0]
        w = fleet_world(5)
        w.bus = InMemoryBus(packet_loss=1.0, seed=2)
        for rid in w.robots:
            w.bus.register(rid)
        sc = Scenarios(w)
        sc.apply("orders", seed=2, every=3.0, limit=None)
        for _ in range(3000):
            sc.keep_busy()
            w.tick(0.05)
            worst[0] = min(worst[0], closest(w))
        self.assertEqual(w.collisions, 0)
        self.assertGreater(worst[0], TOUCHING)


class TestSafeMode(unittest.TestCase):
    def test_robots_notice_the_network_has_gone(self):
        w, sc = busy_world()
        run(w, sc, 40)
        self.assertEqual(w.kpis()["safe_mode"], 0)
        w.bus.packet_loss = 1.0
        run(w, sc, 50)
        self.assertEqual(w.kpis()["safe_mode"], len(w.robots))

    def test_they_slow_down(self):
        """03_ROBOT_AND_ROS2 section 9: NETWORK OFF -> reduce speed."""
        r = Robot("R1", Cell(2, 8), speed=2.5)
        self.assertEqual(r.travel_speed(), 2.5)
        r.safe_mode = True
        self.assertLess(r.travel_speed(), 2.5)

    def test_they_finish_what_they_are_carrying(self):
        w, sc = busy_world()
        run(w, sc, 40)
        before = w.kpis()["task_done"]
        w.bus.packet_loss = 1.0
        run(w, sc, 120)
        self.assertGreater(w.kpis()["task_done"], before,
                           "nothing was delivered after the network went down")

    def test_they_take_no_new_work(self):
        """No radio means no auction: nobody hears the bid or the claim.
        Taking a job anyway is how three robots ended up carrying the same
        parcel to the same square and wedging each other there."""
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 150)
        holders = {}
        for r in w.robots.values():
            if r.task:
                holders.setdefault(r.task.task_id, []).append(r.robot_id)
        duplicated = {k: v for k, v in holders.items() if len(v) > 1}
        self.assertFalse(duplicated, f"same job taken twice: {duplicated}")

    def test_nobody_gets_wedged_during_a_blackout(self):
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 160)
        for r in w.robots.values():
            self.assertLess(r.stalled_for(w.sim_time), 45.0,
                            f"{r.robot_id} went nowhere for "
                            f"{r.stalled_for(w.sim_time):.0f}s")

    def test_a_robot_that_hears_nobody_does_not_declare_everybody_dead(self):
        """If you cannot reach anyone, the one that has gone quiet is you.

        Without this, every robot concluded all the others had failed, released
        their jobs, and grabbed them itself.
        """
        w, sc = busy_world()
        run(w, sc, 40)
        assigned = {t.task_id: t.assigned_robot
                    for t in w.board.tasks.values()
                    if t.assigned_robot and not t.finished}
        w.bus.packet_loss = 1.0
        run(w, sc, 60)
        for r in w.robots.values():
            for tid, owner in assigned.items():
                mine = r.board.get(tid)
                if mine is not None and mine.assigned_robot not in (None, owner):
                    self.fail(f"{r.robot_id} reassigned {tid} from {owner} "
                              f"to {mine.assigned_robot} during a blackout")


class TestComingBack(unittest.TestCase):
    def test_safe_mode_clears_when_the_network_returns(self):
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 80)
        self.assertGreater(w.kpis()["safe_mode"], 0)
        w.bus.packet_loss = 0.0
        run(w, sc, 95)
        self.assertEqual(w.kpis()["safe_mode"], 0)

    def test_work_resumes_after_the_network_returns(self):
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 140)
        during = w.kpis()["task_done"]
        w.bus.packet_loss = 0.0
        run(w, sc, 260)
        after = w.kpis()["task_done"]
        self.assertGreater(after - during, 10,
                           "the warehouse never recovered after the network came back")

    def test_orders_missed_during_a_blackout_are_not_lost_for_ever(self):
        """The order system repeats itself. Without that, every order issued
        while the link was down would vanish."""
        w, sc = busy_world()
        run(w, sc, 40)
        w.bus.packet_loss = 1.0
        run(w, sc, 120)
        missed = [t.task_id for t in w.board.tasks.values()
                  if not t.finished and t.assigned_robot is None]
        self.assertTrue(missed, "no orders were missed, so this proves nothing")
        w.bus.packet_loss = 0.0
        run(w, sc, 300)
        still_unheard = [t for t in w.board.tasks.values()
                         if t.task_id in missed and not t.finished
                         and t.assigned_robot is None]
        self.assertLess(len(still_unheard), len(missed),
                        "orders issued during the blackout were never picked up")


class TestNothingElseBroke(unittest.TestCase):
    def test_a_lone_robot_never_enters_safe_mode(self):
        """It has nobody to hear, so silence means nothing."""
        w = World(bus=InMemoryBus(seed=1))
        r = w.add_robot(Robot("R1", Cell(2, 8)))
        r.set_goal(Cell(26, 8))
        for _ in range(600):
            w.tick(0.05)
        self.assertFalse(r.safe_mode)
        self.assertEqual(r.cell, Cell(26, 8))

    def test_a_parked_robot_is_recognised_by_sensors_alone(self):
        """A sensor sees a shape, not a status. Without watching whether the
        shape MOVES, a parked robot looks like one about to pull out, and
        everybody yields to it for ever.

        Tested on a bare Robot: inside a running world the real sensor sweep
        overwrites anything set by hand, so the fake sighting never survives.
        """
        r = Robot("R1", Cell(2, 8))
        sighting = [(5.0, 5.0, Cell(5, 5))]

        r.sense_robots(sighting, 0.0)
        seen_now = r._all_contacts(0.0)
        self.assertTrue(seen_now)
        self.assertFalse(seen_now[0]["known_still"],
                         "a shape seen for the first time cannot be known parked")

        # Still sitting there a couple of seconds later.
        for t in (0.5, 1.0, 1.5, 2.0):
            r.sense_robots(sighting, t)
        settled = r._all_contacts(2.0)
        self.assertTrue(settled[0]["known_still"],
                        "a shape that never moved was not recognised as parked")

        # It moves on, and the clock starts again.
        r.sense_robots([(6.0, 5.0, Cell(6, 5))], 2.5)
        moved = r._all_contacts(2.5)
        self.assertFalse(moved[0]["known_still"])

    def test_normal_running_is_unaffected(self):
        w, sc = busy_world()
        run(w, sc, 200)
        k = w.kpis()
        self.assertEqual(k["collisions"], 0)
        self.assertEqual(k["safe_mode"], 0)
        self.assertGreater(k["task_done"], 30)


if __name__ == "__main__":
    unittest.main(verbosity=2)
