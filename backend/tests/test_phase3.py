"""Phase 3 tests -- robots talking, and coping when the talking fails.

Run with:   python3 -m unittest discover -s tests -v

Roadmap Phase 3's success test is one line: "R1 can see R2/R3 intent."
Most of these tests are about what happens when messages DON'T arrive, because
04_DECENTRALIZED_FLEET_PROTOCOL section 8 is blunt about it: "The protocol must
never assume every message arrives."
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import (Cell, Heartbeat, InMemoryBus, IntentUpdate, PoseUpdate,
                         Robot, RobotStatus, World, from_dict, phase2_world)


def run(world, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        world.tick(dt)



# Phase 5 added square booking, which is what stops the crashes. Tests below
# that are about the EARLIER behaviour (robots driving blind) now switch it
# off explicitly with reservations_enabled=False. That is not a workaround --
# Phase 15 runs the "before" side of the benchmark exactly the same way.


def three_robots(booking=True, **bus_kwargs):
    w = World(bus=InMemoryBus(seed=42, **bus_kwargs), reservations_enabled=booking)
    w.add_robot(Robot("R1", Cell(2, 8)))
    w.add_robot(Robot("R2", Cell(26, 8)))
    w.add_robot(Robot("R3", Cell(13, 0)))
    w.get("R1").set_goal(Cell(26, 8))
    w.get("R2").set_goal(Cell(2, 8))
    w.get("R3").set_goal(Cell(13, 15))
    return w


class TestMessages(unittest.TestCase):
    def test_messages_survive_a_round_trip_through_plain_data(self):
        """ROS 2 and any log replay will rebuild messages from plain fields."""
        originals = [
            Heartbeat("R1", 1.0, 3, battery=88.0, status="MOVING"),
            PoseUpdate("R1", 1.0, 4, x=2.5, y=8.0, cell=(2, 8), velocity=2.5, heading="E"),
            IntentUpdate("R1", 1.0, 5, x=2.5, y=8.0, velocity=2.5,
                         destination=(26, 8), planned_nodes=[(3, 8), (4, 8)],
                         node_etas=[0.4, 0.8], eta_destination=0.8,
                         priority=5, status="MOVING"),
        ]
        for msg in originals:
            rebuilt = from_dict(json.loads(json.dumps(msg.to_dict())))
            self.assertEqual(rebuilt.robot_id, msg.robot_id)
            self.assertEqual(rebuilt.seq, msg.seq)
            self.assertEqual(type(rebuilt), type(msg))

    def test_unknown_message_type_is_rejected(self):
        with self.assertRaises(ValueError):
            from_dict({"type": "NONSENSE", "robot_id": "R1"})

    def test_intent_says_when_it_reaches_each_square(self):
        intent = IntentUpdate("R1", 0.0, 1, x=0, y=0, velocity=2.5,
                              destination=(3, 0), planned_nodes=[(1, 0), (2, 0), (3, 0)],
                              node_etas=[0.4, 0.8, 1.2], eta_destination=1.2,
                              priority=5, status="MOVING")
        self.assertAlmostEqual(intent.eta_for((2, 0)), 0.8)
        self.assertIsNone(intent.eta_for((9, 9)), "should not claim squares it never visits")

    def test_etas_line_up_with_how_fast_the_robot_actually_moves(self):
        r = Robot("R1", Cell(2, 8), speed=2.0)
        r.set_goal(Cell(8, 8))
        r.decide(World().grid)
        etas = r.node_etas()
        self.assertEqual(len(etas), len(r.path))
        self.assertAlmostEqual(etas[0], 0.5, places=2)      # 1 square at 2/sec
        for a, b in zip(etas, etas[1:]):
            self.assertAlmostEqual(b - a, 0.5, places=2)


class TestRobotsCanHearEachOther(unittest.TestCase):
    def test_r1_can_see_r2_and_r3_intent(self):
        """This IS the roadmap's Phase 3 success test."""
        w = three_robots()
        run(w, 2)
        r1 = w.get("R1")
        self.assertEqual(sorted(n.robot_id for n in r1.fleet.known()), ["R2", "R3"])
        for rid in ("R2", "R3"):
            note = r1.fleet.get(rid)
            self.assertIsNotNone(note.destination, f"R1 never heard where {rid} is going")
            self.assertTrue(note.planned_nodes, f"R1 never heard {rid}'s planned route")

    def test_a_robot_does_not_hear_itself(self):
        w = three_robots()
        run(w, 2)
        for rid, robot in w.robots.items():
            self.assertNotIn(rid, [n.robot_id for n in robot.fleet.known()])

    def test_every_robot_builds_its_own_picture(self):
        w = three_robots()
        run(w, 2)
        for rid, robot in w.robots.items():
            others = sorted(n.robot_id for n in robot.fleet.known())
            self.assertEqual(len(others), 2, f"{rid} should hear the other two")

    def test_who_is_heading_for_a_square(self):
        """The raw material Phase 4 needs to spot a conflict early."""
        w = three_robots()
        run(w, 2)
        coming = w.get("R1").fleet.who_is_heading_for((13, 8), w.sim_time)
        self.assertTrue(coming, "R1 should know somebody is heading for the junction")
        etas = [eta for _, eta in coming]
        self.assertEqual(etas, sorted(etas), "should be listed soonest first")
        for _, eta in coming:
            self.assertGreaterEqual(eta, 0.0)

    def test_intent_is_not_resent_every_tick(self):
        """Repeating an unchanged plan 20x a second would flood the network."""
        w = three_robots()
        run(w, 3)
        intents = sum(1 for m in w.bus.recent if isinstance(m, IntentUpdate))
        self.assertLess(intents, 20, "intent should only go out when the plan changes")


class TestWhenTheRadioFails(unittest.TestCase):
    def test_silenced_robot_goes_stale_then_comes_back(self):
        w = three_robots()
        run(w, 2)
        r1 = w.get("R1")
        self.assertFalse(r1.fleet.get("R2").is_stale(w.sim_time))

        w.bus.silence("R2", True)
        run(w, 1.0)
        self.assertFalse(r1.fleet.get("R2").is_stale(w.sim_time), "2s timeout, not 1s")
        run(w, 2.0)
        self.assertTrue(r1.fleet.get("R2").is_stale(w.sim_time), "should be stale after ~2s")

        w.bus.silence("R2", False)
        run(w, 1.0)
        self.assertFalse(r1.fleet.get("R2").is_stale(w.sim_time), "should recover")

    def test_stale_robot_is_dropped_from_the_trusted_list(self):
        w = three_robots()
        run(w, 2)
        w.bus.silence("R2", True)
        run(w, 3)
        r1 = w.get("R1")
        self.assertEqual([n.robot_id for n in r1.fleet.fresh(w.sim_time)], ["R3"])
        self.assertEqual([n.robot_id for n in r1.fleet.stale(w.sim_time)], ["R2"])

    def test_a_stale_robot_is_not_counted_as_heading_anywhere(self):
        w = three_robots()
        run(w, 2)
        w.bus.silence("R3", True)
        run(w, 3)
        coming = w.get("R1").fleet.who_is_heading_for((13, 8), w.sim_time)
        self.assertNotIn("R3", [rid for rid, _ in coming])

    def test_robots_still_do_their_jobs_with_80_percent_loss(self):
        w = World(bus=InMemoryBus(packet_loss=0.8, latency=0.05, jitter=0.05, seed=1),
                  reservations_enabled=False)
        w.add_robot(Robot("R1", Cell(2, 0)))
        w.add_robot(Robot("R2", Cell(2, 12)))
        w.get("R1").set_goal(Cell(26, 0))
        w.get("R2").set_goal(Cell(26, 12))
        run(w, 40)
        self.assertEqual(w.get("R1").cell, Cell(26, 0))
        self.assertEqual(w.get("R2").cell, Cell(26, 12))
        self.assertGreater(w.bus.dropped, 0, "the test is pointless if nothing was lost")

    def test_lost_messages_are_noticed_and_counted(self):
        w = three_robots(packet_loss=0.5)
        run(w, 5)
        missed = sum(n.missed for n in w.get("R1").fleet.known())
        self.assertGreater(missed, 0, "sequence gaps should reveal the losses")

    def test_delayed_messages_arrive_late_not_early(self):
        bus = InMemoryBus(latency=0.5, seed=3)
        bus.register("R1"); bus.register("R2")
        bus.set_time(0.0)
        bus.publish(Heartbeat("R2", 0.0, 1, battery=100.0, status="IDLE"))
        self.assertEqual(bus.poll("R1"), [], "should not arrive instantly")
        bus.set_time(0.4)
        self.assertEqual(bus.poll("R1"), [], "still in flight at 0.4s")
        bus.set_time(0.6)
        self.assertEqual(len(bus.poll("R1")), 1, "should have landed by 0.6s")

    def test_out_of_order_pose_does_not_rewind_the_picture(self):
        """A delayed message overtaking a newer one must not un-do the newer one."""
        w = World(bus=InMemoryBus(seed=5))
        r1 = w.add_robot(Robot("R1", Cell(0, 0)))
        w.add_robot(Robot("R2", Cell(5, 0)))
        r1.fleet.ingest(PoseUpdate("R2", 10.0, 2, x=9.0, y=0.0, cell=(9, 0), velocity=1.0), 10.0)
        r1.fleet.ingest(PoseUpdate("R2", 5.0, 1, x=5.0, y=0.0, cell=(5, 0), velocity=1.0), 10.1)
        self.assertEqual(r1.fleet.get("R2").cell, (9, 0), "old news must not overwrite new")

    def test_a_failed_robot_stops_talking(self):
        w = three_robots()
        run(w, 2)
        w.get("R2").status = RobotStatus.FAILED
        before = w.get("R2").messages_sent
        run(w, 2)
        self.assertEqual(w.get("R2").messages_sent, before)


class TestNothingElseBroke(unittest.TestCase):
    def test_snapshot_is_still_json_safe(self):
        w = three_robots()
        run(w, 3)
        json.dumps(w.snapshot())

    def test_comms_do_not_change_where_robots_end_up(self):
        """Phase 3 adds talking, not behaviour. Routes must be identical."""
        quiet = World(bus=InMemoryBus(seed=1), reservations_enabled=False)
        quiet.add_robot(Robot("R1", Cell(2, 8)))
        quiet.get("R1").set_goal(Cell(26, 8))
        run(quiet, 30)

        chatty = three_robots(booking=False)
        run(chatty, 30)

        self.assertEqual(quiet.get("R1").cell, chatty.get("R1").cell)

    def test_hearing_each_other_is_not_enough_to_avoid_a_crash(self):
        """Phase 3 is listening only. Talking without booking still crashes --
        which is precisely why Phase 5 exists."""
        from scenarios import Scenarios
        w = phase2_world()
        w.reservations_enabled = False
        sc = Scenarios(w)
        sc.apply("intersection")
        for _ in range(500):
            sc.keep_busy()
            w.tick(0.05)
        self.assertGreater(w.collisions, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
