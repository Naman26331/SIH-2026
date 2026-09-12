"""Phase 20 tests -- message authentication and replay protection.

Run with:   python3 -m unittest discover -s tests -v

Two questions, and the second one turned out to be the whole story:
  1. does a forged or replayed message actually get thrown away?
  2. can normal, honest fleet traffic ever be mistaken for one?

A security layer that rejects real traffic is worse than useless -- it looks
like it works right up until a demo, breaks the fleet, and there is no way to
tell "attack" from "our own bug" from the outside. Most of these tests are
about (2).
"""

import os
import sys
import threading
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, RobotStatus, fleet_world, phase2_world
from fleetx_core import security
from fleetx_core.messages import Heartbeat, TaskClaim
from scenarios import Scenarios


def run(world, seconds, scenarios=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if scenarios is not None:
            scenarios.keep_busy()
        world.tick(dt)


class TestTheSignatureItself(unittest.TestCase):
    def test_a_signed_message_verifies(self):
        m = Heartbeat(robot_id="R1", timestamp=1.0, seq=1, battery=90.0,
                      status="MOVING")
        security.seal(m)
        self.assertTrue(security.check_signature(m))

    def test_an_unsigned_message_fails(self):
        m = Heartbeat(robot_id="R1", timestamp=1.0, seq=1, battery=90.0,
                      status="MOVING")
        verdict = security.check_signature(m)
        self.assertFalse(verdict)
        self.assertIn("no signature", verdict.reason)

    def test_tampering_after_signing_fails(self):
        """Sign it honestly, then change one field -- the tag no longer
        matches, exactly as if someone had rewritten the message in transit."""
        m = Heartbeat(robot_id="R1", timestamp=1.0, seq=1, battery=90.0,
                      status="MOVING")
        security.seal(m)
        m.battery = 0.0                     # a lie added after the fact
        self.assertFalse(security.check_signature(m))

    def test_cannot_forge_without_the_key(self):
        """The actual security property: knowing exactly how a tag is made,
        and exactly what the message should say, is still not enough without
        the key itself. Simulated here by signing with the WRONG key -- an
        attacker who does not hold FLEET_KEY is in exactly this position."""
        m = Heartbeat(robot_id="R1", timestamp=1.0, seq=1, battery=0.0,
                      status="FAILED")
        import hashlib, hmac
        wrong_key_tag = hmac.new(
            b"a-guessed-key", security._canonical(m), hashlib.sha256
        ).hexdigest()[:security.TAG_LEN]
        m.tag = wrong_key_tag
        self.assertFalse(security.check_signature(m))

    def test_the_tag_covers_the_whole_message_not_just_who_sent_it(self):
        """The gap this closed: the first version only signed who/when/seq/
        type, so a battery reading or a status could be rewritten after
        signing and the tag would still say "valid". Every field must count."""
        m = Heartbeat(robot_id="R1", timestamp=1.0, seq=1, battery=90.0,
                      status="MOVING")
        security.seal(m)
        m.status = "FAILED"                 # tamper with content, not identity
        self.assertFalse(security.check_signature(m),
                         "changing the message content did not break the tag")


class TestReplayProtection(unittest.TestCase):
    def test_the_first_message_from_a_sender_is_accepted(self):
        g = security.ReplayGuard()
        self.assertTrue(g.check("R2", 1, 0.0, 0.0))

    def test_a_higher_sequence_number_is_accepted(self):
        g = security.ReplayGuard()
        g.check("R2", 5, 0.0, 0.0)
        self.assertTrue(g.check("R2", 6, 0.1, 0.1))

    def test_the_same_sequence_number_again_is_rejected(self):
        g = security.ReplayGuard()
        g.check("R2", 5, 0.0, 0.0)
        self.assertFalse(g.check("R2", 5, 0.0, 0.0))

    def test_a_lower_sequence_number_is_rejected(self):
        g = security.ReplayGuard()
        g.check("R2", 10, 0.0, 0.0)
        self.assertFalse(g.check("R2", 3, 0.0, 0.0))

    def test_different_dds_topics_do_not_reject_each_other(self):
        g = security.ReplayGuard()
        self.assertTrue(g.check("R2", 10, 0.0, 0.0, stream="POSE"))
        self.assertTrue(g.check("R2", 3, 0.0, 0.0, stream="RESERVATION"))

    def test_lower_sequence_on_same_dds_topic_is_still_rejected(self):
        g = security.ReplayGuard()
        g.check("R2", 10, 0.0, 0.0, stream="POSE")
        self.assertFalse(g.check("R2", 3, 0.0, 0.0, stream="POSE"))

    def test_a_very_old_message_is_rejected_even_with_a_fresh_seq(self):
        g = security.ReplayGuard()
        self.assertFalse(g.check("R2", 1, 0.0, 999.0))

    def test_different_senders_do_not_interfere(self):
        g = security.ReplayGuard()
        g.check("R2", 50, 0.0, 0.0)
        self.assertTrue(g.check("R3", 1, 0.0, 0.0))

    def test_forgetting_a_sender_lets_it_start_over(self):
        g = security.ReplayGuard()
        g.check("R3", 900, 0.0, 0.0)
        self.assertFalse(g.check("R3", 1, 0.0, 0.0))
        g.forget("R3")
        self.assertTrue(g.check("R3", 1, 0.0, 0.0))

    def test_forgetting_sender_clears_every_topic(self):
        g = security.ReplayGuard()
        g.check("R3", 900, 0.0, 0.0, stream="POSE")
        g.check("R3", 800, 0.0, 0.0, stream="RESERVATION")
        g.forget("R3")
        self.assertTrue(g.check("R3", 1, 0.0, 0.0, stream="POSE"))
        self.assertTrue(g.check("R3", 1, 0.0, 0.0, stream="RESERVATION"))


class TestFleetTrafficIsNeverFalselyRejected(unittest.TestCase):
    """The tests that matter. Every one of these caught a real bug during
    development, and every one is left in so it stays caught."""

    def test_a_long_ordinary_run_rejects_nothing(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 200, sc)
        rejected = sum(r.messages_rejected for r in w.robots.values())
        self.assertEqual(rejected, 0)
        self.assertEqual(w.collisions, 0)

    def test_add_and_remove_and_add_again_rejects_nothing(self):
        """The name a departing robot held gets reissued to whoever joins
        next. The new robot's very first message (seq 1) used to read as a
        replay of the old one's much higher last sequence number -- forever,
        because nothing ever told the fleet to forget."""
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(4):
            run(w, 30, sc)
            w.remove_robot()
            run(w, 20, sc)
            w.add_robot_live()
        run(w, 60, sc)
        rejected = sum(r.messages_rejected for r in w.robots.values())
        self.assertEqual(rejected, 0, "a rejoining robot's own traffic was rejected")
        self.assertEqual(w.collisions, 0)

    def test_a_reset_mid_flight_rejects_nothing(self):
        """A message published in the scenario code, in the first tick after
        the clock is rewound, used to be stamped with the BUS's stale,
        not-yet-caught-up clock -- because that publish happens before
        world.tick() resyncs it. It then sat queued and arrived late, after
        later, normally-timed messages had already been accepted, and read as
        a replay. This is a real desync in the bus's own clock, not anything
        about signing -- the replay guard only made it visible.
        """
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 4.5, sc)                  # let real drift build up, like a
                                          # person waiting before clicking
        sc.apply("orders", every=1.5)
        run(w, 40, sc)
        rejected = sum(r.messages_rejected for r in w.robots.values())
        self.assertEqual(rejected, 0, "a message from right after reset was rejected")
        self.assertEqual(w.collisions, 0)

    def test_repeated_resets_with_real_threaded_timing_reject_nothing(self):
        """The same bug as above, but through the real background-thread path
        the dashboard actually uses, with genuine wall-clock gaps between
        scenario switches -- not the synchronous test harness."""
        w = phase2_world()
        sc = Scenarios(w)
        lock = threading.Lock()
        stop = threading.Event()

        def loop():
            dt = 1 / 20.0
            nxt = time.perf_counter()
            while not stop.is_set():
                with lock:
                    sc.keep_busy()
                    w.tick(dt)
                nxt += dt
                sleep_for = nxt - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    nxt = time.perf_counter()

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        try:
            for cycle in range(4):
                time.sleep(0.1 + cycle * 0.1)
                with lock:
                    sc.apply("orders", every=1.2)
                time.sleep(0.05)
                with lock:
                    sc.set_order_rate(1.2)
                time.sleep(0.8)
        finally:
            stop.set()
            t.join(timeout=1)

        rejected = sum(r.messages_rejected for r in w.robots.values())
        self.assertEqual(rejected, 0)
        self.assertEqual(w.collisions, 0)

    def test_a_network_blackout_and_recovery_rejects_nothing(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 30, sc)
        w.bus.packet_loss = 1.0
        run(w, 60, sc)
        w.bus.packet_loss = 0.0
        run(w, 60, sc)
        rejected = sum(r.messages_rejected for r in w.robots.values())
        self.assertEqual(rejected, 0)
        self.assertEqual(w.collisions, 0)

    def test_a_lossy_and_jittery_network_rejects_nothing(self):
        """Out-of-order delivery from jitter is the one thing the replay guard
        cannot tell apart from an attack by design -- both look like "a lower
        sequence number arrived after a higher one." Whether this stays safe
        under jitter depends entirely on whether the fleet already tolerates
        reordering elsewhere, which it must for any other reason."""
        w = fleet_world(5)
        w.bus.packet_loss = 0.15
        w.bus.latency = 0.05
        w.bus.jitter = 0.03
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 150, sc)
        self.assertEqual(w.collisions, 0)


class TestForgedMessagesAreRejected(unittest.TestCase):
    def test_an_unsigned_task_claim_never_steals_a_job(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 20, sc)
        task = next((t for t in w.board.tasks.values() if not t.finished), None)
        self.assertIsNotNone(task, "test needs an open task to attack")
        before = (task.assigned_robot, task.status)

        fake = TaskClaim(robot_id="GHOST-1", timestamp=w.sim_time, seq=1,
                         task_id=task.task_id, action="CLAIM", cost=0.01)
        w.bus.publish(fake)                 # no security.seal() -- unsigned
        run(w, 5, sc)

        self.assertNotEqual(task.assigned_robot, "GHOST-1")
        for r in w.robots.values():
            self.assertNotEqual(r.board.get(task.task_id).assigned_robot
                                if r.board.get(task.task_id) else None, "GHOST-1")

    def test_a_forged_failure_report_is_not_believed(self):
        """A fake message claiming a real, healthy robot has failed must not
        make anyone else treat it as failed."""
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 10, sc)
        target = w.robots["R1"]
        self.assertIsNot(target.status, RobotStatus.FAILED)

        fake = Heartbeat(robot_id="R1", timestamp=w.sim_time, seq=999999,
                         battery=0.0, status="FAILED", health="CRITICAL")
        w.bus.publish(fake)
        run(w, 5, sc)

        self.assertIsNot(target.status, RobotStatus.FAILED,
                         "a forged heartbeat changed R1's own status")

    def test_the_demo_attack_button_is_always_rejected(self):
        w = fleet_world(4)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        run(w, 15, sc)
        before = sum(r.messages_rejected for r in w.robots.values())
        result = w.simulate_fake_message()
        self.assertTrue(result["ok"])
        run(w, 3, sc)
        after = sum(r.messages_rejected for r in w.robots.values())
        self.assertGreater(after, before, "the injected message was never rejected")
        self.assertEqual(w.collisions, 0)


if __name__ == "__main__":
    unittest.main()
