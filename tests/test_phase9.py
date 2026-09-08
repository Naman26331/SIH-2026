"""Phase 9 tests -- real jobs, and robots bidding for them.

Run with:   python3 -m unittest discover -s tests -v

Two of these exist because of bugs that actually happened: a robot that had
been switched off went on to win an auction and collect a parcel, and a dead
robot's job stayed listed as "R1's" for ever so nobody picked it up.
"""

import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import (Cell, RobotStatus, Task, TaskBoard, TaskClaim,
                         TaskStatus, auction_winner, bid_cost, from_dict,
                         phase2_world)
from scenarios import OrderGenerator, Scenarios

TOUCHING = 0.7


def run(world, seconds, sc=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if sc:
            sc.keep_busy()
        world.tick(dt)


def quiet_world():
    w = phase2_world()
    sc = Scenarios(w)
    sc.auto = False
    for r in w.robots.values():
        r.clear_goal()
    return w, sc


class TestTheBid(unittest.TestCase):
    """06_TASK_ALLOCATION section 1: cheapest robot, not the nearest."""

    def test_further_away_costs_more(self):
        near = bid_cost(4, 20, battery=90)
        far = bid_cost(20, 20, battery=90)
        self.assertLess(near, far)

    def test_a_nearly_flat_robot_is_a_bad_bet_even_when_closest(self):
        closest_but_flat = bid_cost(4, 20, battery=15)
        further_but_charged = bid_cost(12, 20, battery=90)
        self.assertGreater(closest_but_flat, further_but_charged)

    def test_a_healthy_battery_adds_nothing(self):
        self.assertEqual(bid_cost(5, 10, battery=90), bid_cost(5, 10, battery=100))

    def test_an_urgent_job_looks_cheaper_to_take_on(self):
        self.assertLess(bid_cost(5, 10, 90, task_priority=9),
                        bid_cost(5, 10, 90, task_priority=5))

    def test_already_stuck_makes_you_a_worse_bet(self):
        self.assertGreater(bid_cost(5, 10, 90, waiting=8.0),
                           bid_cost(5, 10, 90, waiting=0.0))

    def test_lowest_bid_wins(self):
        self.assertEqual(auction_winner({"R1": 7.2, "R2": 4.1, "R3": 9.4}), "R2")

    def test_everyone_picks_the_same_winner_despite_slightly_different_bids(self):
        """Robots hear bids at slightly different moments. A comparison that
        turned on a tenth of a square could let two both think they won."""
        answers = {auction_winner({"R1": 10.0, "R2": 10.0 + d / 10})
                   for d in range(0, 5)}
        self.assertEqual(answers, {"R1"})

    def test_no_bids_means_no_winner(self):
        self.assertIsNone(auction_winner({}))


class TestTheBoard(unittest.TestCase):
    def test_a_new_job_is_open_for_bids(self):
        t = Task("T-1", Cell(1, 1), Cell(2, 2))
        self.assertTrue(t.open_for_bids)
        self.assertFalse(t.finished)

    def test_releasing_puts_it_back_up_for_auction(self):
        t = Task("T-1", Cell(1, 1), Cell(2, 2))
        t.assigned_robot = "R1"
        t.status = TaskStatus.CARRYING
        t.release()
        self.assertIsNone(t.assigned_robot)
        self.assertTrue(t.open_for_bids)
        self.assertEqual(t.reassignments, 1)

    def test_a_board_releases_everything_a_robot_held(self):
        b = TaskBoard()
        for i in range(3):
            t = b.add(Task(f"T-{i}", Cell(1, 1), Cell(2, 2)))
            t.assigned_robot = "R1"
            t.status = TaskStatus.ASSIGNED
        self.assertEqual(len(b.release_all("R1")), 3)
        self.assertEqual(len(b.open_tasks()), 3)


class TestJobsGetDone(unittest.TestCase):
    def test_a_job_is_auctioned_and_delivered(self):
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15), "Wireless Mouse")
        run(w, 60, sc)
        task = w.board.get("T-001")
        self.assertIs(task.status, TaskStatus.DONE)
        self.assertIsNotNone(task.assigned_robot)
        self.assertIsNotNone(task.duration())
        self.assertEqual(w.collisions, 0)

    def test_the_winner_actually_goes_to_the_pickup_first(self):
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15))
        run(w, 2, sc)
        task = w.board.get("T-001")
        winner = w.get(task.assigned_robot)
        self.assertEqual(winner.goal, Cell(7, 4), "went straight to the dropoff")

    def test_only_one_robot_ends_up_doing_a_job(self):
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15))
        run(w, 3, sc)
        holders = [r.robot_id for r in w.robots.values()
                   if r.task is not None and r.task.task_id == "T-001"]
        self.assertLessEqual(len(holders), 1, f"{holders} all took the same job")

    def test_several_jobs_are_shared_out(self):
        w, sc = quiet_world()
        # Collection squares must be OPEN FLOOR next to a shelf, not the shelf
        # itself. (3,6) is inside a shelf block -- an earlier version of this
        # test used it, no robot could reach it, and correctly nobody bid.
        for pickup, drop in ((Cell(7, 4), Cell(13, 15)),
                             (Cell(20, 12), Cell(11, 15)),
                             (Cell(2, 6), Cell(12, 15))):
            self.assertTrue(w.grid.is_walkable(pickup), f"{pickup} is not floor")
            w.announce_task(pickup, drop)
        run(w, 120, sc)
        stats = w.board.stats(w.sim_time)
        self.assertEqual(stats["done"], 3)
        doers = {t.assigned_robot for t in w.board.tasks.values()}
        self.assertGreater(len(doers), 1, "one robot did everything")
        self.assertEqual(w.collisions, 0)

    def test_a_job_nobody_can_reach_gets_no_bids(self):
        w, sc = quiet_world()
        for c in (Cell(1, 0), Cell(0, 1), Cell(1, 1)):
            w.add_obstacle(c)
        for r in w.robots.values():
            r.blocked_map.mark(Cell(1, 0), "test", 0.0)
            r.blocked_map.mark(Cell(0, 1), "test", 0.0)
            r.blocked_map.mark(Cell(1, 1), "test", 0.0)
        w.announce_task(Cell(0, 0), Cell(13, 15))
        run(w, 10, sc)
        self.assertIsNone(w.board.get("T-001").assigned_robot)


class TestSelfHealing(unittest.TestCase):
    """06_TASK_ALLOCATION section 4, and demo step 7."""

    def test_a_failed_robot_stays_failed(self):
        """It did not. Clearing its goal quietly set it back to IDLE, and a
        'failed' robot went on to win an auction and collect a parcel."""
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15))
        run(w, 3, sc)
        victim = next(r for r in w.robots.values() if r.task is not None)
        w.fail_robot(victim.robot_id)
        run(w, 20, sc)
        self.assertIs(w.get(victim.robot_id).status, RobotStatus.FAILED)
        self.assertIsNone(w.get(victim.robot_id).task)

    def test_a_failed_robots_job_is_finished_by_somebody_else(self):
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15))
        run(w, 3, sc)
        victim = next(r for r in w.robots.values() if r.task is not None)
        w.fail_robot(victim.robot_id)
        run(w, 90, sc)
        task = w.board.get("T-001")
        self.assertIs(task.status, TaskStatus.DONE, "the job was lost")
        self.assertNotEqual(task.assigned_robot, victim.robot_id)
        self.assertGreaterEqual(task.reassignments, 1)

    def test_everyone_is_told_the_job_is_free_again(self):
        """The other robots kept it down as the dead robot's job, so nobody
        ever picked it up."""
        w, sc = quiet_world()
        w.announce_task(Cell(7, 4), Cell(13, 15))
        run(w, 3, sc)
        victim = next(r for r in w.robots.values() if r.task is not None)
        w.fail_robot(victim.robot_id)
        run(w, 3, sc)
        for r in w.robots.values():
            if r.robot_id == victim.robot_id:
                continue
            mine = r.board.get("T-001")
            self.assertNotEqual(mine.assigned_robot, victim.robot_id,
                                f"{r.robot_id} still thinks it is the dead robot's")

    def test_a_whole_batch_survives_a_robot_dying(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("orders", seed=2, every=5.0, limit=6)
        killed = False
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
            if not killed and w.sim_time > 8:
                for r in w.robots.values():
                    if r.task is not None:
                        w.fail_robot(r.robot_id)
                        killed = True
                        break
            if w.all_tasks_done():
                break
        self.assertTrue(killed)
        self.assertTrue(w.all_tasks_done(), "jobs were lost when a robot died")
        self.assertEqual(w.collisions, 0)

    def test_a_revived_robot_works_again(self):
        w, sc = quiet_world()
        w.fail_robot("R1")
        w.revive_robot("R1")
        w.announce_task(Cell(2, 8), Cell(13, 15))
        run(w, 60, sc)
        self.assertIs(w.board.get("T-001").status, TaskStatus.DONE)


class TestTheOrderStream(unittest.TestCase):
    """08_SIMULATION section 5: identical workloads. Phase 15 depends on this."""

    def test_the_same_seed_gives_the_same_orders(self):
        runs = []
        for _ in range(2):
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("orders", seed=7, every=4.0, limit=8)
            run(w, 40, sc)
            runs.append([(t.task_id, t.pickup, t.dropoff, t.product, t.priority)
                         for t in w.board.tasks.values()])
        self.assertEqual(runs[0], runs[1])

    def test_different_seeds_give_different_orders(self):
        out = []
        for seed in (1, 2):
            w = phase2_world()
            sc = Scenarios(w)
            sc.apply("orders", seed=seed, every=4.0, limit=8)
            run(w, 40, sc)
            out.append([(t.pickup, t.dropoff) for t in w.board.tasks.values()])
        self.assertNotEqual(out[0], out[1])

    def test_the_limit_is_respected(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("orders", seed=1, every=2.0, limit=5)
        run(w, 60, sc)
        self.assertEqual(len(w.board), 5)

    def test_collections_are_next_to_a_shelf(self):
        w = phase2_world()
        gen = OrderGenerator(w, seed=1)
        for cell in gen.pickups[:20]:
            self.assertTrue(w.grid.is_walkable(cell))
            neighbours = [Cell(cell.x + 1, cell.y), Cell(cell.x - 1, cell.y),
                          Cell(cell.x, cell.y + 1), Cell(cell.x, cell.y - 1)]
            self.assertTrue(any(not w.grid.is_walkable(n) for n in neighbours))


class TestTheMessages(unittest.TestCase):
    def test_task_messages_survive_a_round_trip(self):
        from fleetx_core import TaskAnnounce as TA, TaskBid as TB
        for msg in (TA("ORDERS", 1.0, 1, task_id="T-1", pickup=(7, 4),
                       dropoff=(13, 15), product="Mouse", priority=6),
                    TB("R1", 1.0, 2, task_id="T-1", cost=12.5),
                    TaskClaim("R1", 1.0, 3, task_id="T-1", action="CLAIM", cost=12.5)):
            back = from_dict(json.loads(json.dumps(msg.to_dict())))
            self.assertEqual(type(back), type(msg))
            self.assertEqual(back.task_id, "T-1")


class TestNothingElseBroke(unittest.TestCase):
    def test_a_long_run_of_orders_stays_safe(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("orders", seed=4, every=5.0, limit=25)
        worst = 99.0
        for _ in range(9600):
            sc.keep_busy()
            w.tick(0.05)
            rs = list(w.robots.values())
            for i in range(len(rs)):
                for j in range(i + 1, len(rs)):
                    a, b = rs[i], rs[j]
                    worst = min(worst, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)
            self.assertGreater(worst, TOUCHING,
                               f"robots overlapped at t={w.sim_time:.2f}")
            if w.all_tasks_done():
                break
        self.assertEqual(w.collisions, 0)
        self.assertEqual(w.kpis()["deadlocked"], 0)
        self.assertTrue(w.all_tasks_done(), "not all orders were delivered")

    def test_patrol_still_works(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("patrol")
        run(w, 120, sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(w.kpis()["total_distance"], 700)

    def test_snapshot_still_json_safe(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("orders", seed=1, every=3.0, limit=5)
        run(w, 30, sc)
        json.dumps(w.snapshot())

    def test_reset_clears_the_board(self):
        w = phase2_world()
        sc = Scenarios(w)
        sc.apply("orders", seed=1, every=3.0, limit=5)
        run(w, 20, sc)
        w.reset_counters()
        self.assertEqual(len(w.board), 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
