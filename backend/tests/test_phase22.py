"""Phase 22 tests -- people on the warehouse floor.

Run with:   python3 -m unittest discover -s tests -v

The win condition here is absolute, not a percentage: a robot must NEVER
touch a person. Most of these tests exist to make that claim hold under
exactly the conditions that broke it while this was being built --
a parked robot with nowhere to go, a person who never planned around one in
the first place, real stress with many of both. If any of these ever fails,
the fix is to find the real hole, the same way the first two were found, not
to loosen a number.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Cell, RobotStatus, fleet_world, phase2_world
from fleetx_core.grid import CellKind
from fleetx_core.humans import Human
from scenarios import Scenarios


def run(world, seconds, scenarios=None, dt=0.05):
    for _ in range(int(seconds / dt)):
        if scenarios is not None:
            scenarios.keep_busy()
        world.tick(dt)


def closest_approach(world):
    worst = 999.0
    for r in world.robots.values():
        for h in world.humans.values():
            d = ((r.x - h.x) ** 2 + (r.y - h.y) ** 2) ** 0.5
            worst = min(worst, d)
    return worst


class TestPeopleWalkTheFloor(unittest.TestCase):
    def test_a_person_moves_on_their_own(self):
        w = fleet_world(2)
        w.add_human()
        p = next(iter(w.humans.values()))
        start = (p.x, p.y)
        run(w, 15)
        self.assertNotEqual((p.x, p.y), start, "the person never moved")

    def test_a_person_never_walks_into_a_shelf(self):
        w = fleet_world(2)
        for _ in range(4):
            w.add_human()
        run(w, 60)
        for h in w.humans.values():
            self.assertIs(w.grid.kind(h.cell), CellKind.FLOOR)

    def test_a_person_never_jumps(self):
        """The same teleport invariant robots are held to."""
        w = fleet_world(2)
        w.add_human()
        p = next(iter(w.humans.values()))
        limit = p.speed * 0.05 + 1e-6
        last = (p.x, p.y)
        for _ in range(2000):
            w.tick(0.05)
            moved = abs(p.x - last[0]) + abs(p.y - last[1])
            self.assertLessEqual(moved, limit, f"jumped {moved:.3f} squares in one tick")
            last = (p.x, p.y)

    def test_add_and_remove_person(self):
        w = fleet_world(2)
        self.assertEqual(len(w.humans), 0)
        out = w.add_human()
        self.assertTrue(out["ok"])
        self.assertEqual(len(w.humans), 1)
        out = w.remove_human()
        self.assertTrue(out["ok"])
        self.assertEqual(len(w.humans), 0)

    def test_removing_with_nobody_there_is_refused_politely(self):
        w = fleet_world(2)
        out = w.remove_human()
        self.assertFalse(out["ok"])

    def test_a_full_floor_is_refused_politely(self):
        w = fleet_world(2)
        for _ in range(w.MAX_HUMANS):
            self.assertTrue(w.add_human()["ok"])
        out = w.add_human()
        self.assertFalse(out["ok"])
        self.assertEqual(len(w.humans), w.MAX_HUMANS)


class TestRobotsNeverEnterTheBubble(unittest.TestCase):
    """The absolute requirement. Every test here checks it a different way."""

    def test_ordinary_running_warehouse(self):
        w = fleet_world(8)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(5):
            w.add_human()
        run(w, 150, sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)

    def test_a_person_standing_still_where_a_robot_is_parked(self):
        """The bug that broke this while it was being built: an idle robot
        with nowhere to go never used to check anything at all, and nothing
        stopped a person's own walking route from aiming straight at one."""
        w = fleet_world(3)
        r = list(w.robots.values())[0]
        r.place(Cell(10, 8))
        w.add_human()
        p = next(iter(w.humans.values()))
        ok = p.set_goal(w.grid, Cell(14, 8))
        self.assertTrue(ok)
        for _ in range(400):
            w.tick(0.05)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)

    def test_a_moving_robot_approaching_a_still_person(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.place(Cell(2, 8))
        r.set_goal(Cell(24, 8))
        w.add_human()
        p = next(iter(w.humans.values()))
        p.cell = Cell(13, 8)
        p.x, p.y = 13.0, 8.0
        p.path = []                     # deliberately does not move itself
        for _ in range(1000):
            w.tick(0.05)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)

    def test_stress_many_robots_many_people(self):
        w = fleet_world(15)
        sc = Scenarios(w)
        sc.apply("orders", every=1.5)
        for _ in range(6):
            w.add_human()
        run(w, 150, sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)

    def test_stress_stop_and_wait_too(self):
        """The hard stop lives in the shared reflex both fleets use. It must
        protect the dumb baseline exactly as well as it protects FLEET-X --
        taking it away from the baseline would not make the comparison fair,
        it would make the baseline crash into people."""
        from fleetx_core import World
        w = World(coordination="STOP_AND_WAIT")
        for i in range(8):
            from fleetx_core import Robot
            w.add_robot(Robot(robot_id=f"R{i+1}", cell=Cell(2 + i, 0)))
        sc = Scenarios(w)
        sc.apply("orders", every=1.5)
        for _ in range(5):
            w.add_human()
        run(w, 150, sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)

    def test_repeated_add_and_remove_of_both_kinds(self):
        w = fleet_world(6)
        sc = Scenarios(w)
        sc.apply("orders", every=1.5)
        for cycle in range(5):
            run(w, 15, sc)
            w.add_human()
            if len(w.robots) > 3:
                w.remove_robot()
            w.add_robot_live()
            if len(w.humans) > 4:
                w.remove_human()
        run(w, 60, sc)
        self.assertEqual(w.collisions, 0)

    def test_a_network_blackout_around_people(self):
        """Local sensing, not the radio, is what protects a person -- it must
        keep working exactly when the network stops."""
        w = fleet_world(6)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(4):
            w.add_human()
        run(w, 20, sc)
        w.bus.packet_loss = 1.0
        run(w, 100, sc)
        self.assertEqual(w.collisions, 0)
        self.assertGreater(closest_approach(w), 0.7)


class TestRobotsSlowDownNearPeople(unittest.TestCase):
    def test_travel_speed_drops_near_a_person(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.place(Cell(10, 8))
        full = r.travel_speed()
        r.sense_humans([("P1", 10.8, 8.0, Cell(11, 8))], now=0.0)
        eased = r.travel_speed()
        self.assertLess(eased, full)
        self.assertGreater(eased, 0.0, "slowing must not be indistinguishable from stopping")

    def test_far_from_anyone_speed_is_unaffected(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        full = r.travel_speed()
        r.sense_humans([("P1", 25.0, 14.0, Cell(25, 14))], now=0.0)
        self.assertEqual(r.travel_speed(), full)

    def test_the_dashboard_can_say_who_it_is_slowing_for(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.sense_humans([("P1", r.x + 0.5, r.y, Cell(int(r.x) + 1, int(r.y)))], now=0.0)
        r.travel_speed()
        self.assertEqual(r.near_person, "P1")
        self.assertEqual(r.to_dict(0.0)["near_person"], "P1")


class TestRoutingAroundAPerson(unittest.TestCase):
    def test_a_stationary_person_gets_routed_around(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.place(Cell(2, 8))
        r.set_goal(Cell(24, 8))
        for _ in range(20):
            w.tick(0.05)

        w.add_human()
        p = next(iter(w.humans.values()))
        ahead = Cell(int(r.x) + 3, 8)
        p.cell = ahead
        p.x, p.y = float(ahead.x), float(ahead.y)
        p.path = []
        original_path = list(r.path)
        self.assertIn(ahead, original_path, "test needs the person ON the route")

        for _ in range(400):
            w.tick(0.05)

        self.assertGreater(r.reroutes, 0, "the robot never rerouted")
        self.assertEqual(w.collisions, 0)

    def test_a_persons_cell_is_locally_marked_blocked(self):
        """The actual mechanism: sensing a person marks their square (and
        the four next to it) the same way sensing a dropped box does --
        reused logic, not new logic."""
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.sense_humans([("P1", 10.0, 8.0, Cell(10, 8))], now=0.0)
        for cell in (Cell(10, 8), Cell(11, 8), Cell(9, 8), Cell(10, 9), Cell(10, 7)):
            self.assertTrue(r.blocked_map.is_blocked(cell, 0.0), cell)
        self.assertFalse(r.blocked_map.is_blocked(Cell(13, 8), 0.0))

    def test_the_block_fades_once_the_person_is_out_of_range(self):
        w = fleet_world(1)
        r = list(w.robots.values())[0]
        r.sense_humans([("P1", 10.0, 8.0, Cell(10, 8))], now=0.0)
        self.assertTrue(r.blocked_map.is_blocked(Cell(10, 8), 0.0))
        self.assertFalse(r.blocked_map.is_blocked(Cell(10, 8), 5.0))

    def test_a_person_never_broadcasts_as_a_fleet_wide_obstacle(self):
        """Sensing a person is purely local -- it must never go out over the
        radio the way a genuinely dropped box does, or the whole fleet would
        avoid a square someone merely walked past once."""
        w = fleet_world(2)
        r = list(w.robots.values())[0]
        before = r.messages_sent
        r.sense_humans([("P1", 10.0, 8.0, Cell(10, 8))], now=0.0)
        self.assertEqual(r.messages_sent, before)


class TestHumansOwnGoodBehaviour(unittest.TestCase):
    def test_a_person_pauses_rather_than_close_on_a_robot(self):
        w = fleet_world(1)
        h = Human(human_id="P1", cell=Cell(5, 8))
        h.set_goal(w.grid, Cell(9, 8))
        before = (h.x, h.y)
        h.advance(0.05, robots=[(6.0, 8.0)])       # a robot right in their way
        self.assertEqual((h.x, h.y), before, "walked closer to a nearby robot")

    def test_with_nobody_around_a_person_walks_normally(self):
        w = fleet_world(1)
        h = Human(human_id="P1", cell=Cell(5, 8))
        h.set_goal(w.grid, Cell(9, 8))
        before = (h.x, h.y)
        h.advance(0.05, robots=[(25.0, 14.0)])
        self.assertNotEqual((h.x, h.y), before)


if __name__ == "__main__":
    unittest.main()
