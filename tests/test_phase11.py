"""Phase 11 tests -- inventory-lite: shelves with names and stock.

Run with:   python3 -m unittest discover -s tests -v

The point of this phase is readability, so most of these check that the words
and the squares agree. The one that really matters is that naming a shelf
changed nothing about where a robot actually drives: if "shelf A14" ever sends
a robot to a different square than (7, 5) did, the layer is lying.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "grid_sim"))

from fleetx_core import Cell, Inventory, World, fleet_world, phase2_world
from fleetx_core.grid import CellKind
from fleetx_core.tasks import Task
from scenarios import Scenarios


class TestEveryRackHasAName(unittest.TestCase):
    def setUp(self):
        self.w = phase2_world()
        self.inv = self.w.inventory

    def test_names_are_unique(self):
        names = list(self.inv.by_name)
        self.assertEqual(len(names), len(set(names)))

    def test_names_look_like_warehouse_signs(self):
        for name in self.inv.by_name:
            self.assertRegex(name, r"^[A-H]\d+$", f"{name} is not an aisle code")

    def test_a_rack_you_cannot_reach_is_not_offered(self):
        """The middle of a four-wide block has shelves on all four sides. No
        picker can reach it, so it must never appear in an order."""
        for shelf in self.inv.by_name.values():
            self.assertTrue(self.w.grid.is_walkable(shelf.face),
                            f"{shelf.name} says stand on {shelf.face}, which is not floor")

    def test_the_face_really_is_next_to_the_rack(self):
        for shelf in self.inv.by_name.values():
            step = abs(shelf.face.x - shelf.cell.x) + abs(shelf.face.y - shelf.cell.y)
            self.assertEqual(step, 1,
                             f"{shelf.name}: {shelf.face} is not beside {shelf.cell}")

    def test_every_named_rack_is_actually_a_shelf(self):
        for shelf in self.inv.by_name.values():
            self.assertIs(self.w.grid.kind(shelf.cell), CellKind.SHELF)

    def test_the_same_map_always_gives_the_same_names(self):
        """A judge should be able to point at A14 twice and get A14 twice."""
        a = Inventory(phase2_world().grid)
        b = Inventory(phase2_world().grid)
        self.assertEqual(
            {n: (s.cell, s.product) for n, s in a.by_name.items()},
            {n: (s.cell, s.product) for n, s in b.by_name.items()})


class TestOrdersReadLikeOrders(unittest.TestCase):
    def test_the_line_is_words_not_coordinates(self):
        t = Task(task_id="T-001", pickup=Cell(7, 5), dropoff=Cell(5, 13),
                 product="Mouse", shelf="A14", quantity=2)
        self.assertEqual(t.line(), "Mouse x2 from shelf A14")

    def test_a_task_with_no_shelf_still_says_something(self):
        """Every test written before this phase builds tasks without a shelf.
        They must not start printing an empty string."""
        self.assertEqual(
            Task(task_id="T", pickup=Cell(7, 5), dropoff=Cell(5, 13),
                 product="Mouse").line(),
            "Mouse from (7, 5)")
        self.assertEqual(
            Task(task_id="T", pickup=Cell(7, 5), dropoff=Cell(5, 13)).line(),
            "(7, 5)")

    def test_real_orders_name_a_real_shelf(self):
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(400):
            sc.keep_busy()
            w.tick(0.05)
        self.assertTrue(w.board.tasks, "no orders were made")
        for task in w.board.tasks.values():
            self.assertTrue(task.shelf, f"{task.task_id} has no shelf name")
            self.assertIn(task.shelf, w.inventory.by_name)
            self.assertGreaterEqual(task.quantity, 1)
            self.assertIn("from shelf", task.line())


class TestTheNameMatchesTheSquare(unittest.TestCase):
    """The only test here that could hide a real bug."""

    def test_the_robot_drives_to_the_square_the_shelf_names(self):
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(800):
            sc.keep_busy()
            w.tick(0.05)
        checked = 0
        for task in w.board.tasks.values():
            shelf = w.inventory.shelf(task.shelf)
            self.assertIsNotNone(shelf, f"{task.shelf} is not a real shelf")
            self.assertEqual(task.pickup, shelf.face,
                             f"{task.task_id} says {task.shelf} but collects "
                             f"from {task.pickup}, not {shelf.face}")
            checked += 1
        self.assertGreater(checked, 5)

    def test_looking_a_shelf_up_by_name_gives_the_same_square(self):
        w = phase2_world()
        for name, shelf in w.inventory.by_name.items():
            self.assertEqual(w.inventory.face_of(name), shelf.face)

    def test_collecting_still_happens_on_that_square(self):
        """Follow a job all the way through: whatever the order calls it, the
        parcel must be picked up standing on the shelf's own face square."""
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        picked = []
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
            for r in w.robots.values():
                if r.task is not None and r.task.picked_at is not None:
                    t = r.task
                    if t.task_id not in [p[0] for p in picked]:
                        picked.append((t.task_id, t.shelf, r.cell, t.pickup))
        self.assertTrue(picked, "nothing was ever collected")
        for task_id, shelf_name, where, pickup in picked:
            shelf = w.inventory.shelf(shelf_name)
            self.assertEqual(where, shelf.face,
                             f"{task_id} for {shelf_name} was collected at "
                             f"{where}, but that shelf is reached from {shelf.face}")


class TestStock(unittest.TestCase):
    def test_taking_stock_reduces_it(self):
        w = phase2_world()
        shelf = w.inventory.shelf("A1")
        before = shelf.stock
        took = w.inventory.take("A1", 2)
        self.assertEqual(took, min(2, before))
        if before > 2:
            self.assertEqual(shelf.stock, before - 2)

    def test_a_shelf_never_goes_negative(self):
        w = phase2_world()
        for _ in range(200):
            w.inventory.take("A1", 5)
            self.assertGreater(w.inventory.shelf("A1").stock, 0)

    def test_orders_keep_coming_after_a_long_run(self):
        """If stock ran out and was never replaced, orders would dry up."""
        w = fleet_world(5)
        sc = Scenarios(w)
        sc.apply("orders", every=1.0)
        for _ in range(6000):
            sc.keep_busy()
            w.tick(0.05)
        self.assertGreater(sc.orders.emitted, 250)


class TestNothingAboutDrivingChanged(unittest.TestCase):
    def test_naming_the_shelves_did_not_cause_a_collision(self):
        w = fleet_world(15)
        sc = Scenarios(w)
        sc.apply("orders", every=3.0)
        for _ in range(4000):
            sc.keep_busy()
            w.tick(0.05)
        self.assertEqual(w.collisions, 0)

    def test_orders_still_collect_from_beside_a_real_shelf(self):
        w = fleet_world(3)
        sc = Scenarios(w)
        sc.apply("orders", every=2.0)
        for _ in range(400):
            sc.keep_busy()
            w.tick(0.05)
        g = w.grid
        for task in w.board.tasks.values():
            touching = [n for n in (Cell(task.pickup.x + 1, task.pickup.y),
                                    Cell(task.pickup.x - 1, task.pickup.y),
                                    Cell(task.pickup.x, task.pickup.y + 1),
                                    Cell(task.pickup.x, task.pickup.y - 1))
                        if g.in_bounds(n) and g.kind(n) is CellKind.SHELF]
            self.assertTrue(touching)


if __name__ == "__main__":
    unittest.main()
