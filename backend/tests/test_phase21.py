"""Phase 21 tests -- moving inventory.

Run with:   python3 -m unittest discover -s tests -v

The claim being tested: the warehouse learns which products actually sell,
and moves the best ones closer to packing -- and when it can't help, it
does nothing, rather than guessing. shared/fleetx_core/slotting.py is the
model and the decision; grid_sim/scenarios.py's OrderGenerator is the other
half that makes a moved shelf mean a shorter TRIP, not just a relabelled
rack nobody's order ever visits.

Off by default everywhere except the live dashboard, because turning it on
changes which square an order actually collects from -- every test file
written before this phase must see EXACTLY the behaviour it always has
unless it explicitly opts in.
"""

import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import Reslotter, fleet_world, phase2_world
from scenarios import Scenarios


def run(w, sc, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        sc.keep_busy()
        w.tick(dt)


class TestOffByDefault(unittest.TestCase):
    """The opt-in flag itself. Every OTHER test file in this suite depends
    on this being true without ever having to say so."""

    def test_reslotting_is_off_by_default(self):
        self.assertFalse(fleet_world(3).reslotting_enabled)

    def test_off_means_off_no_learning_no_moves_no_redirect(self):
        w = fleet_world(4)
        sc = Scenarios(w)
        sc.apply("orders", seed=1, every=1.0, limit=None)
        run(w, sc, 200)
        self.assertEqual(w.reslotter.demand.total, 0,
                         "the model learned something while switched off")
        self.assertEqual(len(w.reslotter.moves), 0)
        self.assertEqual(w.collisions, 0)
        # Every order still collects from exactly the shelf it names --
        # the Phase 11 invariant, unaffected by this phase being off.
        checked = 0
        for task in w.board.tasks.values():
            shelf = w.inventory.shelf(task.shelf)
            self.assertEqual(task.pickup, shelf.face)
            checked += 1
        self.assertGreater(checked, 5)


def _worst_per_product(shelves, distance_fn):
    """product -> the distance of its OWN farthest-from-packing shelf.

    A product usually sits on several shelves (16 products, many more
    shelves than that) -- consider() moves the WORST holder of a product,
    not just whichever shelf a test happened to look at first, so a
    fixture has to reason about products the same way to mean anything.
    """
    by_product = {}
    for s in shelves:
        by_product.setdefault(s.product, []).append(s)
    return {p: max(distance_fn(s) for s in ss) for p, ss in by_product.items()}


class TestTheModelItself(unittest.TestCase):
    """Reslotter in isolation -- no robots, no world, just the decision."""

    def test_no_idea_means_no_move(self):
        w = phase2_world()
        r = Reslotter()
        for _ in range(3):                      # far too few to have an opinion
            r.record("Mouse", 0.0)
        self.assertIsNone(r.consider(w.inventory, w.grid, 100.0))

    def test_a_clear_winner_moves_if_it_is_not_already_close(self):
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        best_possible = min(r.distance_to_packing(s, w.grid) for s in shelves)
        worst = _worst_per_product(shelves, lambda s: r.distance_to_packing(s, w.grid))
        # A product whose OWN worst-placed shelf is genuinely far, so the
        # move has something real to do.
        product = max(worst, key=worst.get)
        self.assertGreater(worst[product] - best_possible, 3.0,
                           "test grid has no poorly-placed product -- adjust fixture")

        # DemandModel's lift is "how far above the AVERAGE zone" -- with
        # only one product ever recorded it trivially ties the average
        # (lift == 1.0) and never looks interesting. A little background
        # noise for something else is what gives the winner something to
        # actually be ahead OF.
        other = next(p for p in worst if p != product)
        for i in range(40):
            r.record(product, float(i))
            if i % 6 == 0:
                r.record(other, float(i))
        move = r.consider(w.inventory, w.grid, 100.0)
        self.assertIsNotNone(move, "a clear, poorly-placed winner did not move")
        self.assertEqual(move.product, product)
        self.assertAlmostEqual(move.from_distance, worst[product])
        self.assertLess(move.to_distance, move.from_distance)
        self.assertGreaterEqual(move.from_distance - move.to_distance, 3.0)
        # The swap actually happened, on the real Shelf objects.
        self.assertEqual(w.inventory.shelf(move.to_shelf).product, product)
        self.assertNotEqual(w.inventory.shelf(move.from_shelf).product, product)

    def test_already_close_does_nothing(self):
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        closest = min(shelves, key=lambda s: r.distance_to_packing(s, w.grid))
        # Inventory's random assignment does not guarantee a product with
        # EVERY instance already well placed -- so build the case directly:
        # make the closest shelf the ONLY holder of its product, so "already
        # close" is unambiguous rather than hoped-for.
        for s in shelves:
            if s is not closest and s.product == closest.product:
                s.product = "Filler product for this test"
        product = closest.product
        other = next(s.product for s in shelves if s.product != product)
        for i in range(40):
            r.record(product, float(i))
            if i % 6 == 0:
                r.record(other, float(i))
        move = r.consider(w.inventory, w.grid, 100.0)
        self.assertIsNone(move, "moved a product whose shelf was already "
                                "about as close to packing as any shelf gets")

    def test_a_cooldown_prevents_checking_every_tick(self):
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        worst = _worst_per_product(shelves, lambda s: r.distance_to_packing(s, w.grid))
        product = max(worst, key=worst.get)
        other = next(p for p in worst if p != product)
        for i in range(40):
            r.record(product, float(i))
            if i % 6 == 0:
                r.record(other, float(i))
        first = r.consider(w.inventory, w.grid, 100.0)
        self.assertIsNotNone(first)
        again = r.consider(w.inventory, w.grid, 100.05)   # one tick later
        self.assertIsNone(again, "acted again before RESLOT_EVERY had passed")

    def test_never_bumps_a_more_popular_line_out_of_a_good_spot(self):
        """The swap target must be selling LESS than the product being
        promoted -- otherwise this would just be shuffling two winners."""
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        worst = _worst_per_product(shelves, lambda s: r.distance_to_packing(s, w.grid))
        product = max(worst, key=worst.get)
        best_possible_shelf = min(shelves, key=lambda s: r.distance_to_packing(s, w.grid))
        # Make the CURRENT closest shelf's occupant look just as popular.
        for i in range(40):
            r.record(product, float(i))
            r.record(best_possible_shelf.product, float(i))
        move = r.consider(w.inventory, w.grid, 100.0)
        if move is not None:
            self.assertNotEqual(move.to_shelf, best_possible_shelf.name,
                               "swapped with an equally popular line's shelf")

    def test_sentence_reads_like_a_product_not_an_aisle(self):
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        worst = _worst_per_product(shelves, lambda s: r.distance_to_packing(s, w.grid))
        product = max(worst, key=worst.get)
        other = next(p for p in worst if p != product)
        for i in range(40):
            r.record(product, float(i))
            if i % 6 == 0:
                r.record(other, float(i))
        move = r.consider(w.inventory, w.grid, 100.0)
        self.assertIsNotNone(move)
        self.assertNotIn("aisle", move.sentence())
        self.assertIn(product, move.sentence())
        self.assertIn("squares from packing", move.sentence())

    def test_to_dict_is_worded_for_products(self):
        w = phase2_world()
        r = Reslotter()
        shelves = list(w.inventory.by_name.values())
        far = max(shelves, key=lambda s: r.distance_to_packing(s, w.grid))
        for i in range(40):
            r.record(far.product, float(i))
        d = r.to_dict(50.0)
        self.assertTrue(d["ready"])
        self.assertNotIn("aisle", d["reason"])
        self.assertIn("product", d["products"][0])


class TestWiredIntoTheWorld(unittest.TestCase):
    """The world calling this every tick, gated the same way everything
    else opt-in in this project is."""

    def test_a_move_shows_up_in_the_decisions_feed(self):
        # Checked the instant a move happens, not after -- self.decisions is
        # a small rolling window shared with every other kind of decision
        # in the fleet, and a long run's ordinary bidding/negotiation
        # traffic can push an old entry out long before a test gets to look.
        w = fleet_world(5)
        w.reslotting_enabled = True
        sc = Scenarios(w)
        sc.apply("orders", seed=2, every=0.8, limit=None)
        found = False
        for _ in range(int(300 / 0.05)):
            sc.keep_busy()
            w.tick(0.05)
            if w.reslotter.moves:
                found = True
                break
        self.assertTrue(found, "no move happened in 300s of brisk ordering -- "
                               "check the fixture still has a poorly-placed shelf")
        texts = [d["text"] for d in w.decisions if d["robot_id"] == "INVENTORY"]
        self.assertTrue(any(t.startswith("moved ") for t in texts))
        self.assertEqual(w.collisions, 0)

    def test_snapshot_carries_reslotting_data(self):
        w = fleet_world(3)
        w.reslotting_enabled = True
        sc = Scenarios(w)
        sc.apply("orders", seed=2, every=0.8, limit=None)
        run(w, sc, 60)
        snap = w.snapshot()
        self.assertIn("reslotting", snap)
        self.assertIn("recent", snap["reslotting"])

    def test_redirected_pickup_still_matches_the_shelf_it_names(self):
        """The Phase 11 invariant, now under redirect: whatever shelf an
        order ends up naming, task.pickup must still be exactly that
        shelf's face -- never the one the zone pattern picked first."""
        w = fleet_world(5)
        w.reslotting_enabled = True
        sc = Scenarios(w)
        sc.apply("orders", seed=2, every=0.8, limit=None)
        run(w, sc, 300)
        checked = 0
        for task in w.board.tasks.values():
            shelf = w.inventory.shelf(task.shelf)
            self.assertIsNotNone(shelf)
            self.assertEqual(task.pickup, shelf.face,
                             f"{task.task_id} says {task.shelf} but collects "
                             f"from {task.pickup}, not {shelf.face}")
            checked += 1
        self.assertGreater(checked, 20)

    def test_no_collisions_under_load_with_reslotting_on(self):
        w = fleet_world(8)
        w.reslotting_enabled = True
        sc = Scenarios(w)
        sc.apply("orders", seed=3, every=0.6, limit=None)
        run(w, sc, 300)
        self.assertEqual(w.collisions, 0)


class TestTripsGetShorter(unittest.TestCase):
    """The actual point: the same orders, the same seed, only the flag
    different, and less driving per delivered order because of it."""

    def test_reslotting_reduces_squares_driven_per_order(self):
        def once(enabled):
            w = fleet_world(5)
            w.reslotting_enabled = enabled
            sc = Scenarios(w)
            sc.apply("orders", seed=3, every=1.0, limit=None)
            run(w, sc, 400)
            done = w.board.stats(w.sim_time)["done"]
            distance = sum(r.distance for r in w.robots.values())
            return done, distance

        off_done, off_dist = once(False)
        on_done, on_dist = once(True)
        self.assertGreater(off_done, 0)
        self.assertGreater(on_done, 0)
        off_per_order = off_dist / off_done
        on_per_order = on_dist / on_done
        self.assertLess(on_per_order, off_per_order,
                        f"reslotting on ({on_per_order:.2f} sq/order) was not "
                        f"better than off ({off_per_order:.2f} sq/order)")


if __name__ == "__main__":
    unittest.main()
