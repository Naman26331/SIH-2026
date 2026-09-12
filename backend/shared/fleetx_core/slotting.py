"""PART 1 and PART 2 -- moving fast-selling stock closer to packing.

A real warehouse puts its fastest-moving lines near shipping for the same
reason a supermarket puts milk near the door: every trip for it gets shorter.
This is the same idea, kept small and honest:

  * DemandModel (Phase 12's own module, reused completely unchanged) counts
    which PRODUCT gets ordered, exactly the way it already counts which
    AISLE does -- it is generic over whatever string key you feed it.
  * When one product is clearly, significantly ahead of the others -- the
    same "no idea" gate Phase 12 already uses, so a handful of orders can
    never trigger a move -- and its shelf is not already about as close to
    packing as any shelf gets, that shelf trades PLACES with a shelf that
    already sits close to packing but is not doing much. Same racks, same
    floor squares a robot drives to, same everything except which product
    answers to which shelf name.

What this deliberately does NOT touch: Shelf.cell and Shelf.face never move
-- a rack is bolted to the floor, only the label on it changes. An order
already announced captured its pickup square the instant it was created, so
a robot already on its way is completely unaffected by a later re-slot; only
the NEXT order for that product starts from the new location. Nothing here
decides who fetches a job, plans a route, or books a square.

Pure Python. No web, no ROS 2.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional

from .demand import MIN_ORDERS, DemandModel
from .grid import CellKind, Grid
from .inventory import Inventory, Shelf

# How often to even consider a move. Frequent enough to matter inside a
# demo, rare enough that this reads as a warehouse re-slotting exercise, not
# a shelf twitching back and forth every tick.
RESLOT_EVERY = 20.0

# A move has to shorten the trip by at least this many squares, or it is not
# worth disturbing two shelves for. The same "not worth the drive" idea
# consider_prepositioning already uses, just for a permanent move instead of
# a temporary one.
MIN_GAIN = 3.0

# Within this many squares of the single closest shelf in the whole
# warehouse, a product is already about as well placed as it can be. Nothing
# should move it again just to be tidy.
ALREADY_CLOSE = 2.0


@dataclass
class SlotMove:
    """One re-slotting decision, in a form that can be printed and tested."""

    at: float
    product: str
    from_shelf: str
    to_shelf: str
    from_distance: float
    to_distance: float
    displaced_product: str
    share: float
    sample: int
    lift: float

    def sentence(self) -> str:
        return (f"moved {self.product} from shelf {self.from_shelf} "
                f"({self.from_distance:.0f} squares from packing) to shelf "
                f"{self.to_shelf} ({self.to_distance:.0f} squares) -- "
                f"{self.product} had {self.share * 100:.0f}% of the last "
                f"{self.sample} orders ({self.lift:.1f}x the average)")


class Reslotter:
    """Learns which products sell, and moves the best ones closer to
    packing. Nothing here touches a robot, a route, or a booking -- only
    Shelf.product and Shelf.stock on the two racks it trades."""

    def __init__(self) -> None:
        self.demand = DemandModel()
        self._distance: Dict[str, float] = {}   # shelf name -> squares to nearest bay
        self._last_check = -1e9
        self.moves: List[SlotMove] = []

    def record(self, product: str, now: float) -> None:
        """One order for this product just went out. Learn from it."""
        self.demand.record(product, now)

    def _packing_cells(self, grid: Grid):
        cells = grid.cells_of_kind(CellKind.DROP)
        return cells or grid.cells_of_kind(CellKind.PICK)

    def distance_to_packing(self, shelf: Shelf, grid: Grid) -> float:
        """Squares from this shelf's own face to the nearest packing bay.
        A property of the SLOT, cached once -- it never changes, because the
        rack itself never moves, only what is labelled on it."""
        cached = self._distance.get(shelf.name)
        if cached is not None:
            return cached
        bays = self._packing_cells(grid)
        if not bays:
            return 0.0
        dist = min(abs(shelf.face.x - b.x) + abs(shelf.face.y - b.y) for b in bays)
        self._distance[shelf.name] = dist
        return dist

    def best_shelf_for(self, product: str, inventory: Inventory,
                       grid: Grid) -> Optional[Shelf]:
        """Given a product an order wants, which of its current shelves is
        closest to packing right now?

        This is the other half of what makes re-slotting mean anything.
        Swapping Shelf.product alone changes nothing an order ever sees
        unless something ELSE resolves "I want a Mouse" to wherever the
        best Mouse currently is, rather than to whichever shelf happened to
        be picked first -- the same thing a real picker does: check the
        nearest bin with the SKU you need, not just the one you were
        already walking towards.
        """
        holders = [s for s in inventory.by_name.values() if s.product == product]
        if not holders:
            return None
        return min(holders, key=lambda s: self.distance_to_packing(s, grid))

    def consider(self, inventory: Inventory, grid: Grid,
                 now: float) -> Optional[SlotMove]:
        """Called once a tick; only actually acts every RESLOT_EVERY
        seconds, and only when a move can genuinely help. None the rest of
        the time is the common, correct answer -- "if it can't help, it
        does nothing"."""
        if now - self._last_check < RESLOT_EVERY:
            return None
        self._last_check = now

        pred = self.demand.predict(now)
        if pred is None:
            return None                          # no idea -- do nothing

        product = pred.zone                       # generic model; here the
                                                    # key is a PRODUCT name
        shelves = list(inventory.by_name.values())
        if not shelves:
            return None
        holders = [s for s in shelves if s.product == product]
        if not holders:
            return None

        best_possible = min(self.distance_to_packing(s, grid) for s in shelves)
        worst = max(holders, key=lambda s: self.distance_to_packing(s, grid))
        worst_dist = self.distance_to_packing(worst, grid)
        if worst_dist - best_possible <= ALREADY_CLOSE:
            return None                          # already about as good as it gets

        # Only swap with something CURRENTLY selling less than the product
        # being promoted -- never bump one popular line out just to make
        # room for another.
        my_score = self.demand.score(product, now)
        candidates = [
            s for s in shelves
            if s.name != worst.name and self.demand.score(s.product, now) < my_score
        ]
        if not candidates:
            return None
        target = min(candidates, key=lambda s: self.distance_to_packing(s, grid))
        target_dist = self.distance_to_packing(target, grid)
        if worst_dist - target_dist < MIN_GAIN:
            return None                          # not worth disturbing two shelves for

        displaced = target.product
        worst.product, target.product = target.product, worst.product
        worst.stock, target.stock = target.stock, worst.stock

        move = SlotMove(
            at=now, product=product, from_shelf=worst.name, to_shelf=target.name,
            from_distance=worst_dist, to_distance=target_dist,
            displaced_product=displaced,
            share=pred.share, sample=pred.sample, lift=pred.lift,
        )
        self.moves.append(move)
        del self.moves[:-40]
        return move

    def to_dict(self, now: float) -> Dict[str, object]:
        """Plain data for the dashboard.

        Not a plain pass-through of DemandModel.to_dict() -- its "reason"
        and "zones" fields are worded for an AISLE, and read as nonsense
        ("aisle Monitor has had...") for a product name. Same numbers,
        worded for what this model is actually keyed by here.
        """
        pred = self.demand.predict(now)
        ranked = self.demand.ranked(now)
        total = sum(v for _, v in ranked) or 1.0
        return {
            "orders_seen": self.demand.total,
            "ready": self.demand.total >= MIN_ORDERS,
            "busiest": pred.zone if pred else None,
            "reason": (f"{pred.zone} has had {pred.share * 100:.0f}% of the "
                      f"last {pred.sample} orders ({pred.lift:.1f}x the average)"
                      if pred else ""),
            "products": [{"product": p, "score": round(v, 2),
                         "share": round(v / total, 3)} for p, v in ranked],
            "recent": [
                {"at": round(m.at, 1), "product": m.product,
                 "from_shelf": m.from_shelf, "to_shelf": m.to_shelf,
                 "from_distance": m.from_distance, "to_distance": m.to_distance,
                 "displaced_product": m.displaced_product,
                 "reason": m.sentence()}
                for m in self.moves[-8:][::-1]
            ],
        }
