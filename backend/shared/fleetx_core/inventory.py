"""PART 1 and PART 2 -- what is actually on the shelves.

Orders must read the way a human reads them. Up to now an
order said "collect from (7, 5)", which is a coordinate, not a job. A picker in
a real warehouse is told "Mouse x2 from shelf A14" and the number on the rack
is how they find it.

This is a thin layer on purpose -- "inventory-lite". It gives every shelf a
name and something to hold, and it can turn a shelf name back into the floor
square a robot must stand on to reach it. It makes no decisions, so it changes
nothing about how robots drive or who gives way.

Pure Python, like everything else in here. No web, no ROS 2.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

from .grid import Cell, CellKind, Grid

# What a warehouse like this one would actually hold. Small, boring, real.
CATALOGUE: Sequence[str] = (
    "Mouse", "Keyboard", "Monitor", "Laptop", "Headset", "Webcam",
    "Router", "Cable", "Charger", "Docking station", "Printer", "Speaker",
    "Hard drive", "Memory stick", "Tablet", "Phone case",
)

AISLE_LETTERS = "ABCDEFGH"


@dataclass
class Shelf:
    """One shelf square, with a name a person can read and something on it."""

    name: str                 # "A14"
    cell: Cell                # the rack itself -- a robot CANNOT drive here
    face: Cell                # the floor square you stand on to reach it
    product: str
    stock: int

    def label(self, quantity: int = 1) -> str:
        """The order line, e.g. "Mouse x2 from shelf A14"."""
        return f"{self.product} x{quantity} from shelf {self.name}"


class Inventory:
    """Names every rack in a warehouse and says what is on it.

    Shelves are named the way aisles are signed: a letter for the aisle running
    top to bottom, then a number counting along it. Aisle A is the leftmost
    block of racks, B the next, and so on -- so A14 is always in the same place
    and a judge can find it on the map without being told.
    """

    def __init__(self, grid: Grid, seed: int = 7) -> None:
        import random

        self.grid = grid
        self._rng = random.Random(seed)
        self.by_name: Dict[str, Shelf] = {}
        self.by_cell: Dict[Cell, Shelf] = {}

        # Group the racks into vertical blocks -- that is what an aisle is.
        racks = grid.cells_of_kind(CellKind.SHELF)
        columns = sorted({c.x for c in racks})
        blocks: List[List[int]] = []
        for x in columns:
            if blocks and x == blocks[-1][-1] + 1:
                blocks[-1].append(x)
            else:
                blocks.append([x])

        for i, block in enumerate(blocks):
            letter = AISLE_LETTERS[i % len(AISLE_LETTERS)]
            in_block = sorted((c for c in racks if c.x in block),
                              key=lambda c: (c.y, c.x))
            number = 0
            for cell in in_block:
                face = self._face_of(cell)
                if face is None:
                    continue          # a rack nothing can reach holds nothing
                number += 1
                name = f"{letter}{number}"
                shelf = Shelf(name=name, cell=cell, face=face,
                              product=self._rng.choice(CATALOGUE),
                              stock=self._rng.randint(4, 40))
                self.by_name[name] = shelf
                self.by_cell[cell] = shelf

    def _face_of(self, rack: Cell) -> Optional[Cell]:
        """The floor square a robot stands on to reach this rack.

        Left and right first: the aisles here run top to bottom, so that is
        where a picker would stand.
        """
        for side in (Cell(rack.x - 1, rack.y), Cell(rack.x + 1, rack.y),
                     Cell(rack.x, rack.y - 1), Cell(rack.x, rack.y + 1)):
            if self.grid.is_walkable(side):
                return side
        return None

    # ------------------------------------------------------------- lookups

    def shelf(self, name: str) -> Optional[Shelf]:
        return self.by_name.get(name)

    def shelf_at(self, rack: Cell) -> Optional[Shelf]:
        return self.by_cell.get(rack)

    def face_of(self, name: str) -> Optional[Cell]:
        """Shelf name -> the square the robot actually drives to.

        This is the whole point of the layer: the order says A14, the robot
        still goes to exactly the square it always went to.
        """
        shelf = self.by_name.get(name)
        return shelf.face if shelf else None

    def shelves_facing(self, face: Cell) -> List[Shelf]:
        """Every rack you can reach from this floor square."""
        return [s for s in self.by_name.values() if s.face == face]

    def pick_shelf(self, rng) -> Shelf:
        """A shelf with something on it, for a new order."""
        stocked = [s for s in self.by_name.values() if s.stock > 0]
        return rng.choice(stocked or list(self.by_name.values()))

    def take(self, name: str, quantity: int) -> int:
        """Remove stock. Returns how many actually came off the shelf.

        Stock can run out, and an order for an empty shelf would be a lie. It
        is restocked rather than allowed to go negative -- goods-in is not
        modelled, and a warehouse that empties itself makes a worse demo than
        one that does not.
        """
        shelf = self.by_name.get(name)
        if shelf is None:
            return 0
        taken = min(shelf.stock, quantity)
        shelf.stock -= taken
        if shelf.stock <= 0:
            shelf.stock = self._rng.randint(4, 40)      # restocked overnight
        return taken

    def to_dict(self) -> Dict[str, object]:
        """Plain data for the dashboard. Dicts and numbers only."""
        return {
            "shelves": [
                {"name": s.name, "cell": [s.cell.x, s.cell.y],
                 "face": [s.face.x, s.face.y],
                 "product": s.product, "stock": s.stock}
                for s in self.by_name.values()
            ]
        }
