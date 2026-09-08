"""The warehouse floor, stored as a grid of squares.

PURE LOGIC ONLY. This file must never import a web library, a ROS 2 library,
or anything that only exists on one machine. Both the grid simulator (Part 1)
and the ROS 2 robot agent (Part 2) import this exact file.

A "cell" is one square of the floor, addressed as (x, y):
  x = column, counted from the left, starting at 0
  y = row,    counted from the top,  starting at 0
"""

from enum import Enum
from typing import Dict, Iterable, List, NamedTuple, Tuple


class Cell(NamedTuple):
    """One square on the warehouse floor."""

    x: int
    y: int


class CellKind(str, Enum):
    """What is standing on a square."""

    FLOOR = "FLOOR"        # open aisle, a robot may drive here
    SHELF = "SHELF"        # storage rack, a robot may NOT drive here
    PICK = "PICK"          # pickup station (goods come in)
    DROP = "DROP"          # packing / drop-off station (orders go out)
    CHARGER = "CHARGER"    # charging bay


# How the ASCII map below is written down.
_ASCII_TO_KIND = {
    ".": CellKind.FLOOR,
    "#": CellKind.SHELF,
    "P": CellKind.PICK,
    "D": CellKind.DROP,
    "C": CellKind.CHARGER,
}

# Anything that is not a shelf can be driven over.
_BLOCKED = {CellKind.SHELF}


# The default FLEET-X warehouse: 28 columns wide, 16 rows tall.
# Twelve shelf blocks, vertical aisles between them, three horizontal
# cross-aisles so a robot always has more than one way around.
#
# The two packing stations (DD) sit in the open band at the bottom, well apart.
# They used to be four bays in a row ON the bottom wall, which made a dead-end
# pocket with a single mouth: at 15+ robots they queued into it and wedged each
# other out, and throughput fell as robots were added. Same four bays here, but
# each one is reachable from four sides and rows 14-15 stay clear underneath as
# a bypass, so a robot standing on a bay blocks nothing.
DEFAULT_WAREHOUSE: Tuple[str, ...] = (
    "............................",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "............................",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "............................",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "...####..####..####..####...",
    "............................",
    "PP...DD.............DD..CC..",
    "PP......................CC..",
    "............................",
)


class Grid:
    """The warehouse map.

    Robots move between neighbouring squares only up/down/left/right --
    never diagonally. That keeps the map a clean graph of nodes and edges,
    which is exactly what the reservation system in Phase 5 will need.
    """

    def __init__(self, rows: Iterable[str]):
        rows = tuple(rows)
        if not rows:
            raise ValueError("The warehouse map has no rows.")

        widths = {len(r) for r in rows}
        if len(widths) != 1:
            raise ValueError(
                f"Every row of the map must be the same width, got widths {sorted(widths)}."
            )

        self.height: int = len(rows)
        self.width: int = widths.pop()
        self._kinds: List[List[CellKind]] = []

        for y, row in enumerate(rows):
            line: List[CellKind] = []
            for x, ch in enumerate(row):
                if ch not in _ASCII_TO_KIND:
                    raise ValueError(
                        f"Unknown map symbol {ch!r} at column {x}, row {y}."
                    )
                line.append(_ASCII_TO_KIND[ch])
            self._kinds.append(line)

    # ---------------------------------------------------------------- basics

    def in_bounds(self, cell: Cell) -> bool:
        """Is this square actually on the map?"""
        return 0 <= cell.x < self.width and 0 <= cell.y < self.height

    def kind(self, cell: Cell) -> CellKind:
        """What is on this square?"""
        return self._kinds[cell.y][cell.x]

    def is_walkable(self, cell: Cell) -> bool:
        """Can a robot stand on this square?"""
        return self.in_bounds(cell) and self.kind(cell) not in _BLOCKED

    def neighbours(self, cell: Cell) -> List[Cell]:
        """The squares a robot can step to from here (up/down/left/right)."""
        candidates = (
            Cell(cell.x, cell.y - 1),
            Cell(cell.x + 1, cell.y),
            Cell(cell.x, cell.y + 1),
            Cell(cell.x - 1, cell.y),
        )
        return [c for c in candidates if self.is_walkable(c)]

    def is_junction(self, cell: Cell) -> bool:
        """Is this a place where aisles cross?

        Three or more ways out means traffic can arrive from several
        directions, which is where the trouble is. These are the squares that
        get booked in Phase 5.
        """
        return len(self.neighbours(cell)) >= 3

    # ------------------------------------------------------------- stations

    def cells_of_kind(self, wanted: CellKind) -> List[Cell]:
        """Every square of a given type, e.g. all the chargers."""
        found: List[Cell] = []
        for y in range(self.height):
            for x in range(self.width):
                if self._kinds[y][x] is wanted:
                    found.append(Cell(x, y))
        return found

    # ---------------------------------------------------------------- output

    def to_dict(self) -> Dict[str, object]:
        """Plain Python data describing the map.

        This is only dicts, lists, ints and strings -- no web code. Whoever
        wants it (the dashboard, a log file, a test) can turn it into JSON.
        """
        return {
            "width": self.width,
            "height": self.height,
            "cells": [[k.value for k in row] for row in self._kinds],
        }


def default_grid() -> Grid:
    """The standard FLEET-X warehouse used by the simulator and the demo."""
    return Grid(DEFAULT_WAREHOUSE)
