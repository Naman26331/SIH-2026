"""PART 1 -- demo setups and the "keep the robots busy" loop.

This is simulator scaffolding, NOT robot brain. It lives here on purpose so
shared/fleetx_core/ stays pure. Proper job assignment arrives in Phase 9
(task allocation), and will replace the patrol loop below.
"""

import random
from typing import Dict, List, Optional, Tuple

from fleetx_core import Cell, RobotStatus, World
from fleetx_core.grid import CellKind

# Where each robot wanders when nothing else is going on.
#
# These are chosen to cause trouble, because Phase 2 is meant to SHOW the
# problem. R1 and R2 patrol the same open corridor (row 8) in opposite
# directions, so they meet head-on. R3 patrols straight down column 13, which
# crosses that corridor at (13, 8). Three robots, one junction, no rules.
PATROL: Dict[str, List[Tuple[int, int]]] = {
    "R1": [(26, 8), (2, 8)],
    "R2": [(2, 8), (26, 8)],
    "R3": [(13, 15), (13, 0)],
}


class Scenarios:
    """Sets up the demo situations from 08_SIMULATION_AND_TESTING §4."""

    def __init__(self, world: World):
        self.world = world
        self.auto = True                 # is the patrol loop running?
        self.name = "patrol"
        self._next: Dict[str, int] = {rid: 0 for rid in world.robots}
        self.orders: Optional[OrderGenerator] = None

    # ------------------------------------------------------- the busy loop

    def keep_busy(self) -> None:
        """Feed in orders, or -- if we are just patrolling -- keep robots moving.

        Once real orders are running, the patrol loop stops: robots have proper
        jobs and should not wander off in the middle of one.
        """
        if self.orders is not None:
            note = self.orders.tick(self.world.sim_time)
            if note:
                self.world.decisions.append({
                    "sim_time": round(self.world.sim_time, 2),
                    "robot_id": "ORDERS", "text": note,
                })
                del self.world.decisions[:-20]
            return

        if not self.auto:
            return
        for rid, robot in self.world.robots.items():
            if robot.goal is not None or robot.status is RobotStatus.MOVING:
                continue
            stops = PATROL.get(rid)
            if not stops:
                continue
            i = self._next.get(rid, 0) % len(stops)
            self._next[rid] = i + 1
            x, y = stops[i]
            robot.set_goal(Cell(x, y))

    # --------------------------------------------------------- the setups

    def apply(self, name: str, **kwargs) -> str:
        """Switch to a named setup. Returns a plain-English description."""
        handler = {
            "patrol": self._patrol,
            "head_on": self._head_on,
            "intersection": self._intersection,
            "stop": self._stop,
            "orders": self._orders,
        }.get(name)

        if handler is None:
            return f"There is no scenario called '{name}'."

        self.name = name
        return handler(**kwargs) if kwargs else handler()

    def _orders(self, seed: int = 1, every: float = 6.0,
                limit: Optional[int] = None) -> str:
        """Real work: a stream of orders, robots bidding for each one."""
        self.auto = False
        self.world.reset_counters()
        for robot in self.world.robots.values():
            robot.clear_goal()
        self.orders = OrderGenerator(self.world, seed=seed, every=every, limit=limit)
        return (f"Orders running: one every {every:g}s"
                + (f", {limit} in total" if limit else "") + ".")

    def _patrol(self) -> str:
        self.orders = None
        self.auto = True
        self._next = {rid: 0 for rid in self.world.robots}
        self.world.reset_counters()
        for robot in self.world.robots.values():
            robot.clear_goal()
        return "Free roam: all three robots patrolling. They will meet at (13, 8)."

    def _head_on(self) -> str:
        """08_SIMULATION Scenario 2 -- R1 and R2 driving straight at each other."""
        self.auto = False
        self.orders = None
        self.world.reset_counters()
        self._clear_the_floor()
        self._place("R1", Cell(4, 8), Cell(24, 8))
        self._place("R2", Cell(24, 8), Cell(4, 8))
        self._place("R3", Cell(26, 0), None)          # parked out of the way
        return "Head-on: R1 and R2 sent down the same aisle at each other."

    def _intersection(self) -> str:
        """08_SIMULATION Scenario 3 -- three robots aimed at one junction."""
        self.auto = False
        self.orders = None
        self.world.reset_counters()
        self._clear_the_floor()
        # All three start exactly 8 squares from the junction at (13, 8), so
        # they arrive at the same moment. Get this wrong and one robot sails
        # through before the others turn up, and the demo shows nothing.
        self._place("R1", Cell(5, 8), Cell(24, 8))
        self._place("R2", Cell(21, 8), Cell(2, 8))
        self._place("R3", Cell(13, 0), Cell(13, 15))
        return "Intersection: all three robots converging on the junction at (13, 8)."

    def _stop(self) -> str:
        self.auto = False
        self.orders = None
        for robot in self.world.robots.values():
            robot.clear_goal()
        return "Stopped. All robots parked where they are."

    # -------------------------------------------------------------- helper

    def _clear_the_floor(self) -> None:
        """Park every robot well out of the way before staging a scenario, so
        no staging square is already taken."""
        spare = [Cell(0, 0), Cell(1, 0), Cell(2, 0), Cell(0, 1), Cell(1, 1)]
        for i, robot in enumerate(self.world.robots.values()):
            robot.place(spare[i % len(spare)])

    def _place(self, robot_id: str, at: Cell, goal: Optional[Cell]) -> None:
        """Drop a robot onto a square to set up a demo.

        Real robots cannot teleport; this is staging, not behaviour. Every
        robot in a scenario is given its own square, and this checks that --
        setting up a demo must never start by putting two robots in one place.
        """
        robot = self.world.get(robot_id)
        if robot is None:
            return
        clash = [r.robot_id for r in self.world.robots.values()
                 if r.robot_id != robot_id and r.cell == at]
        if clash:
            raise ValueError(
                f"Cannot place {robot_id} on {at}: {clash[0]} is already there."
            )
        robot.place(at)
        if goal is not None:
            robot.set_goal(goal)


# What the robots are fetching. Names only, so the event feed reads like a
# warehouse rather than a maths exercise.
PRODUCTS = [
    "Wireless Mouse", "Keyboard", "USB Hub", "Webcam", "Headphones",
    "Monitor Stand", "Laptop Sleeve", "Power Bank", "HDMI Cable", "Desk Lamp",
]


class OrderGenerator:
    """Turns a clock into a stream of orders.

    Seeded on purpose. 08_SIMULATION section 5: "Create identical workloads."
    A benchmark comparing FLEET-X against stop-and-wait is worthless unless
    both sides run the SAME jobs in the SAME order, so the same seed must
    always produce the same list.
    """

    def __init__(self, world: World, seed: int = 1, every: float = 6.0,
                 limit: Optional[int] = None):
        self.world = world
        self.every = every
        self.limit = limit
        self.seed = seed
        self._rng = random.Random(seed)
        self._next_at = 0.0
        self.emitted = 0

        grid = world.grid
        # Collect from a square with a shelf next to it -- that is where the
        # goods are. Deliver to a packing station.
        self.pickups = [
            c for c in grid.cells_of_kind(CellKind.FLOOR)
            if any(not grid.is_walkable(n) for n in (
                Cell(c.x + 1, c.y), Cell(c.x - 1, c.y),
                Cell(c.x, c.y + 1), Cell(c.x, c.y - 1)))
        ]
        self.dropoffs = (grid.cells_of_kind(CellKind.DROP)
                         or grid.cells_of_kind(CellKind.PICK))

    def reset(self) -> None:
        self._rng = random.Random(self.seed)
        self._next_at = 0.0
        self.emitted = 0

    def tick(self, now: float) -> Optional[str]:
        """Emit an order if one is due. Returns a line for the event feed."""
        if self.limit is not None and self.emitted >= self.limit:
            return None
        if now < self._next_at:
            return None
        self._next_at = now + self.every
        self.emitted += 1

        pickup = self._rng.choice(self.pickups)
        dropoff = self._rng.choice(self.dropoffs)
        product = self._rng.choice(PRODUCTS)
        priority = self._rng.choice([5, 5, 5, 6, 7])
        task = self.world.announce_task(pickup, dropoff, product, priority)
        return (f"ORDER {task.task_id}: {product} - collect ({pickup.x},{pickup.y}) "
                f"deliver ({dropoff.x},{dropoff.y})")


def random_floor_cell(world: World) -> Cell:
    """A random open square, for the 'send somewhere' button."""
    from fleetx_core.grid import CellKind
    options = world.grid.cells_of_kind(CellKind.FLOOR)
    return random.choice(options)
