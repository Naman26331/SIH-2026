"""PART 1 -- demo setups and the "keep the robots busy" loop.

This is simulator scaffolding, NOT robot brain. It lives here on purpose so
shared/fleetx_core/ stays pure. Proper job assignment arrives in Phase 9
(task allocation), and will replace the patrol loop below.
"""

import random
from typing import Dict, List, Optional, Tuple

from fleetx_core import Cell, RobotStatus, Task, TaskClaim, TaskStatus, World
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

# Squares the Head-on and Intersection demos put robots down on. Nothing may
# be parked here, or the demo cannot stage itself.
STAGING_CELLS = frozenset([
    Cell(4, 8), Cell(24, 8), Cell(26, 0),          # head-on
    Cell(5, 8), Cell(21, 8), Cell(13, 0),          # intersection
])


class Scenarios:
    """Sets up the demo situations for the dashboard."""

    def __init__(self, world: World):
        self.world = world
        self.auto = True                 # is the patrol loop running?
        self.name = "patrol"
        self._next: Dict[str, int] = {rid: 0 for rid in world.robots}
        self.orders: Optional[OrderGenerator] = None
        self._routes: Dict[str, List[Tuple[int, int]]] = self._patrol_routes()

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
            # Never hijack a robot that is on its way to charge or sitting on a
            # bay. This loop hands a new waypoint to anything briefly without a
            # destination, and it sent a robot on 2.7% off across the warehouse
            # instead of to the charger it had just booked.
            if robot.charger is not None or robot.status is RobotStatus.CHARGING:
                continue
            if robot.goal is not None or robot.status is RobotStatus.MOVING:
                continue
            stops = self._routes.get(rid)
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
            "deadlock": self._deadlock,
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
        self.world.resume_all()
        self.world.reset_counters()
        for robot in self.world.robots.values():
            robot.clear_goal()
        self.orders = OrderGenerator(self.world, seed=seed, every=every, limit=limit)
        return (f"Orders running: one every {every:g}s"
                + (f", {limit} in total" if limit else "") + ".")

    def _patrol(self) -> str:
        self.orders = None
        self.auto = True
        self.world.resume_all()
        self._next = {rid: 0 for rid in self.world.robots}
        self._routes = self._patrol_routes()
        self.world.reset_counters()
        for robot in self.world.robots.values():
            robot.clear_goal()
        n = len(self.world.robots)
        return (f"Free roam: all {n} robots patrolling. "
                "R1, R2 and R3 will meet at (13, 8).")

    def _patrol_routes(self) -> Dict[str, List[Tuple[int, int]]]:
        """A there-and-back route for every robot on the floor.

        R1, R2 and R3 keep the hand-picked routes that make them collide at
        (13, 8) -- that is the whole point of the demo. Everyone else used to
        get nothing and simply stood still, so Free roam with twenty robots
        moved three of them. Now the rest get their own long routes, picked the
        same way every time so the demo is repeatable.
        """
        routes: Dict[str, List[Tuple[int, int]]] = dict(PATROL)
        spare = [c for c in self.world.grid.cells_of_kind(CellKind.FLOOR)
                 if c not in STAGING_CELLS]
        rng = random.Random(4)
        rng.shuffle(spare)
        pool = iter(spare)
        for rid in self.world.robots:
            if rid in routes:
                continue
            pair = [next(pool, None), next(pool, None)]
            if None in pair:
                break
            routes[rid] = [(c.x, c.y) for c in pair]
        return routes

    def _head_on(self) -> str:
        """R1 and R2 driving straight at each other down one aisle."""
        self.auto = False
        self.orders = None
        self.world.resume_all()
        self.world.reset_counters()
        self._clear_the_floor()
        self._place("R1", Cell(4, 8), Cell(24, 8))
        self._place("R2", Cell(24, 8), Cell(4, 8))
        self._place("R3", Cell(26, 0), None)          # parked out of the way
        return "Head-on: R1 and R2 sent down the same aisle at each other."

    def _intersection(self) -> str:
        """Three robots aimed at one junction."""
        self.auto = False
        self.orders = None
        self.world.resume_all()
        self.world.reset_counters()
        self._clear_the_floor()
        # All three start exactly 8 squares from the junction at (13, 8), so
        # they arrive at the same moment. Get this wrong and one robot sails
        # through before the others turn up, and the demo shows nothing.
        self._place("R1", Cell(5, 8), Cell(24, 8))
        self._place("R2", Cell(21, 8), Cell(2, 8))
        self._place("R3", Cell(13, 0), Cell(13, 15))
        return "Intersection: all three robots converging on the junction at (13, 8)."

    def _deadlock(self) -> str:
        """The five-robot knot: two tasked pairs nose to nose in 1-wide
        aisles, plus a fifth converging on a contested pickup.

        R4 carries east while R5 collects west through the same row-4
        corridor cells; R1 and R3 mirror it in row 8; R2 drives up from
        the south for the D16 pickup R1 also wants. Static tasks and no
        order stream, so the jam is repeatable: watch DEADLOCKED climb,
        then JAMS BROKEN as PIBT backtracking clears each pair.
        """
        self.auto = False
        self.orders = None
        self.world.resume_all()
        while len(self.world.robots) < 5:
            self.world.add_robot_live()
        self.world.reset_counters()
        self._clear_the_floor()
        self._place("R1", Cell(21, 8), None)
        self._place("R2", Cell(25, 9), None)
        self._place("R3", Cell(22, 8), None)
        self._place("R4", Cell(21, 4), None)
        self._place("R5", Cell(22, 4), None)
        for rid, battery in (("R1", 34.0), ("R2", 87.0), ("R3", 33.0),
                             ("R4", 52.0), ("R5", 53.0)):
            robot = self.world.get(rid)
            if robot is not None:
                robot.battery = battery
        now = self.world.sim_time
        jobs = [
            # (robot, pickup, dropoff, product, shelf, qty, carrying)
            ("R4", Cell(23, 4), Cell(27, 2), "Router", "D13", 1, True),
            ("R5", Cell(2, 5), Cell(0, 6), "Laptop", "A11", 1, False),
            ("R1", Cell(25, 6), Cell(27, 6), "Charger", "D16", 1, False),
            ("R3", Cell(2, 1), Cell(0, 2), "Printer", "A1", 2, False),
            ("R2", Cell(25, 6), Cell(27, 10), "Charger", "D16", 1, False),
        ]
        for rid, pickup, dropoff, product, shelf, qty, carrying in jobs:
            task = self.world.announce_task(
                pickup, dropoff, product=product, shelf=shelf, quantity=qty)
            self._hand_task(rid, task, carrying, now)
        # Two more open jobs, like the live floor: everyone here already has
        # work, so these sit waiting for bids.
        self.world.announce_task(Cell(25, 6), Cell(27, 6), product="Charger",
                                 shelf="D16", quantity=2)
        self.world.announce_task(Cell(2, 5), Cell(0, 6), product="Laptop",
                                 shelf="A11", quantity=2)
        return ("Deadlock drill: R4/R5 nose to nose in row 4, R1/R3 nose to "
                "nose in row 8, R2 closing on R1's D16 pickup. Watch the jam "
                "form, then PIBT backtracking clear it.")

    def _hand_task(self, robot_id: str, task, carrying: bool,
                   now: float) -> None:
        """Give a staged robot its job directly, exactly as if it had won the
        auction for it.

        The robot's own board copy is assigned first, so the CLAIM broadcast
        that follows reads as a tie on every board -- including its own --
        and never as a steal. Skipping the broadcast would leave the rest of
        the fleet bidding on taken work.
        """
        robot = self.world.get(robot_id)
        if robot is None:
            return
        mine = Task(
            task_id=task.task_id,
            pickup=task.pickup, dropoff=task.dropoff,
            product=task.product, priority=task.priority,
            shelf=task.shelf, quantity=task.quantity,
            created_at=now, announced_at=now,
            status=TaskStatus.ASSIGNED, flexible=task.flexible,
        )
        mine.assigned_robot = robot_id
        mine.claimed_at = now
        mine.winning_bid = 0.0
        robot.board.add(mine)
        robot._my_bids[task.task_id] = 0.0
        task.assigned_robot = robot_id
        task.status = TaskStatus.ASSIGNED
        task.claimed_at = now
        task.winning_bid = 0.0
        robot._take_task(mine, now)
        robot._publish(self.world.bus, TaskClaim(
            robot_id=robot_id, timestamp=now, seq=robot.seq,
            task_id=task.task_id, action="CLAIM", cost=0.0))
        if carrying:
            mine.status = TaskStatus.CARRYING
            mine.picked_at = now
            task.status = TaskStatus.CARRYING
            task.picked_at = now
            robot.set_goal(task.dropoff)
            robot._publish(self.world.bus, TaskClaim(
                robot_id=robot_id, timestamp=now, seq=robot.seq,
                task_id=task.task_id, action="PICKED"))

    def _stop(self) -> str:
        """The emergency stop. Everything halts where it is.

        This used to only clear each robot's destination, which stopped almost
        nothing: a robot carrying a parcel hands itself the same destination
        back on the very next tick, on purpose, because finishing what you are
        carrying is meant to be hard to interrupt. So the button said "all
        robots parked" while half the fleet drove on. It now pulls the actual
        stop button on every robot, and they stay stopped until you press
        something else.
        """
        self.auto = False
        self.orders = None
        n = self.world.halt_all()
        return f"STOPPED. All {n} robots have halted where they stand."

    # -------------------------------------------------------------- helper

    def set_order_rate(self, every: float) -> str:
        """Change how fast orders arrive, without disturbing anything.

        Twenty robots on a six-second drip stand around with nothing to do --
        the fleet is not the bottleneck, the order book is. Turning the rate up
        is what makes a big fleet look busy.
        """
        every = max(0.25, min(30.0, float(every)))
        if self.orders is None:
            return "No orders running. Press Start orders first."
        self.orders.every = every
        # Bring the next one forward if it was already scheduled further out.
        self.orders._next_at = min(self.orders._next_at,
                                   self.world.sim_time + every)
        per_min = 60.0 / every
        return f"Orders now every {every:g}s (about {per_min:.0f} a minute)."

    def _clear_the_floor(self) -> None:
        """Park every robot on a square of its OWN, well out of the way, before
        staging a scenario.

        This used to cycle five corner squares with a modulo. That was fine for
        the three-robot demos it was written for, but with twenty robots on the
        floor it dropped four robots onto every square: twenty robots became
        five stacks, and the collision counter went off, correctly. Now each
        robot gets its own square and we run out of robots before squares.
        """
        for robot, cell in zip(self.world.robots.values(), self._parking()):
            robot.place(cell)

    def _parking(self) -> List[Cell]:
        """Squares to park spare robots on: never a staging square, and as far
        as possible from the corridor the demos run down (row 8, column 13),
        so parked robots do not stand in the way of the demo."""
        free = [c for c in self.world.grid.cells_of_kind(CellKind.FLOOR)
                if c not in STAGING_CELLS]
        # "Far" means far from BOTH the corridor and the crossing aisle, so
        # take whichever of the two it is nearer to and push that as high as
        # it will go. Ties break on the coordinates, so the demo is repeatable.
        free.sort(key=lambda c: (-min(abs(c.y - 8), abs(c.x - 13)), c.x, c.y))
        return free

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

    Seeded on purpose: the same seed must always produce the same list, so
    every run is repeatable and comparable.
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
        # Collect from a square with a SHELF next to it -- that is where the
        # goods physically are. Deliver to a packing bay.
        #
        # This used to ask "is any neighbour not walkable?", and off the edge
        # of the map counts as not walkable, so the entire outer wall looked
        # like a shelf. 82 of the 202 collect points were on the border with no
        # rack anywhere near them: two orders in five said "fetch a box from
        # the far wall". Asking for an actual shelf leaves 120 real ones.
        self.pickups = [
            c for c in grid.cells_of_kind(CellKind.FLOOR)
            if any(grid.in_bounds(n) and grid.kind(n) is CellKind.SHELF
                   for n in (Cell(c.x + 1, c.y), Cell(c.x - 1, c.y),
                             Cell(c.x, c.y + 1), Cell(c.x, c.y - 1)))
        ]
        self.dropoffs = (grid.cells_of_kind(CellKind.DROP)
                         or grid.cells_of_kind(CellKind.PICK))

    # How long one aisle stays the busy one, and what share of orders it takes.
    HOT_SECONDS = 120.0
    HOT_SHARE = 0.55

    def hot_zone(self, now: float) -> str:
        """Which aisle is busy at this moment. Deterministic from the seed."""
        zones = sorted({s.name[0] for s in self.world.inventory.by_name.values()})
        if not zones:
            return ""
        period = int(now // self.HOT_SECONDS)
        return zones[(self.seed + period * 3) % len(zones)]

    def _pick_shelf_with_a_pattern(self, now: float):
        inv = self.world.inventory
        zone = self.hot_zone(now)
        if zone and self._rng.random() < self.HOT_SHARE:
            in_zone = [s for s in inv.by_name.values() if s.name[0] == zone]
            if in_zone:
                return self._rng.choice(in_zone)
        return inv.pick_shelf(self._rng)

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

        # Phase 11. Pick a SHELF, not a square. The shelf knows which floor
        # square a robot has to stand on to reach it, so the robot still drives
        # exactly where it always drove -- the name is for the humans.
        #
        # Phase 12. Orders are NOT spread evenly over the warehouse, because
        # real ones are not: a few fast-moving lines account for most picks,
        # and which lines those are drifts through the day. So most orders come
        # from whichever aisle is currently hot, and the hot aisle changes every
        # few minutes.
        #
        # This was not put in to flatter the predictor. Before it, orders were
        # uniform random, the model correctly reported "no idea" for ever, and
        # pre-positioning could never be anything but noise -- there was nothing
        # there to learn. It is seeded like everything else, so every run
        # still gets the identical stream of orders.
        shelf = self._pick_shelf_with_a_pattern(now)
        # Phase 21. Off by default. On: a picker does not walk to the shelf
        # they thought of first if a closer bin has the same product --
        # they go collect the PRODUCT, from wherever the best current stock
        # of it actually is. Without this redirect, re-slotting could swap
        # shelves all day and no order would ever notice; this is the other
        # half that makes a shorter shelf into a shorter TRIP.
        if self.world.reslotting_enabled:
            shelf = (self.world.reslotter.best_shelf_for(
                        shelf.product, self.world.inventory, self.world.grid)
                    or shelf)
        pickup = shelf.face
        dropoff = self._rng.choice(self.dropoffs)
        quantity = self._rng.choice([1, 1, 1, 2, 2, 3])
        priority = self._rng.choice([5, 5, 5, 6, 7])
        self.world.inventory.take(shelf.name, quantity)
        # flexible=True: "take it to a packing station", not "to THAT square".
        task = self.world.announce_task(pickup, dropoff, shelf.product, priority,
                                        flexible=True, shelf=shelf.name,
                                        quantity=quantity)
        return (f"ORDER {task.task_id}: {task.line()} "
                f"deliver ({dropoff.x},{dropoff.y})")


def random_floor_cell(world: World) -> Cell:
    """A random open square, for the 'send somewhere' button."""
    from fleetx_core.grid import CellKind
    options = world.grid.cells_of_kind(CellKind.FLOOR)
    return random.choice(options)
