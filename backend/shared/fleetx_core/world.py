"""The whole warehouse: the map, every robot, and the clock.

PURE LOGIC ONLY. No web code, no ROS 2 code.

This is the thing the dashboard watches. In Part 2
the ROS 2 gateway builds the same object from live robot messages, so the
dashboard code does not change at all.
"""

import math
import random
import time
from typing import Callable, Dict, List, Optional

from .bus import FleetBus, InMemoryBus
from .conflicts import DEFAULT_HORIZON, Conflict
from .deadlock import STUCK_SECONDS
from .obstacles import SENSOR_RANGE, within_range
from .messages import Heartbeat, PoseUpdate, TaskAnnounce, TaskClaim
from .tasks import Task, TaskBoard, TaskStatus
from .reservations import Reservation
from .grid import Cell, CellKind, Grid, default_grid
from . import security
from .humans import Human
from .inventory import Inventory
from .slotting import Reslotter
from .robot import Robot, RobotStatus

# Two robots closer together than this (in squares) are touching.
# Each robot is drawn about 0.68 squares wide, so this is roughly the moment
# their bodies overlap on screen.
COLLISION_DISTANCE = 0.7

# How many recent crashes to remember for the dashboard's event feed.
_MAX_EVENTS = 40


class World:
    """Everything happening on the warehouse floor right now."""

    def __init__(self, grid: Optional[Grid] = None, bus: Optional[FleetBus] = None):
        self.grid: Grid = grid if grid is not None else default_grid()
        # The group chat the robots talk over. Swap this for a ROS 2 bus and
        # the robots do not notice the difference.
        self.bus: FleetBus = bus if bus is not None else InMemoryBus()
        self.robots: Dict[str, Robot] = {}
        self.halted: bool = False        # emergency stop pulled on the fleet
        # Phase 22: people on the floor. Empty until "+ Add person" is pressed.
        self.humans: Dict[str, Human] = {}
        self._human_seq: int = 0
        self._human_rng = random.Random(4242)

        # Phase 12. Off means idle robots wait where they finished, as before.
        # Kept as a switch so the question "does guessing actually help?" can be
        # answered by measuring both, rather than by assuming it does.
        self.prepositioning: bool = True
        # Phase 11: every rack has a name and something on it, so an order can
        # say "Mouse x2 from shelf A14" instead of a pair of coordinates.
        self.inventory: Inventory = Inventory(self.grid)
        # Phase 21: which products actually sell, and moving the best ones
        # closer to packing. A warehouse-operations decision, not a robot
        # one -- it runs the same for the whole fleet, the same way
        # Inventory itself already does.
        self.reslotter = Reslotter()
        # Off by default -- deliberately. It is not just the swap itself:
        # once on, an order for a product also resolves to the CLOSEST shelf
        # currently holding it (see OrderGenerator.tick() in scenarios.py),
        # not necessarily the exact shelf a scenario's own zone pattern
        # picked first. That changes which square orders actually collect
        # from. The live dashboard turns it on; headless runs leave it off.
        self.reslotting_enabled: bool = False
        self.sim_time: float = 0.0     # seconds since the run started
        self.ticks: int = 0
        self.started_at: float = time.time()

        # The number that has to stay at zero: collisions.
        self.collisions: int = 0
        self.tasks_completed: int = 0
        self.collision_events: List[Dict[str, object]] = []

        # --- Phase 4: did we see it coming? ---
        # For each pair of robots, when we last warned about them and how far
        # ahead. Used to check predictions against what actually happened.
        self._warned: Dict[frozenset, Dict[str, object]] = {}
        self.conflicts_raised: int = 0
        self.collisions_predicted: int = 0
        self.collisions_unpredicted: int = 0
        self.warnings_expired: int = 0     # warned, then no crash happened
        self.warning_times: List[float] = []

        # --- Phase 5: bookings and the jams they cause ---
        # A robot stuck this long is almost certainly in a deadlock, not just
        # politely waiting its turn. Phase 7 is what actually breaks these.
        self.deadlock_after: float = 6.0
        self.deadlocks_seen: int = 0
        self._deadlocked: set = set()

        # --- Phase 6: giving way ---
        self.reroutes: int = 0
        self.decisions: List[Dict[str, object]] = []

        # --- Phase 7: breaking jams that will never clear themselves ---
        self.deadlocks_broken: int = 0

        # --- Phase 8: things that should not be there ---
        # The TRUTH about what is on the floor. Robots cannot read this. They
        # only learn about it by driving close enough to see it -- see
        # _run_sensors below. That is what makes finding one worth broadcasting.
        self.obstacles: Dict[Cell, float] = {}
        self.obstacle_reports: int = 0

        # --- Phase 14: the order system retries ---
        # A job announced while the network was down reached nobody. Real order
        # systems re-send; without this, every order issued during a blackout is
        # simply lost for ever and the warehouse never recovers its backlog.
        self.reannounce_every: float = 8.0
        self._last_reannounce: float = 0.0

        # --- Phase 9: real jobs ---
        # The order book. Orders come from OUTSIDE the fleet -- somebody bought
        # something. Who does each job is settled by the robots between
        # themselves, which is the part that has to survive the server dying.
        self.board: TaskBoard = TaskBoard()
        self._task_seq: int = 0
        # Phase 23: every message the WORLD itself publishes -- order
        # announcements, and the "release my jobs" message sent on a robot's
        # behalf when it leaves or fails -- needs its own always-increasing
        # sequence number too, or authentication has nothing real to check.
        # Never reset with the rest of the scoreboard, for the same reason
        # a robot's own self.seq never is: a lower number after a reset would
        # read as a replay of a higher one everybody already remembers.
        self._system_seq: int = 0

        # Which pairs of robots are touching RIGHT NOW. Used so that one crash
        # is counted once, instead of once per tick while they overlap.
        self._touching: set = set()
        self._event_id: int = 0

        # Phase 19: which pairs the geometry backstop currently has braked.
    # --------------------------------------------------------------- fleet

    def add_robot(self, robot: Robot) -> Robot:
        if robot.robot_id in self.robots:
            raise ValueError(f"Robot {robot.robot_id} already exists.")
        if not self.grid.is_walkable(robot.cell):
            raise ValueError(
                f"Robot {robot.robot_id} cannot start on {robot.cell}, that square is blocked."
            )
        self.robots[robot.robot_id] = robot
        self.bus.register(robot.robot_id)
        return robot

    def get(self, robot_id: str) -> Optional[Robot]:
        return self.robots.get(robot_id)

    @property
    def active_robots(self) -> List[Robot]:
        """Robots that are switched on and can make decisions or move."""
        return [r for r in self.robots.values() if r.status is not RobotStatus.FAILED]

    @property
    def physical_robots(self) -> List[Robot]:
        """Every body still on the floor, including a failed robot."""
        return list(self.robots.values())

    # ---------------------------------------------------------------- clock

    def tick(self, dt: float, cost_fn: Optional[Callable] = None) -> None:
        """Move the whole world forward by dt seconds.

        Five passes, on purpose:
          1. every robot listens and speaks
          2. every robot thinks
          3. every robot checks its plan against everyone else's
          4. every robot books the squares ahead and stops if refused
          5. every robot moves
          6. check whether anybody crashed
        Thinking before moving means no robot gets an unfair advantage from
        being earlier in the list. From Phase 4 onward, conflict checking and
        negotiation slot in between passes 2 and 3.
        """
        if hasattr(self.bus, "set_time"):
            self.bus.set_time(self.sim_time)
        # Phase 8: hand each robot only what its own sensors can reach.
        self._run_sensors()

        # Phase 23: authentication happens INSIDE communicate(), before a
        # message is even handed to the rest of the brain -- a forged or
        # replayed one is logged here and goes no further.
        batched = hasattr(self.bus, "begin_batch") and hasattr(self.bus, "end_batch")
        if batched:
            self.bus.begin_batch()
        try:
            for robot in self.robots.values():
                for note in robot.communicate(self.bus, self.sim_time):
                    self.decisions.append({
                        "sim_time": round(self.sim_time, 2),
                        "robot_id": robot.robot_id, "text": note,
                    })
                    del self.decisions[:-20]
        finally:
            if batched:
                self.bus.end_batch()

        # Phase 14: has the network gone? Each robot decides for itself, from
        # whether anything at all has reached it lately.
        for robot in self.robots.values():
            note = robot.update_link_health(self.sim_time)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        # Phase 13: charge. Before jobs on purpose -- a robot that is about to
        # run out should hand its work back BEFORE the auction runs, not after.
        for robot in self.robots.values():
            note = robot.manage_battery(self.grid, self.bus, self.sim_time)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        # Phase 9: bid for jobs, claim what we win, get on with them.
        # Phase 9b: an idle robot must not sit on a pick or drop station.
        for robot in self.robots.values():
            note = robot.vacate_station(self.grid, self.sim_time)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        for robot in self.robots.values():
            for note in robot.work_on_tasks(self.grid, self.bus, self.sim_time):
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]
        # The robots negotiate the outcome between themselves; the world board
        # is only watching, so pull it into step with their copies here.
        # The dashboard reads this copy.
        self._sync_board()
        # Finished work is terminal. Keep a short recent window for display,
        # then reduce it to counters/tombstones on every local board.
        self.board.archive_finished()
        for robot in self.robots.values():
            robot.board.archive_finished()
        self._reannounce_forgotten_tasks()

        # Phase 21: does the warehouse know enough yet to move anything
        # closer to packing? A warehouse-operations decision, the same as
        # Inventory.take() already is -- runs identically whatever is
        # driving the fleet, never touches a robot, a route, or a booking.
        move = (self.reslotter.consider(self.inventory, self.grid, self.sim_time)
                if self.reslotting_enabled else None)
        if move:
            self.decisions.append({
                "sim_time": round(self.sim_time, 2),
                "robot_id": "INVENTORY", "text": move.sentence(),
            })
            del self.decisions[:-20]

        # Phase 12: anybody still with nothing to do goes and waits where the
        # next order is likely to be. AFTER the auction on purpose -- a real job
        # always beats a guess, and this only ever sees robots that got none.
        for robot in (self.robots.values() if self.prepositioning else ()):
            note = robot.consider_prepositioning(self.grid, self.sim_time,
                                                 self.inventory)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        # The Medic AMR's equivalent of task bidding / pre-positioning: pick a
        # failed robot to go fix, or head back to the dock. Before decide()
        # so a freshly-set goal gets its route planned this same tick.
        for robot in self.robots.values():
            note = robot.consider_medic_dispatch(self.grid, self.sim_time)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        for robot in self.robots.values():
            robot.decide(self.grid, cost_fn, self.sim_time)

        # Phase 4: every robot looks at its own plan against everyone else's.
        # It only LOOKS. Phase 5 books the square, Phase 6 decides who yields.
        for robot in self.robots.values():
            robot.check_conflicts(self.grid, self.sim_time, DEFAULT_HORIZON)
            robot.announce_conflicts(self.bus, self.sim_time)
        self._record_warnings()
        self._expire_warnings()

        # Decentralized coordination, every tick: SIPP reservations (Phase 5),
        # PIBT priority passing and rerouting (Phase 6), and deadlock
        # breaking (Phase 7). Each robot acts on its OWN local knowledge;
        # there is no boss and no baseline branch -- this is the only path.
        #
        # Phase 5: book the squares ahead, then check we are allowed onto the
        # next one. A robot that is not the owner stops before it.
        # Phase 6: standing is recomputed first -- the longer a robot has
        # been stuck, the more it is owed.
        for robot in self.robots.values():
            robot.update_priority(self.sim_time, dt)
        # Act on the intent horizon now, while robots are still several cells
        # apart. The priority winner requests a PIBT yield; only the loser
        # diverts, so both robots no longer wait for next-cell contact.
        for robot in self.robots.values():
            if robot.consider_predicted_conflict(
                    self.grid, self.sim_time, self.bus):
                self.reroutes += 1
        for robot in self.robots.values():
            robot.reserve_ahead(self.bus, self.sim_time)
        for robot in self.robots.values():
            robot.check_clearance(self.sim_time, dt)
        for robot in self.robots.values():
            note = robot.back_out_if_wedged(self.sim_time)
            if note:
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]
        # Phase 6: anyone held up decides whether to wait or go around.
        for robot in self.robots.values():
            if robot.consider_reroute(self.grid, self.sim_time, self.bus):
                self.reroutes += 1
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id,
                    "text": robot.last_decision,
                })
                del self.decisions[:-20]
        # Phase 7: anyone stuck far too long works out WHY, and either
        # asks the robot in front to move or gets out of the way itself.
        for robot in self.robots.values():
            note = robot.answer_requests(self.grid, self.sim_time)
            if note is None:
                note = robot.resume_after_yielding(
                    self.grid, self.sim_time)
            if note is None:
                note = robot.report_jam(self.grid, self.bus, self.sim_time)
            if note:
                # Only count it as BROKEN when a robot actually moved.
                # Counting every request would report 37 "recoveries"
                # for one jam that never cleared.
                if ("stepped aside" in note or "moving aside" in note
                        or "accepted PIBT request" in note):
                    self.deadlocks_broken += 1
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

        self._watch_for_jams()
        # Remember where everyone was, so we can spot two robots swapping places.
        was_at: Dict[str, Cell] = {rid: r.cell for rid, r in self.robots.items()}
        human_was_at: Dict[str, Cell] = {hid: h.cell for hid, h in self.humans.items()}

        for robot in self.robots.values():
            robot.advance(dt)

        self._service_medics()

        # Phase 22: people walk too. Anyone with nowhere to go gets a new
        # random destination -- the same idea the old robot free-roam patrol
        # used, just for the one thing a person here actually does: wander.
        # A failed robot stops moving; its body does not disappear. People
        # must still walk around it.
        robot_positions = [(r.x, r.y) for r in self.physical_robots]
        for human in self.humans.values():
            if not human.path:
                floor = self.grid.cells_of_kind(CellKind.FLOOR)
                for _ in range(6):                 # a few tries, then give up
                    target = self._human_rng.choice(floor)
                    if target != human.cell and human.set_goal(self.grid, target):
                        break
            human.advance(dt, robot_positions)

        self._detect_collisions(was_at, human_was_at)

        self.sim_time += dt
        self.ticks += 1

    # -------------------------------------------- real jobs (Phase 9)

    def _next_system_seq(self) -> int:
        self._system_seq += 1
        return self._system_seq

    def announce_task(self, pickup: Cell, dropoff: Cell, product: str = "",
                      priority: int = 5, flexible: bool = False,
                      shelf: str = "", quantity: int = 1) -> Task:
        """Put a new job on the air. Nobody is told who should do it."""
        # Phase 21: learn from this order regardless of who ends up doing
        # it -- the warehouse-level count, not a robot's own belief. Gated
        # the same as the rest of re-slotting: off means off, not quietly
        # learning in the background.
        if product and self.reslotting_enabled:
            self.reslotter.record(product, self.sim_time)
        self._task_seq += 1
        task = Task(
            task_id=f"T-{self._task_seq:03d}",
            pickup=pickup, dropoff=dropoff, product=product, priority=priority,
            shelf=shelf, quantity=quantity,
            created_at=self.sim_time, announced_at=self.sim_time,
            status=TaskStatus.ANNOUNCED, flexible=flexible,
        )
        self.board.add(task)
        self.bus.publish(security.seal(TaskAnnounce(
            robot_id="ORDERS", timestamp=self.sim_time, seq=self._next_system_seq(),
            task_id=task.task_id,
            pickup=(pickup.x, pickup.y), dropoff=(dropoff.x, dropoff.y),
            product=product, priority=priority, flexible=flexible,
            shelf=shelf, quantity=quantity,
        )))
        return task

    def _reannounce_forgotten_tasks(self) -> None:
        """Say the unclaimed jobs again, now and then.

        Nothing clever: the order system simply repeats itself. A job announced
        into a dead network reached nobody, and the robots have no way of
        knowing they missed it. Task ids never change, so a repeat that DID
        arrive is harmless -- the board already has it.
        """
        if self.sim_time - self._last_reannounce < self.reannounce_every:
            return
        self._last_reannounce = self.sim_time

        for task in self.board.tasks.values():
            if task.finished or task.assigned_robot is not None:
                continue
            heard_by = sum(1 for r in self.robots.values()
                           if r.board.get(task.task_id) is not None)
            if heard_by == len(self.robots):
                continue
            self.bus.publish(security.seal(TaskAnnounce(
                robot_id="ORDERS", timestamp=self.sim_time,
                seq=self._next_system_seq(),
                task_id=task.task_id,
                pickup=(task.pickup.x, task.pickup.y),
                dropoff=(task.dropoff.x, task.dropoff.y),
                product=task.product, priority=task.priority,
                flexible=task.flexible)))

    def _held_cells(self, rid: str, robot) -> List[List[int]]:
        if robot.table is None:
            return []
        return [[c.x, c.y] for c in robot.table.held_nodes(
            rid, self.sim_time, self.sim_time + 3.0)]

    def _sync_board(self) -> None:
        """Keep the dashboard's copy of the board in step with the robots'.

        The dashboard is an observer: it watches, it does not make safety
        decisions -- and here it does not hand out the work either.
        """
        for robot in self.robots.values():
            for rid, mine in robot.board.tasks.items():
                ours = self.board.get(rid)
                if ours is None:
                    self.board.add(mine)
                    continue
                if mine.status is TaskStatus.DONE and not ours.finished:
                    ours.status = TaskStatus.DONE
                    ours.done_at = mine.done_at
                    ours.assigned_robot = mine.assigned_robot
                elif not ours.finished:
                    ours.status = mine.status
                    ours.assigned_robot = mine.assigned_robot
                    ours.winning_bid = mine.winning_bid
                    ours.picked_at = mine.picked_at
                    ours.reassignments = max(ours.reassignments, mine.reassignments)
                    if mine.bids:
                        ours.bids.update(mine.bids)

    MAX_ROBOTS = 20

    def simulate_fake_message(self) -> dict:
        """Try to inject a forged message onto the bus, unsigned.

        The forged-message demo: "simulate a fake robot message ->
        authentication fails -> message is rejected -> event logged -> fleet
        continues safely." This is that button.

        Deliberately published with `self.bus.publish(...)` DIRECTLY, not
        through a robot's `_publish()` -- that is the whole point. `_publish`
        is the only place a message ever gets signed, so bypassing it is
        exactly what an attacker who is not a real fleet member would have to
        do: hand the bus a message that looks right but carries no valid tag.
        """
        alive = [r for r in self.robots.values()
                if r.status is not RobotStatus.FAILED]
        if not alive:
            return {"ok": False, "message": "No robot to impersonate."}
        target = alive[0]
        open_task = next((t for t in self.board.tasks.values()
                          if not t.finished), None)

        if open_task is not None:
            fake = TaskClaim(
                robot_id="GHOST-1", timestamp=self.sim_time, seq=1,
                task_id=open_task.task_id, action="CLAIM", cost=0.01)
            attempt = (f"a message pretending to be a robot called GHOST-1, "
                      f"claiming to have won {open_task.task_id}")
        else:
            fake = Heartbeat(
                robot_id=target.robot_id, timestamp=self.sim_time, seq=999999,
                battery=0.0, status="FAILED")
            attempt = (f"a message pretending to be {target.robot_id}, "
                      f"claiming it has failed")

        # No security.seal() here -- an attacker does not hold the fleet key,
        # so a forged message never carries a valid tag. That absence, not
        # anything clever, is what every robot's check_signature() catches.
        self.bus.publish(fake)
        return {"ok": True,
                "message": f"Injected {attempt}. No signature -- watch the "
                           f"event feed for every robot rejecting it."}

    def demand_view(self) -> dict:
        """What the fleet currently believes about where the work is.

        Read off R1's model. Every robot builds its own from the same
        announcements, so any of them would answer much the same -- and if they
        ever disagreed wildly that would itself be worth seeing.
        """
        first = next(iter(self.robots.values()), None)
        if first is None:
            return {"orders_seen": 0, "ready": False, "busiest": None,
                    "reason": "", "zones": [], "slot": 0}
        return first.demand.to_dict(self.sim_time)

    def drain_batteries(self, low: float = 8.0, high: float = 22.0) -> dict:
        """Run every robot down flat, to force the charging demo.

        Without this you wait about five minutes for batteries to fall on their
        own. Twenty robots suddenly needing four bays is also the interesting
        case: they have to take turns rather than all setting off at once.
        """
        rng = random.Random(len(self.robots))
        for robot in self.robots.values():
            robot.battery = rng.uniform(low, high)
            robot._charge_checked_at = -99.0
        return {"ok": True,
                "message": f"Batteries drained to {low:.0f}-{high:.0f}% on "
                           f"{len(self.robots)} robots. Watch them take turns."}

    def halt_all(self) -> int:
        """Pull the emergency stop on the whole fleet. Returns how many stopped."""
        self.halted = True
        for robot in self.robots.values():
            robot.halt()
        return len(self.robots)

    def resume_all(self) -> int:
        """Release the emergency stop on the whole fleet."""
        self.halted = False
        for robot in self.robots.values():
            robot.resume()
        return len(self.robots)

    def add_robot_live(self) -> Dict[str, object]:
        """Put another robot on the floor of a warehouse that is already running.

        It joins with an empty head: no bookings, no job, nobody in its
        notebook. Within a second it has heard the others and they have heard
        it. Nothing special is needed, because that is how a robot joins on
        DDS too -- discovery is automatic.
        """
        if len(self.robots) >= self.MAX_ROBOTS:
            return {"ok": False,
                    "message": f"{self.MAX_ROBOTS} robots is the most this warehouse holds."}

        # Keep clear of where robots actually ARE, not just which square they
        # are registered on: one that is part way along a segment has its body
        # between two squares, and dropping a new robot beside it would put two
        # bodies in the same space before either had moved.
        def clear(cell: Cell) -> bool:
            if not self.grid.is_walkable(cell):
                return False
            for r in self.robots.values():
                if (r.x - cell.x) ** 2 + (r.y - cell.y) ** 2 < 1.6 ** 2:
                    return False
            return True

        spot = next((c for c in FLEET_START_CELLS if clear(c)), None)
        if spot is None:
            spot = next((c for c in self.grid.cells_of_kind(CellKind.FLOOR)
                         if clear(c)), None)
        if spot is None:
            return {"ok": False, "message": "Nowhere free to put another robot."}

        used = {r.robot_id for r in self.robots.values()}
        n = 1
        while f"R{n}" in used:
            n += 1
        robot = self.add_robot(Robot(robot_id=f"R{n}", cell=spot))
        if getattr(self, "halted", False):
            robot.halt()
        # Phase 23. This name may have belonged to a robot that left earlier.
        # remove_robot() already tells everyone to forget it then, but the
        # departing robot can still have one legitimate message in flight at
        # that exact moment (releasing its last job), and THAT message is
        # what every other robot's guard ends up remembering as "the highest
        # sequence number R3 ever sent" -- a real, high number. The new R3
        # starts counting from 1 again, which is always lower, so its very
        # first message reads as a replay of one already seen and never
        # arrives. Forgetting again HERE, at the moment the name is actually
        # reissued, is the point where it is truly safe: nothing further will
        # ever legitimately arrive under the old identity.
        for other in self.robots.values():
            if other.robot_id != robot.robot_id:
                other._replay_guard.forget(robot.robot_id)
        return {"ok": True,
                "message": f"{robot.robot_id} joined at ({spot.x}, {spot.y}). "
                           f"{len(self.robots)} robots now."}

    # Off to one side, on ordinary floor, away from every FLEET_START_CELLS
    # spot and every scenario's staging squares -- so adding one never
    # collides with whatever demo is already set up.
    MEDIC_HOME = Cell(0, 14)
    MEDIC_REPAIR_RANGE = 1.5      # squares apart counts as "alongside it"
    MEDIC_REPAIR_DURATION = 2.5   # seconds standing there before it counts as fixed

    def add_medic(self, home: Optional[Cell] = None,
                 robot_id: str = "MEDIC") -> Dict[str, object]:
        """Put the repair robot on the floor, parked at its dock.

        It carries no logistics work of its own -- see is_medic guards in
        Robot._bid_and_claim and Robot.consider_prepositioning -- and goes
        nowhere until a DistressSignal names a robot for it to reach.
        """
        if robot_id in self.robots:
            return {"ok": False, "message": f"{robot_id} is already on the floor."}
        dock = home if home is not None else self.MEDIC_HOME
        if not self.grid.is_walkable(dock):
            return {"ok": False,
                    "message": f"({dock.x}, {dock.y}) is not a floor square."}
        self.add_robot(Robot(robot_id=robot_id, cell=dock, is_medic=True, home=dock))
        return {"ok": True,
                "message": f"{robot_id} is on standby at ({dock.x}, {dock.y})."}

    def _service_medics(self) -> None:
        """A medic that has reached the robot it was sent to fix works on it
        for MEDIC_REPAIR_DURATION seconds, then fixes it.

        Only World may flip another robot's status -- a Robot object never
        touches a peer's state directly, the same boundary fail_robot() and
        revive_robot() already draw -- so the medic itself only ever decides
        where to go (Robot.consider_medic_dispatch); this is what actually
        runs the repair once it physically gets there.
        """
        for medic in self.robots.values():
            if not medic.is_medic or medic.medic_target is None:
                continue
            target = self.robots.get(medic.medic_target)
            if target is None or target.status is not RobotStatus.FAILED:
                medic.mark_repaired(medic.medic_target)
                medic.repair_started_at = None
                continue

            dx, dy = medic.x - target.x, medic.y - target.y
            if dx * dx + dy * dy > self.MEDIC_REPAIR_RANGE ** 2:
                medic.repair_started_at = None    # stepped out of range mid-repair
                continue

            if medic.repair_started_at is None:
                medic.repair_started_at = self.sim_time
                medic.status = RobotStatus.REPAIRING
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": medic.robot_id,
                    "text": f"{medic.robot_id} reached {medic.medic_target} - repairing.",
                })
                del self.decisions[:-20]
                continue

            if self.sim_time - medic.repair_started_at < self.MEDIC_REPAIR_DURATION:
                continue

            fixed_id = medic.medic_target
            self.revive_robot(fixed_id)
            medic.mark_repaired(fixed_id)
            medic.repair_started_at = None
            medic.status = RobotStatus.IDLE   # let decide() plan the trip home
            medic.set_goal(medic.home)
            self.decisions.append({
                "sim_time": round(self.sim_time, 2),
                "robot_id": medic.robot_id,
                "text": f"{medic.robot_id} repaired {fixed_id} - heading back to dock.",
            })
            del self.decisions[:-20]

    def remove_robot(self, robot_id: Optional[str] = None) -> Dict[str, object]:
        """Take a robot off the floor.

        Its bookings go back and its job goes back up for auction -- exactly
        what happens when one fails, because from the fleet's point of view a
        robot that has been removed and one that has died look the same.
        """
        if len(self.robots) <= 1:
            return {"ok": False, "message": "One robot has to stay."}
        if robot_id is None:
            # The medic does not count as a spare logistics robot to remove
            # by default, and its id ("MEDIC") does not fit the R<n> pattern
            # this picks the highest of anyway.
            removable = [rid for rid, r in self.robots.items() if not r.is_medic]
            if not removable:
                return {"ok": False, "message": "No removable robot on the floor."}
            robot_id = sorted(removable, key=lambda r: int(r[1:]))[-1]
        robot = self.robots.get(robot_id)
        if robot is None:
            return {"ok": False, "message": f"There is no robot called {robot_id}."}

        robot.release_all(self.bus, self.sim_time)
        held = self.board.release_all(robot_id)
        # Sent AS the departing robot ("its own last words"), so it must
        # continue that robot's OWN sequence space, one past its last real
        # message -- not restart at 0, or every other robot's memory of R3's
        # much higher last-seen number would read this as a replay and throw
        # the release away, and the job would never come back up for auction.
        for task in held:
            self.bus.publish(security.seal(TaskClaim(
                robot_id=robot_id, timestamp=self.sim_time, seq=robot.seq + 1,
                task_id=task.task_id, action="RELEASE")))
        del self.robots[robot_id]
        self._deadlocked.discard(robot_id)
        if hasattr(self.bus, "unregister"):
            self.bus.unregister(robot_id)
        # Phase 23. Its NAME goes back into the pool -- add_robot_live() picks
        # the lowest free number, so a new robot can turn up as "R3" again.
        # Everyone else remembers the LAST sequence number the old R3 ever
        # sent; without forgetting it, the new R3's very first message (seq 1)
        # reads as a replay of one already seen and every robot in the fleet
        # silently threw its radio away. Found by testing exactly this.
        for other in self.robots.values():
            other._replay_guard.forget(robot_id)
        names = ", ".join(t.task_id for t in held) or "no jobs"
        return {"ok": True,
                "message": f"{robot_id} left. Released: {names}. "
                           f"{len(self.robots)} robots now."}

    # ----------------------------------------------------------- Phase 22

    MAX_HUMANS = 8

    def add_human(self) -> Dict[str, object]:
        """Put a person on the floor. They start walking on their own --
        world.tick() gives anyone with nowhere to go a new random destination
        every tick, the same idea as a robot's old free-roam patrol."""
        if len(self.humans) >= self.MAX_HUMANS:
            return {"ok": False,
                    "message": f"{self.MAX_HUMANS} people is enough for one floor."}

        def clear(cell: Cell) -> bool:
            if not self.grid.is_walkable(cell):
                return False
            for r in self.robots.values():
                if (r.x - cell.x) ** 2 + (r.y - cell.y) ** 2 < 1.6 ** 2:
                    return False
            for h in self.humans.values():
                if (h.x - cell.x) ** 2 + (h.y - cell.y) ** 2 < 1.6 ** 2:
                    return False
            return True

        spot = next((c for c in self.grid.cells_of_kind(CellKind.FLOOR)
                    if clear(c)), None)
        if spot is None:
            return {"ok": False, "message": "Nowhere free to put another person."}

        self._human_seq += 1
        human_id = f"P{self._human_seq}"
        while human_id in self.humans:            # never actually loops in
            self._human_seq += 1                   # practice; guards a reused id
            human_id = f"P{self._human_seq}"
        self.humans[human_id] = Human(human_id=human_id, cell=spot)
        return {"ok": True,
                "message": f"{human_id} joined at ({spot.x}, {spot.y}). "
                           f"{len(self.humans)} people on the floor now."}

    def remove_human(self, human_id: Optional[str] = None) -> Dict[str, object]:
        """Take a person off the floor."""
        if not self.humans:
            return {"ok": False, "message": "Nobody on the floor to remove."}
        if human_id is None:
            human_id = sorted(self.humans, key=lambda h: int(h[1:]))[-1]
        if human_id not in self.humans:
            return {"ok": False, "message": f"There is no person called {human_id}."}
        del self.humans[human_id]
        return {"ok": True,
                "message": f"{human_id} left. {len(self.humans)} people on the floor now."}

    def fail_robot(self, robot_id: str) -> Dict[str, object]:
        """Switch a robot off mid-job. Failure drill."""
        robot = self.get(robot_id)
        if robot is None:
            return {"ok": False, "message": f"There is no robot called {robot_id}."}
        # The grid simulator has no braking-distance model. Settle a robot
        # that fails mid-step onto the nearest endpoint before freezing it.
        # Leaving it fractionally across two cells can seal an otherwise
        # passable one-cell aisle forever; making it vanish is unsafe too.
        stopped_at = Cell(math.floor(robot.x + 0.5),
                          math.floor(robot.y + 0.5))
        robot.place(stopped_at)
        # Release BEFORE marking it FAILED: release_all() only broadcasts its
        # RELEASE messages "if bus is not None and self.status is not
        # RobotStatus.FAILED" -- so calling it after flipping the status
        # (the order this used to run in) silently swallowed every one of
        # them, and the rest of the fleet kept believing a dead robot still
        # owned its last-claimed squares until those bookings simply expired.
        robot.release_all(self.bus, self.sim_time)
        robot.status = RobotStatus.FAILED
        # Say so immediately, not left to a stale-notebook timeout. A failed
        # robot's own communicate() returns before publishing anything ever
        # again, so without this, every peer's picture of it just goes quiet
        # -- which is indistinguishable from a dropped packet for up to
        # DEFAULT_STALE_AFTER seconds. This is what lets Robot._live_peer
        # (and the hard block in Robot._track_failed_peers) react at once.
        #
        # The PoseUpdate matters just as much as the Heartbeat here: a robot
        # failed mid-step, so its last ORDINARY pose (sent up to POSE_PERIOD
        # ago, mid-transit) still names the square it was leaving, not the
        # one `place(stopped_at)` just snapped it onto -- and since it never
        # speaks again, that stale square would stick in every peer's
        # notebook forever. That mismatch is exactly what let a robot drive
        # right past a failed peer's TRUE square without ever reporting it:
        # its own LiDAR saw one cell, its notebook's memory of the failed
        # robot said another, and the two never matched.
        robot.seq += 2
        self.bus.publish(security.seal(PoseUpdate(
            robot_id=robot_id, timestamp=self.sim_time, seq=robot.seq - 1,
            x=robot.x, y=robot.y, cell=(stopped_at.x, stopped_at.y),
            velocity=0.0, heading=robot.heading,
        )))
        self.bus.publish(security.seal(Heartbeat(
            robot_id=robot_id, timestamp=self.sim_time, seq=robot.seq,
            battery=robot.battery, status=RobotStatus.FAILED.value,
        )))
        held = self.board.release_all(robot_id)
        # Tell everyone, or the other robots keep the job down as "R2's" and
        # nobody ever picks it up. Continues the robot's own sequence space
        # -- see the identical note in remove_robot().
        for task in held:
            self.bus.publish(security.seal(TaskClaim(
                robot_id=robot_id, timestamp=self.sim_time, seq=robot.seq + 1,
                task_id=task.task_id, action="RELEASE")))
        names = ", ".join(t.task_id for t in held) or "no jobs"
        return {"ok": True,
                "message": f"{robot_id} has failed. Released: {names}."}

    def revive_robot(self, robot_id: str) -> Dict[str, object]:
        robot = self.get(robot_id)
        if robot is None:
            return {"ok": False, "message": f"There is no robot called {robot_id}."}
        robot.status = RobotStatus.IDLE
        robot.battery = max(robot.battery, 60.0)
        return {"ok": True, "message": f"{robot_id} is back."}

    def all_tasks_done(self) -> bool:
        return self.board.all_done()

    # -------------------------------------- things in the way (Phase 8)

    def add_obstacle(self, cell: Cell) -> Dict[str, object]:
        """Drop something in an aisle."""
        if not self.grid.is_walkable(cell):
            return {"ok": False, "message": "That square is already a shelf."}
        for robot in self.robots.values():
            if robot.cell == cell:
                return {"ok": False,
                        "message": f"{robot.robot_id} is standing there."}
        self.obstacles[cell] = self.sim_time
        return {"ok": True, "message": f"Something is now blocking ({cell.x}, {cell.y})."}

    def remove_obstacle(self, cell: Cell) -> Dict[str, object]:
        if self.obstacles.pop(cell, None) is None:
            return {"ok": False, "message": "Nothing there to remove."}
        return {"ok": True, "message": f"({cell.x}, {cell.y}) is clear again."}

    def clear_obstacles(self) -> Dict[str, object]:
        count = len(self.obstacles)
        self.obstacles.clear()
        return {"ok": True, "message": f"Removed {count} obstacle(s)."}

    def _run_sensors(self) -> None:
        """Act as each robot's laser scanner.

        A robot is told about squares within SENSOR_RANGE, and only those. It
        learns two things: which nearby squares have something in them, and
        which nearby squares it can see are empty (so a block it was told about
        earlier can be forgotten).
        """
        for robot in self.robots.values():
            if robot.status is RobotStatus.FAILED:
                continue

            seen_blocked = [
                c for c in self.obstacles
                if within_range(robot.x, robot.y, c, SENSOR_RANGE)
                and self._has_line_of_sight(robot.x, robot.y, c.x, c.y)
            ]

            # Squares it believes are blocked, can see plainly, and are empty.
            seen_clear = [c for c in robot.blocked_cells(self.sim_time)
                          if c not in self.obstacles
                          and within_range(robot.x, robot.y, c, SENSOR_RANGE)
                          and self._has_line_of_sight(robot.x, robot.y, c.x, c.y)]

            # Robots reflect a laser beam exactly like a dropped pallet does.
            # This is what makes the safety reflex work with the radio dead --
            # it needs no messages, only line of sight.
            robot.sense_robots(
                [(other.x, other.y, other.cell)
                 for other in self.robots.values()
                 if other.robot_id != robot.robot_id
                 and within_range(robot.x, robot.y, other.cell, SENSOR_RANGE + 1.0)
                 and self._has_line_of_sight(robot.x, robot.y, other.x, other.y)],
                self.sim_time)

            # Phase 22: the same laser sees people too. A dedicated, wider
            # range -- see Robot.PERSON_SENSE_RANGE -- because a person
            # deserves earlier warning than a dropped box does.
            robot.sense_humans(
                [(human.human_id, human.x, human.y, human.cell)
                 for human in self.humans.values()
                 if within_range(robot.x, robot.y, human.cell,
                                 robot.PERSON_SENSE_RANGE)
                 and self._has_line_of_sight(robot.x, robot.y,
                                             human.x, human.y)],
                self.sim_time)

            for note in robot.sense(seen_blocked, seen_clear, self.sim_time):
                self.obstacle_reports += 1
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

    def _has_line_of_sight(self, x0: float, y0: float,
                           x1: float, y1: float) -> bool:
        """Whether a laser ray reaches its target without crossing a shelf.

        Coordinates name cell centres. This grid traversal visits every cell
        crossed by the ray. Exact corner crossings are conservative: a shelf
        on either side blocks sight instead of letting the laser see through
        a zero-width diagonal crack.
        """
        start = Cell(math.floor(x0 + 0.5), math.floor(y0 + 0.5))
        target = Cell(math.floor(x1 + 0.5), math.floor(y1 + 0.5))
        if not self.grid.in_bounds(start) or not self.grid.in_bounds(target):
            return False
        if start == target:
            return True

        dx, dy = x1 - x0, y1 - y0
        step_x = 1 if dx > 0 else (-1 if dx < 0 else 0)
        step_y = 1 if dy > 0 else (-1 if dy < 0 else 0)
        infinity = float("inf")
        delta_x = abs(1.0 / dx) if dx else infinity
        delta_y = abs(1.0 / dy) if dy else infinity
        boundary_x = start.x + (0.5 if step_x > 0 else -0.5)
        boundary_y = start.y + (0.5 if step_y > 0 else -0.5)
        max_x = (boundary_x - x0) / dx if dx else infinity
        max_y = (boundary_y - y0) / dy if dy else infinity
        current = start

        def blocks(cell: Cell) -> bool:
            return cell != target and not self.grid.is_walkable(cell)

        while current != target:
            if max_x < max_y:
                current = Cell(current.x + step_x, current.y)
                max_x += delta_x
            elif max_y < max_x:
                current = Cell(current.x, current.y + step_y)
                max_y += delta_y
            else:
                side_x = Cell(current.x + step_x, current.y)
                side_y = Cell(current.x, current.y + step_y)
                if blocks(side_x) or blocks(side_y):
                    return False
                current = Cell(current.x + step_x, current.y + step_y)
                max_x += delta_x
                max_y += delta_y
            if blocks(current):
                return False
        return True

    # ------------------------------------------------ predictions (Phase 4)

    # A warning that has not been repeated for this long has lapsed: the
    # robots re-planned, or drifted apart, and there was no crash.
    WARNING_LAPSE = 3.0

    def _record_warnings(self) -> None:
        """Note every pair of robots somebody has warned about, and WHEN FIRST.

        first_at is written once and never overwritten. That matters: the
        warning time we report is how long before the crash the alarm FIRST
        went up. Using the latest reading instead would report nearly zero
        every time, because a live conflict counts down to zero as the robots
        close in -- which would make the headline number meaningless.
        """
        now = self.sim_time
        for robot in self.robots.values():
            for conflict in robot.conflicts:
                pair = conflict.pair
                entry = self._warned.get(pair)
                if entry is None:
                    self.conflicts_raised += 1
                    entry = {"first_at": now}
                    self._warned[pair] = entry
                entry["at"] = now
                entry["lead"] = conflict.lead_time
                entry["cell"] = (conflict.cell.x, conflict.cell.y)
                entry["kind"] = conflict.kind.value

    def _expire_warnings(self) -> None:
        """Drop warnings that quietly went away. These are the false alarms."""
        now = self.sim_time
        lapsed = [p for p, e in self._warned.items()
                  if now - e["at"] > self.WARNING_LAPSE]
        for pair in lapsed:
            self.warnings_expired += 1
            del self._warned[pair]

    def active_conflicts(self) -> List[Dict[str, object]]:
        """Every conflict any robot can currently see, soonest first.

        Two robots can disagree about whether there is a problem -- each one is
        working from its own notebook. That is what decentralised means. Here
        we merge them purely so the dashboard can draw them.
        """
        merged: Dict[tuple, Dict[str, object]] = {}
        for robot in self.robots.values():
            for conflict in robot.conflicts:
                key = (tuple(sorted(conflict.pair)), conflict.cell.x, conflict.cell.y)
                row = conflict.to_dict()
                row["seen_by"] = [robot.robot_id]
                if key in merged:
                    merged[key]["seen_by"].append(robot.robot_id)
                    if row["lead_time"] < merged[key]["lead_time"]:
                        merged[key].update({k: v for k, v in row.items() if k != "seen_by"})
                else:
                    merged[key] = row
        for row in merged.values():
            row["seen_by"] = sorted(set(row["seen_by"]))
        return sorted(merged.values(), key=lambda r: r["lead_time"])

    # ------------------------------------------------------ jams (Phase 5)

    def _watch_for_jams(self) -> None:
        """Count robots that have been stuck far too long.

        Phase 5 does not fix these -- it causes them. Booking squares turns
        crashes into jams. Measuring them now is the honest way to show what
        Phases 6 and 7 are actually for.
        """
        for robot in self.robots.values():
            # Either measure counts. A robot that keeps rerouting is not
            # "waiting", but if it has gone nowhere for ages it is still jammed
            # -- and the dashboard used to report 0 deadlocks while a robot sat
            # outside the packing station for 800 seconds.
            stuck = max(robot.waited_for(self.sim_time),
                        robot.stalled_for(self.sim_time))
            if stuck >= self.deadlock_after:
                if robot.robot_id not in self._deadlocked:
                    self._deadlocked.add(robot.robot_id)
                    self.deadlocks_seen += 1
            elif robot.waiting_since is None and robot.stalled_for(self.sim_time) < 1.0:
                self._deadlocked.discard(robot.robot_id)

    def stuck_robots(self) -> List[str]:
        return sorted(self._deadlocked)

    # ----------------------------------------------------------- collisions

    def _detect_collisions(self, was_at: Dict[str, Cell],
                           human_was_at: Optional[Dict[str, Cell]] = None) -> None:
        """Count crashes honestly.

        Three ways two robots can crash, and we check all three:

        1. SAME SQUARE   -- both standing on the same square.

        2. HEAD-ON SWAP  -- they drove straight through each other and traded
                            places. They never share a square, so checking
                            squares alone would MISS this. It is an edge
                            conflict: "Do not check only nodes."

                                R1 -> <- R2      becomes      <- R2   R1 ->

        3. TOUCHING      -- their bodies overlap part-way between squares.
                            This is the one your eyes see on the dashboard.

        One crash counts ONCE. The world ticks 20 times a second, so without
        this the same crash would be counted 20 times and the scoreboard
        would be nonsense.
        """
        robots = self.physical_robots
        touching_now = set()

        for i in range(len(robots)):
            for j in range(i + 1, len(robots)):
                a, b = robots[i], robots[j]
                reason = self._crash_reason(a, b, was_at)
                if reason is None:
                    continue

                pair = frozenset((a.robot_id, b.robot_id))
                touching_now.add(pair)

                if pair not in self._touching:      # a NEW crash, not the same one
                    self.collisions += 1
                    self._event_id += 1

                    # Did anybody call this one in advance?
                    warned = self._warned.get(pair)
                    warning = None
                    if warned is not None:
                        # how long ago the alarm FIRST went up for this pair
                        warning = round(self.sim_time - warned["first_at"], 2)
                        self.collisions_predicted += 1
                        self.warning_times.append(warning)
                    else:
                        self.collisions_unpredicted += 1

                    self.collision_events.append({
                        "id": self._event_id,
                        "robots": sorted([a.robot_id, b.robot_id]),
                        "reason": reason,
                        "x": round((a.x + b.x) / 2, 2),
                        "y": round((a.y + b.y) / 2, 2),
                        "sim_time": round(self.sim_time, 2),
                        "predicted": warned is not None,
                        "warning": warning,
                    })
                    self._warned.pop(pair, None)
                    del self.collision_events[:-_MAX_EVENTS]

        # Phase 22: a robot touching a PERSON counts on the exact same
        # scoreboard, not a separate one that could quietly go unwatched.
        # "Zero collisions" means zero of either kind, full stop.
        human_was_at = human_was_at or {}
        for robot in robots:
            for human in self.humans.values():
                reason = self._crash_reason_person(robot, human, was_at, human_was_at)
                if reason is None:
                    continue

                pair = frozenset((robot.robot_id, f"person:{human.human_id}"))
                touching_now.add(pair)

                if pair not in self._touching:
                    self.collisions += 1
                    self._event_id += 1
                    self.collision_events.append({
                        "id": self._event_id,
                        "robots": sorted([robot.robot_id, human.human_id]),
                        "reason": f"{reason} (person)",
                        "x": round((robot.x + human.x) / 2, 2),
                        "y": round((robot.y + human.y) / 2, 2),
                        "sim_time": round(self.sim_time, 2),
                        "predicted": False,
                        "warning": None,
                    })
                    del self.collision_events[:-_MAX_EVENTS]

        self._touching = touching_now

    @staticmethod
    def _crash_reason(a: Robot, b: Robot, was_at: Dict[str, Cell]) -> Optional[str]:
        if a.cell == b.cell:
            return "same square"

        moved = was_at.get(a.robot_id) != a.cell or was_at.get(b.robot_id) != b.cell
        if moved and was_at.get(a.robot_id) == b.cell and was_at.get(b.robot_id) == a.cell:
            return "head-on swap"

        dx, dy = a.x - b.x, a.y - b.y
        if (dx * dx + dy * dy) < COLLISION_DISTANCE * COLLISION_DISTANCE:
            return "touching"

        return None

    @staticmethod
    def _crash_reason_person(robot: Robot, human: Human,
                             was_at: Dict[str, Cell],
                             human_was_at: Dict[str, Cell]) -> Optional[str]:
        """The same three checks as _crash_reason, for a robot and a person.

        This should never fire. If it ever does, it means the hard stop in
        _local_safety_says_stop() -- PERSON_STOP_RADIUS, checked every tick,
        in both fleets -- has a hole in it, and that is a real bug to find
        and fix, not a number to explain away.
        """
        if robot.cell == human.cell:
            return "same square"

        moved = (was_at.get(robot.robot_id) != robot.cell
                or human_was_at.get(human.human_id) != human.cell)
        if (moved and was_at.get(robot.robot_id) == human.cell
                and human_was_at.get(human.human_id) == robot.cell):
            return "head-on swap"

        dx, dy = robot.x - human.x, robot.y - human.y
        if (dx * dx + dy * dy) < COLLISION_DISTANCE * COLLISION_DISTANCE:
            return "touching"

        return None

    def reset_counters(self) -> None:
        """Zero the scoreboard and start the clock again.

        Note this winds sim_time back to zero, so EVERY booking has to go too.
        Bookings are stamped with absolute times; left behind, they would sit in
        the table claiming squares from the middle of next week, and the
        warehouse would gum up solid. (It did exactly that.)
        """
        self.collisions = 0
        self.tasks_completed = 0
        self.collision_events.clear()
        self._touching.clear()
        self._event_id = 0
        self._warned.clear()
        self.conflicts_raised = 0
        self.collisions_predicted = 0
        self.collisions_unpredicted = 0
        self.warnings_expired = 0
        self.warning_times.clear()
        self.deadlocks_seen = 0
        self._deadlocked.clear()
        self.reroutes = 0
        self.decisions.clear()
        self.deadlocks_broken = 0
        self.obstacle_reports = 0
        self._last_reannounce = 0.0
        self.board = TaskBoard()
        self._task_seq = 0
        self.sim_time = 0.0
        self.ticks = 0
        if hasattr(self.bus, "reset_clock"):
            self.bus.reset_clock(0.0)
        for robot in self.robots.values():
            robot.forget_bookings()
            robot.steps = 0
            robot.distance = 0.0
            robot.replans = 0
            robot.messages_sent = 0
            robot.messages_heard = 0
            robot.conflicts_seen = 0
            robot._announced.clear()
            robot.wait_time = 0.0
            robot.refusals = 0
            robot.grants = 0
            robot.reroutes = 0
            robot.last_decision = None
            robot.wait_credit = 0.0
            robot.yields = 0
            robot.asked = 0
            robot.obstacles_found = 0
            robot.tasks_done = 0
            robot.bids_won = 0
        if hasattr(self.bus, "reset_stats"):
            self.bus.reset_stats()

    # ------------------------------------------------------------------ KPIs

    def kpis(self) -> Dict[str, object]:
        """The numbers on the dashboard's top strip."""
        robots = list(self.robots.values())
        task_stats = self.board.stats(self.sim_time)
        moving = sum(1 for r in robots if r.status is RobotStatus.MOVING)
        idle = sum(1 for r in robots if r.status is RobotStatus.IDLE)
        blocked = sum(1 for r in robots if r.status is RobotStatus.BLOCKED)
        charging = sum(1 for r in robots if r.status is RobotStatus.CHARGING)
        distance = sum(r.distance for r in robots)
        battery = (sum(r.battery for r in robots) / len(robots)) if robots else 0.0

        return {
            "robots": len(robots),
            "moving": moving,
            "idle": idle,
            "blocked": blocked,
            "charging": charging,
            "collisions": self.collisions,
            "tasks_completed": task_stats["done"],
            "total_distance": round(distance, 1),
            "avg_battery": round(battery, 1),
            "sim_time": round(self.sim_time, 1),
            "messages_sent": sum(r.messages_sent for r in robots),
            "messages_heard": sum(r.messages_heard for r in robots),
            "stale_links": sum(
                len(r.fleet.stale(self.sim_time)) for r in robots if r.fleet
            ),
            "conflicts_active": len(self.active_conflicts()),
            "conflicts_raised": self.conflicts_raised,
            "collisions_predicted": self.collisions_predicted,
            "collisions_unpredicted": self.collisions_unpredicted,
            "warnings_expired": self.warnings_expired,
            "waiting": sum(1 for r in robots if r.status is RobotStatus.WAITING),
            "deadlocked": len(self._deadlocked),
            "deadlocks_seen": self.deadlocks_seen,
            "total_wait": round(sum(r.wait_time for r in robots), 1),
            "refusals": sum(r.refusals for r in robots),
            "reroutes": self.reroutes,
            "deadlocks_broken": self.deadlocks_broken,
            "obstacles": len(self.obstacles),
            "safe_mode": sum(1 for r in robots if r.safe_mode),
            **{f"task_{k}": v for k, v in task_stats.items()
               if k not in {"avg_task_time", "tasks_per_hour",
                            "total_task_time"}},
            "blocked_known": len(set().union(*[r.blocked_cells(self.sim_time)
                                               for r in robots]) if robots else set()),
            "yields": sum(r.yields for r in robots),
            "avg_priority": (round(sum(r.priority for r in robots) / len(robots) / 10.0, 1)
                             if robots else 0.0),
            "avg_warning": (round(sum(self.warning_times) / len(self.warning_times), 2)
                            if self.warning_times else 0.0),
            "messages_rejected": sum(r.messages_rejected for r in robots),
        }

    # ---------------------------------------------------------------- output

    def snapshot(self, focus: Optional[str] = None) -> Dict[str, object]:
        """Everything the dashboard needs, as plain data."""
        return {
            "sim_time": round(self.sim_time, 2),
            "ticks": self.ticks,
            "planner": "acceleration-aware SIPP",
            "coordination_algorithm": "PIBT",
            "architecture": "DECENTRALIZED_P2P",
            # The map draws each robot's planned route as a short dotted line;
            # past a dozen squares it is off the edge of anything anyone looks
            # at, so there is no point sending the rest twenty times a second.
            "robots": [dict(r.to_dict(self.sim_time),
                            path=[[c.x, c.y] for c in r.path[:12]])
                       for r in self.robots.values()],
            "collision_events": self.collision_events[-12:],
            "conflicts": self.active_conflicts()[:14],
            # Only ONE robot's full picture is ever shown on screen, so only
            # one is sent. Sending all of them made this 78% of a 65 KB
            # payload, twenty times a second -- over a megabyte a second of
            # detail that was thrown away on arrival, and the reason the page
            # crawled with a big fleet.
            #
            # Every robot still contributes `holds`, because the map tints the
            # squares each one has booked -- but that is a handful of numbers.
            "views": {
                rid: (dict(r.fleet.to_dict(self.sim_time),
                           table=r.table.rows(self.sim_time) if r.table else [],
                           holds=self._held_cells(rid, r),
                           blocked=r.blocked_map.to_rows(self.sim_time) if r.blocked_map else [])
                      if rid == focus
                      else {"owner": rid, "neighbours": [], "table": [],
                            "blocked": [], "holds": self._held_cells(rid, r)})
                for rid, r in self.robots.items() if r.fleet
            },
            "stuck": self.stuck_robots(),
            "obstacles": [[c.x, c.y] for c in self.obstacles],
            "tasks": self.board.rows(),
            "wait_graph": (list(self.robots.values())[0].waits.to_rows()
                           if self.robots else []),
            "bus": self.bus.stats() if hasattr(self.bus, "stats") else {},
            "demand": self.demand_view(),
            "reslotting": self.reslotter.to_dict(self.sim_time),
            "humans": [h.to_dict() for h in self.humans.values()],
            "kpis": self.kpis(),
        }


# --------------------------------------------------------------- setups


def phase1_world() -> World:
    """Phase 1: one warehouse, one robot, parked at a pick station.

    Roadmap Phase 1 asks for exactly one thing: R1 can move from A to B.
    """
    world = World()
    # Was the first PICK square, back when the map had some. It starts in the
    # bottom-left corner, which is what the Phase 1 demo has always shown.
    start = Cell(0, 13)
    world.add_robot(Robot(robot_id="R1", cell=start))
    return world


# Places to park robots at the start, spread out so nobody begins in a jam.
FLEET_START_CELLS = [
    Cell(2, 8), Cell(26, 8), Cell(13, 0), Cell(2, 0), Cell(26, 0),
    Cell(2, 4), Cell(26, 4), Cell(13, 4), Cell(2, 12), Cell(26, 12),
    Cell(19, 0), Cell(7, 0), Cell(19, 4), Cell(7, 4), Cell(19, 12),
    Cell(7, 12), Cell(13, 12), Cell(20, 8), Cell(7, 8), Cell(19, 8),
]


def fleet_world(robots: int = 3, grid=None, **kwargs) -> World:
    """A warehouse with however many robots you ask for.

    More robots on the same floor means more contention, which is the whole
    point: coordination only earns its keep when robots actually get in each
    other's way.
    """
    world = World(grid=grid, **kwargs)
    if robots > len(FLEET_START_CELLS):
        raise ValueError(f"No start position for more than {len(FLEET_START_CELLS)} robots.")
    for i in range(robots):
        world.add_robot(Robot(robot_id=f"R{i + 1}", cell=FLEET_START_CELLS[i]))
    return world


def phase2_world() -> World:
    """Phase 2: three robots navigating simultaneously.

    Roadmap Phase 2 asks for: R1, R2, R3 navigating simultaneously, with
    the full decentralized coordination stack (SIPP + PIBT) active.
    """
    world = World()
    world.add_robot(Robot(robot_id="R1", cell=Cell(2, 8)))
    world.add_robot(Robot(robot_id="R2", cell=Cell(26, 8)))
    world.add_robot(Robot(robot_id="R3", cell=Cell(13, 0)))
    return world
