"""One robot's brain.

PURE LOGIC ONLY. No web code, no ROS 2 code. This is THE file both Part 1
(the grid simulator) and Part 2 (the ROS 2 agent) share. If you find yourself
wanting to import a web or ROS 2 library in here, put it in a wrapper instead.

The split that makes the brain reusable
---------------------------------------
  decide()       WHAT to do next  -- thinking. BOTH parts call this.
  advance()      pretend to drive -- ONLY the grid simulator calls this.
  update_pose()  told where I am  -- ONLY the ROS 2 agent calls this,
                                     because there Nav2 drives the wheels
                                     and the real robot reports back.

So the thinking is written once, and only the "how do the wheels turn" bit
differs between simulation and a real robot.
"""

import random
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, Iterable, List, Optional

from .astar import find_path
from .conflicts import (DEFAULT_CLEARANCE, DEFAULT_HORIZON, Conflict, Plan,
                        build_plan, find_conflicts)
from .deadlock import STUCK_SECONDS, WaitForGraph, Waiting, choose_victim
from .fleet_view import FleetView
from .grid import Cell, Grid
from .messages import (BlockedAisle, ConflictAlert, Heartbeat, IntentUpdate,
                       PathReservation, PoseUpdate, TaskAnnounce, TaskBid,
                       TaskClaim, WaitReport, YieldRequest)
from .obstacles import DEFAULT_TTL, BlockedMap
from .tasks import (BID_WINDOW, CLAIM_TIMEOUT, Task, TaskBoard, TaskStatus,
                    auction_winner, bid_cost, bid_rank)
from .priority import (BASE_PRIORITY, effective_priority, next_wait_credit,
                       quantise, yields_to)
from .reservations import (DEFAULT_LOOKAHEAD, OCCUPANCY_PRIORITY, Reservation,
                           ReservationTable, avoidance_cost, edge_key, node_key)


class RobotStatus(str, Enum):
    """The words the dashboard shows. Taken from 07_DASHBOARD §7."""

    IDLE = "IDLE"                  # nothing to do, parked
    MOVING = "MOVING"              # driving along its route
    WAITING = "WAITING"            # holding still on purpose (Phase 6)
    NEGOTIATING = "NEGOTIATING"    # sorting out who goes first (Phase 6)
    REROUTING = "REROUTING"        # picking a different way round (Phase 8)
    YIELDING = "YIELDING"          # shuffling out of somebody's way (Phase 7)
    BLOCKED = "BLOCKED"            # no route exists to the goal
    CHARGING = "CHARGING"          # sitting on a charger (Phase 13)
    FAILED = "FAILED"              # broken / switched off (Phase 9)
    SAFE_MODE = "SAFE_MODE"        # lost the network, moving carefully (Phase 14)


# Which way it is facing, purely so the dashboard can draw a nose on it.
_HEADINGS = {(0, -1): "N", (1, 0): "E", (0, 1): "S", (-1, 0): "W"}


def _heading(from_cell: Cell, to_cell: Cell) -> Optional[str]:
    return _HEADINGS.get((to_cell.x - from_cell.x, to_cell.y - from_cell.y))


@dataclass
class Robot:
    """A single autonomous mobile robot."""

    robot_id: str
    cell: Cell                      # the square it is on (or just left)
    speed: float = 2.5              # squares per second
    battery: float = 100.0          # percent

    goal: Optional[Cell] = None     # where it has been told to go
    path: List[Cell] = field(default_factory=list)   # squares still to visit
    status: RobotStatus = RobotStatus.IDLE
    heading: str = "E"

    # Smooth position for drawing. Sits between cell and the next square,
    # so the robot slides instead of jumping.
    x: float = 0.0
    y: float = 0.0

    # Running totals, used later for the benchmark in Phase 15.
    steps: int = 0
    distance: float = 0.0
    replans: int = 0

    # --- talking to the other robots (Phase 3) ---
    # How important this robot thinks it is, in tenths of a point. Worked out
    # fresh every tick from waiting time and battery (Phase 6).
    priority: int = BASE_PRIORITY
    base_priority: int = BASE_PRIORITY
    task_priority: int = 0          # filled in by Phase 9 task allocation
    fleet: Optional[FleetView] = None     # its notebook on the others
    seq: int = 0                          # message counter, lets others spot losses
    messages_sent: int = 0
    messages_heard: int = 0

    # --- seeing trouble coming (Phase 4) ---
    conflicts: List[Conflict] = field(default_factory=list)
    conflicts_seen: int = 0
    _announced: Dict[tuple, float] = field(default_factory=dict)

    # --- booking squares before driving onto them (Phase 5) ---
    table: Optional[ReservationTable] = None   # its OWN copy of the bookings
    hold: bool = False              # true = do not start onto the next square
    emergency: bool = False         # true = stop NOW, even mid-aisle
    blocked_by: Optional[str] = None
    waiting_since: Optional[float] = None
    wait_time: float = 0.0          # total seconds spent held up
    refusals: int = 0               # times it was refused a square
    grants: int = 0
    _claimed: Dict[tuple, Reservation] = field(default_factory=dict)

    # --- giving way properly (Phase 6) ---
    reroutes: int = 0
    last_decision: Optional[str] = None      # plain words, for the event feed
    _reroute_at: float = -99.0
    _reroute_cooldown_until: float = 0.0
    wait_credit: float = 0.0        # fades slowly, see priority.py

    # --- breaking jams that will never clear themselves (Phase 7) ---
    waits: Optional[WaitForGraph] = None    # who it has heard is stuck behind whom
    yields: int = 0                         # times it stepped out of somebody's way
    asked: int = 0                          # times it asked somebody to move
    _suspended_goal: Optional[Cell] = None  # where it was going before it moved aside
    _resume_at: float = 0.0
    _last_wait_report: Optional[tuple] = None
    _asked_at: Dict[str, float] = field(default_factory=dict)
    _make_way_for: Optional[tuple] = None
    _pending_ask: Optional[tuple] = None
    _saw_rng: object = None         # backoff randomness, seeded per robot

    # --- things that should not be there (Phase 8) ---
    blocked_map: Optional[BlockedMap] = None
    obstacles_found: int = 0
    _pending_blocks: List[tuple] = field(default_factory=list)

    # --- real jobs (Phase 9) ---
    board: Optional[TaskBoard] = None      # the jobs it knows about
    task: Optional[Task] = None            # the one it is doing
    tasks_done: int = 0
    bids_won: int = 0
    _my_bids: Dict[str, float] = field(default_factory=dict)
    _task_notes: List[str] = field(default_factory=list)

    _progress: float = 0.0          # 0..1 of the way to the next square
    _next_heartbeat: float = 0.0
    _next_pose: float = 0.0
    _last_intent_key: Optional[tuple] = None

    def __post_init__(self) -> None:
        self.x = float(self.cell.x)
        self.y = float(self.cell.y)
        if self.fleet is None:
            self.fleet = FleetView(self.robot_id)
        if self.table is None:
            self.table = ReservationTable()
        if self.waits is None:
            self.waits = WaitForGraph()
        if self.blocked_map is None:
            self.blocked_map = BlockedMap()
        if self.board is None:
            self.board = TaskBoard()
        if self._saw_rng is None:
            # Seeded from the name so a benchmark run is repeatable.
            self._saw_rng = random.Random(sum(ord(c) for c in self.robot_id))

    # ------------------------------------------------------------- commands

    def set_goal(self, goal: Optional[Cell]) -> None:
        """Tell the robot where to go. It will work out the route itself.

        If it is already part way along a segment it CANNOT simply forget where
        it was -- a real robot cannot teleport back onto the square behind it.
        So it finishes the step it is on, and plans afresh from there.

        (Without this, changing a robot's destination mid-move jumped it up to
        most of a square backwards, straight through the safety checks.)
        """
        if goal == self.goal:
            return
        self.goal = goal

        if self._progress > 0.0 and self.path:
            self.path = [self.path[0]]      # finish the step already begun
        else:
            self.path = []
            self._progress = 0.0

        if goal is None and self.status is not RobotStatus.FAILED:
            # A broken robot stays broken. Clearing its goal must not quietly
            # bring it back to life -- it did, and a "failed" robot went on to
            # win an auction and collect a parcel.
            self.status = RobotStatus.IDLE

    def clear_goal(self) -> None:
        self.set_goal(None)

    def place(self, cell: Cell) -> None:
        """Put the robot down on a square immediately, forgetting its route.

        Used to set up a test scenario ("both robots start at opposite ends of
        this aisle"). A real robot obviously cannot do this.
        """
        self.cell = cell
        self.x = float(cell.x)
        self.y = float(cell.y)
        self.goal = None
        self.path = []
        self._progress = 0.0
        self.status = RobotStatus.IDLE

    # -------------------------------------------------------------- think

    def decide(self, grid: Grid, cost_fn: Optional[Callable] = None,
               now: float = 0.0) -> None:
        """Work out what to do next. BOTH the simulator and ROS 2 call this.

        It only changes the robot's plan and status. It never moves anything.
        """
        if self.status is RobotStatus.FAILED:
            return

        # Nothing to do.
        if self.goal is None:
            self.path = []
            self.status = RobotStatus.IDLE
            return

        # Arrived. (Only once it is standing still on a square, never mid-step.)
        if self.cell == self.goal and not self.path and self._progress == 0.0:
            self.goal = None
            self.status = RobotStatus.IDLE
            self._progress = 0.0
            return

        # Something has appeared on the route we were following. Tear it up.
        if self.path:
            in_the_way = self.blocked_map.blocks_any(self.path, now)
            if in_the_way is not None:
                self.path = []
                self._progress = 0.0
                self.status = RobotStatus.REROUTING
                self._reroute_at = now

        # Already have a route: keep going, unless we are being held back.
        if self.path:
            if self.status in (RobotStatus.REROUTING, RobotStatus.YIELDING):
                # hold the label briefly so it is visible on the dashboard,
                # then go back to plain MOVING
                if now - self._reroute_at < self.REROUTE_SHOW:
                    return
            if not self.hold:
                self.status = RobotStatus.MOVING
            return

        # Need a route.
        route = find_path(grid, self.cell, self.goal, cost_fn,
                          blocked=self.blocked_map.cells(now))
        if route is None:
            # No way through. Sit still rather than guess.
            self.status = RobotStatus.BLOCKED
            self.path = []
            return

        # route[0] is the square we are already on, so drop it.
        self.path = route[1:]
        self.replans += 1
        self.status = RobotStatus.MOVING if self.path else RobotStatus.IDLE

    # ------------------------------------- drive (grid simulator only)

    def advance(self, dt: float) -> None:
        """Pretend to drive for dt seconds.

        ONLY the grid simulator calls this. On a real robot Nav2 turns the
        wheels and update_pose() is used instead.
        """
        if self.hold and self.emergency:
            # EMERGENCY STOP. Something is in the space ahead, so stop dead --
            # even part way down an aisle. 05_PATH_PLANNING section 12: "the
            # safety controller takes precedence over fleet optimisation."
            #
            # Robots have brakes. Insisting on always finishing the segment
            # already begun is what let two robots, each committed a fraction of
            # a second before hearing about the other, drive into each other.
            self._drain_battery(dt, moving=False)
            return

        if self.hold and self._progress == 0.0:
            # An ordinary hold: it simply does not own the next square yet. No
            # danger, so it waits tidily on the square it is standing on.
            self.x = float(self.cell.x)
            self.y = float(self.cell.y)
            self._drain_battery(dt, moving=False)
            return

        if self.status not in (RobotStatus.MOVING, RobotStatus.WAITING,
                               RobotStatus.REROUTING,
                               RobotStatus.YIELDING) or not self.path:
            self.x = float(self.cell.x)
            self.y = float(self.cell.y)
            self._progress = 0.0
            self._drain_battery(dt, moving=False)
            return

        self._progress += self.speed * dt

        # Cross as many whole squares as this tick allows. A robot always
        # finishes the segment it started -- it stops cleanly ON a square,
        # never halfway down an aisle.
        while self._progress >= 1.0 and self.path:
            self._progress -= 1.0
            nxt = self.path.pop(0)
            direction = _heading(self.cell, nxt)
            if direction:
                self.heading = direction
            self.cell = nxt
            self.steps += 1
            self.distance += 1.0
            if self.hold and self.path:
                self._progress = 0.0
                break

        if self.path:
            # Part-way between the current square and the next one.
            nxt = self.path[0]
            direction = _heading(self.cell, nxt)
            if direction:
                self.heading = direction
            self.x = self.cell.x + (nxt.x - self.cell.x) * self._progress
            self.y = self.cell.y + (nxt.y - self.cell.y) * self._progress
        else:
            # Route finished, sit exactly on the square.
            self._progress = 0.0
            self.x = float(self.cell.x)
            self.y = float(self.cell.y)

        self._drain_battery(dt, moving=True)

    # ------------------------------------------- told where I am (ROS 2 only)

    def update_pose(self, x: float, y: float, cell: Optional[Cell] = None) -> None:
        """A real robot reporting its true position back to the brain.

        ONLY the ROS 2 agent calls this, from the odometry callback.
        """
        self.x = x
        self.y = y
        new_cell = cell if cell is not None else Cell(int(round(x)), int(round(y)))
        if new_cell != self.cell:
            direction = _heading(self.cell, new_cell)
            if direction:
                self.heading = direction
            self.cell = new_cell
            self.steps += 1
            self.distance += 1.0
            # Keep the plan honest: drop squares we have already passed.
            if new_cell in self.path:
                self.path = self.path[self.path.index(new_cell) + 1:]

    # ------------------------------------------------- talking (Phase 3)

    # How often each message goes out, in seconds.
    HEARTBEAT_PERIOD = 0.5      # twice a second
    POSE_PERIOD = 0.1           # ten times a second

    def node_etas(self) -> List[float]:
        """How many seconds until this robot reaches each square on its plan.

        This is the heart of intent sharing. A pose says where it IS. This says
        which squares it is about to occupy and WHEN -- which is what lets
        another robot see a crash coming instead of reacting to one.
        """
        if not self.path or self.speed <= 0:
            return []
        first = (1.0 - self._progress) / self.speed     # time to the next square
        return [first + i / self.speed for i in range(len(self.path))]

    def current_velocity(self) -> float:
        """How fast it is ACTUALLY going, right now.

        Not "how fast could it go" -- a robot that is stopped and waiting must
        report zero. It used to report its full speed whenever it had a route,
        which made every other robot dead-reckon it creeping forward into
        squares it was not moving into. Two stopped robots would then block each
        other forever over a square neither was going to enter.
        """
        if self.hold or not self.path:
            return 0.0
        if self.status in (RobotStatus.FAILED, RobotStatus.IDLE,
                           RobotStatus.WAITING, RobotStatus.CHARGING):
            return 0.0
        return self.speed

    def build_intent(self, now: float) -> IntentUpdate:
        etas = self.node_etas()
        return IntentUpdate(
            robot_id=self.robot_id, timestamp=now, seq=self.seq,
            x=self.x, y=self.y, velocity=self.current_velocity(),
            destination=(self.goal.x, self.goal.y) if self.goal else None,
            planned_nodes=[(c.x, c.y) for c in self.path],
            node_etas=etas,
            eta_destination=etas[-1] if etas else None,
            priority=self.priority, status=self.status.value,
        )

    def communicate(self, bus, now: float) -> None:
        """Listen first, then speak. Called once per tick by the world.

        The listening half is identical on a real robot. The speaking half is
        identical too -- only the bus underneath changes.
        """
        if bus is None:
            return

        # 1. Read the inbox.
        for message in bus.poll(self.robot_id):
            self.fleet.ingest(message, now)
            self._ingest_reservation(message)
            self._ingest_jam_news(message, now)
            self._ingest_blocked_aisle(message, now)
            self._ingest_task_news(message, now)
            self.messages_heard += 1

        if self.status is RobotStatus.FAILED:
            return

        # 2. Heartbeat -- "I am alive", on a fixed drumbeat.
        if now >= self._next_heartbeat:
            self._next_heartbeat = now + self.HEARTBEAT_PERIOD
            self._publish(bus, Heartbeat(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                battery=self.battery, status=self.status.value,
            ))

        # 3. Pose -- "this is where I am", often, because it changes fast.
        if now >= self._next_pose:
            self._next_pose = now + self.POSE_PERIOD
            self._publish(bus, PoseUpdate(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                x=self.x, y=self.y, cell=(self.cell.x, self.cell.y),
                velocity=self.current_velocity(), heading=self.heading,
            ))

        # 4. Intent -- only when the plan actually changes. No point repeating
        #    "still going to the same place by the same route" 20 times a second.
        key = (
            (self.goal.x, self.goal.y) if self.goal else None,
            len(self.path),
            (self.path[0].x, self.path[0].y) if self.path else None,
            self.status.value,
            self.hold,          # stopping or starting is news worth sending
        )
        if key != self._last_intent_key:
            self._last_intent_key = key
            self._publish(bus, self.build_intent(now))

        # 4a. Anything we just found in the way.
        for cell, cleared in self._pending_blocks:
            self._publish(bus, BlockedAisle(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                cell=(cell.x, cell.y), confidence=1.0,
                ttl=self.blocked_map.ttl, cleared=cleared,
            ))
        self._pending_blocks.clear()

        # 4b. Any request we were asked to pass along.
        if self._pending_ask is not None:
            who, side = self._pending_ask
            self._pending_ask = None
            self._publish(bus, YieldRequest(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                target=who, resource=(side.x, side.y), reason="BOXED_IN",
            ))

        # 5. "I am stuck behind X" -- the raw material for spotting a jam that
        #    will never clear itself. Sent when it changes, not constantly.
        report = (self.blocked_by, self.current_velocity() > 0.0)
        if report != self._last_wait_report:
            self._last_wait_report = report
            self._publish(bus, WaitReport(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                blocked_by=self.blocked_by, waiting=self.waited_for(now),
                priority=self.priority, is_moving=self.current_velocity() > 0.0,
            ))

    def _publish(self, bus, message) -> None:
        self.seq += 1
        self.messages_sent += 1
        bus.publish(message)

    # ------------------------------------ seeing trouble coming (Phase 4)

    ALERT_COOLDOWN = 1.0        # don't re-announce the same clash every tick

    def own_plan(self, now: float, horizon: float = DEFAULT_HORIZON) -> Plan:
        """This robot's own bookings: which squares it will hold, and when."""
        return build_plan(
            self.robot_id, self.cell, self.path, self.node_etas(),
            self.speed, horizon=horizon,
        )

    def neighbour_plans(self, now: float, horizon: float = DEFAULT_HORIZON) -> List[Plan]:
        """What this robot BELIEVES everyone else has booked.

        Built from its own notebook, so it is only as good as what it heard.
        A robot whose messages were lost simply is not in this list -- which is
        the honest behaviour, and part of why Phase 6 needs real negotiation.
        """
        plans: List[Plan] = []
        for note in self.fleet.fresh(now):
            nodes = [Cell(n[0], n[1]) for n in note.planned_nodes]
            plans.append(build_plan(
                note.robot_id, Cell(note.cell[0], note.cell[1]),
                nodes, note.node_etas,
                note.velocity if note.velocity > 0 else self.speed,
                horizon=horizon,
                # its ETAs were true when it spoke, not when we heard it
                time_offset=max(0.0, now - note.intent_sent_at),
            ))
        return plans

    def check_conflicts(self, grid: Grid, now: float,
                        horizon: float = DEFAULT_HORIZON) -> List[Conflict]:
        """Compare my plan against everyone else's. Purely a look, no action.

        Phase 5 books the contested square. Phase 6 decides who yields.
        """
        if self.status is RobotStatus.FAILED:
            self.conflicts = []
            return self.conflicts

        self.conflicts = find_conflicts(
            self.own_plan(now, horizon),
            self.neighbour_plans(now, horizon),
            grid, horizon,
        )
        return self.conflicts

    def announce_conflicts(self, bus, now: float) -> None:
        """Tell the fleet what we spotted, without spamming the radio."""
        if bus is None:
            return
        for conflict in self.conflicts:
            key = (conflict.robot_b, conflict.cell.x, conflict.cell.y, conflict.kind.value)
            last = self._announced.get(key)
            if last is not None and now - last < self.ALERT_COOLDOWN:
                continue
            self._announced[key] = now
            self.conflicts_seen += 1
            self._publish(bus, ConflictAlert(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                robot_a=conflict.robot_a, robot_b=conflict.robot_b,
                resource=(conflict.cell.x, conflict.cell.y),
                kind=conflict.kind.value, estimated_time=conflict.lead_time,
            ))
        # forget old announcements so a clash that returns is announced again
        stale_keys = [k for k, t in self._announced.items() if now - t > 6.0]
        for k in stale_keys:
            del self._announced[k]

    # ------------------------------ the STOP-AND-WAIT baseline (Phase 15)

    # How close another robot has to be before a stop-and-wait robot freezes.
    SAW_STOP_RADIUS = 1.6
    # How long two of them stand nose to nose before the lower name goes first.
    # Without something like this they wait for each other for ever and the
    # warehouse dies -- so a real stop-and-wait system would have it, and
    # leaving it out would be rigging the comparison.
    SAW_STANDOFF = 2.0

    # Stuck this long and it backs off to a random free square nearby and tries
    # again. Crude, but it is what a simple system does, and without it the
    # baseline deadlocks permanently and never finishes the work at all -- so
    # there would be no time to compare. Being generous to the baseline makes
    # our own number SMALLER, which is the direction that survives scrutiny.
    SAW_BACKOFF = 5.0

    def stop_and_wait_check(self, now: float, dt: float) -> None:
        """The dumb fleet's only traffic rule: if somebody is close, stop.

        This is the thing FLEET-X is measured against. It has NO intent
        sharing, NO booking, NO priority, NO negotiation and NO rerouting. A
        robot sees another robot near, stops, and waits for it to clear.

        Everything else -- the map, A*, the speed, the job auction -- is
        identical to FLEET-X. The coordination logic is the only difference,
        which is the whole point of the comparison.
        """
        self.blocked_by = None
        if self.status is RobotStatus.FAILED or not self.path:
            self._set_hold(False, now, dt)
            return

        nxt = self.path[0]

        # Exactly the same safety reflex FLEET-X has. Not a coordination
        # feature -- without it the baseline crashes, and a fleet that crashes
        # cannot be compared with one that does not.
        if self._local_safety_says_stop(nxt, now, dt):
            return

        nearest = None
        for note in self.fleet.fresh(now):
            for px, py in ((note.x, note.y), note.position_at(now)):
                for tx, ty in ((nxt.x, nxt.y), (self.x, self.y)):
                    if (px - tx) ** 2 + (py - ty) ** 2 < self.SAW_STOP_RADIUS ** 2:
                        nearest = note
                        break
                if nearest:
                    break
            if nearest:
                break

        if nearest is None:
            self._set_hold(False, now, dt, emergency=False)
            return

        # Nose to nose for a while: the lower name goes first, or neither ever
        # moves again and the warehouse dies. This is the least clever
        # tie-break there is, and a real stop-and-wait system would have
        # something like it -- leaving it out would rig the comparison.
        if (self.waited_for(now) > self.SAW_STANDOFF
                and nearest.velocity <= 0.01
                and self.robot_id < nearest.robot_id):
            self._set_hold(False, now, dt, emergency=False)
            return

        self.blocked_by = nearest.robot_id
        self._set_hold(True, now, dt, emergency=True)

    def stop_and_wait_backoff(self, grid: Grid, now: float) -> Optional[str]:
        """Stuck a long time: shuffle somewhere random and try again.

        Note what this is NOT. There is no intent sharing, no booking, no
        priority, no negotiation and no wait-for graph. The robot does not know
        why it is stuck or who it is stuck behind. It just gives up and moves,
        the way a simple system would.
        """
        if not self.hold or self.goal is None:
            return None
        if self.waited_for(now) < self.SAW_BACKOFF:
            return None
        if now < self._reroute_cooldown_until:
            return None
        self._reroute_cooldown_until = now + self.SAW_BACKOFF

        taken = {Cell(n.cell[0], n.cell[1]) for n in self.fleet.fresh(now)}
        options = [c for c in grid.neighbours(self.cell) if c not in taken]
        if not options:
            return None

        # Retreat AWAY from whoever is in the way, not just anywhere -- and
        # then sit still for a random moment so they have time to get past.
        # Backing off and immediately walking back into the same robot is a
        # weaker baseline than a real system would be, and a weak baseline
        # would make our own result look better than it is.
        blocker = self.fleet.get(self.blocked_by) if self.blocked_by else None
        if blocker is not None:
            options.sort(key=lambda c: -((c.x - blocker.x) ** 2 + (c.y - blocker.y) ** 2))
            spot = options[0]
        else:
            spot = options[self._saw_rng.randrange(len(options))]

        if self._suspended_goal is None:
            self._suspended_goal = self.goal
        self.set_goal(spot)
        self.status = RobotStatus.REROUTING
        self._reroute_at = now
        self._resume_at = now + 1.0 + self._saw_rng.random() * 3.0
        self.reroutes += 1
        return f"{self.robot_id} backed off to ({spot.x}, {spot.y}) - stuck too long"

    # --------------------------------------- booking squares (Phase 5)

    LOOKAHEAD = DEFAULT_LOOKAHEAD     # how many squares ahead to book
    CLEARANCE = 0.35                  # a little clear time either side
    SAFE_GAP = 0.9                    # squares of clear space needed ahead
    CONTEST_RADIUS = 1.2              # this close to my target square = a contest
    INTENT_TRUST = 0.7                # older news than this is not trusted
    POSE_TRUST = 0.35                 # ...same, for positions

    def _ingest_reservation(self, message) -> None:
        """File somebody else's booking into our own copy of the table."""
        if not isinstance(message, PathReservation):
            return
        cells = [Cell(c[0], c[1]) for c in message.cells]
        key = node_key(cells[0]) if message.kind == "NODE" else edge_key(cells[0], cells[1])
        if message.action == "RELEASE":
            self.table.release(message.robot_id, key)
        else:
            self.table.put(Reservation(
                robot_id=message.robot_id, resource=key,
                start=message.start, end=message.end, priority=message.priority,
            ))

    def _wanted_slots(self, now: float) -> Dict[tuple, tuple]:
        """Which squares and aisle segments this robot needs, and for how long.

        Each entry is (start, end, priority).

        Only the next few squares. Booking the whole route would let one robot
        own half the warehouse while everybody else sat still.
        """
        wanted: Dict[tuple, tuple] = {}
        dwell = (1.0 / self.speed) if self.speed > 0 else 1.0
        etas = self.node_etas()

        # The square it is physically standing on, at OCCUPANCY_PRIORITY.
        # Nothing outranks this. A robot cannot be booked out of the square its
        # body is sitting in -- that is a fact about the world, not a request.
        leave_current = now + (etas[0] if etas else dwell) + self.CLEARANCE
        wanted[node_key(self.cell)] = (now - self.CLEARANCE, leave_current,
                                       OCCUPANCY_PRIORITY)

        # If it has already set off down a segment it cannot stop halfway, so
        # the square it is entering is occupied in practice. Claim it at the
        # same unbeatable priority. Without this, two robots can commit to the
        # same square from opposite ends and meet in the middle -- the gate
        # only bites at a square boundary, and by then it is too late.
        if self._progress > 0.0 and self.path:
            entering = self.path[0]
            wanted[node_key(entering)] = (now - self.CLEARANCE,
                                          now + etas[0] + dwell + self.CLEARANCE,
                                          OCCUPANCY_PRIORITY)

        chain = [self.cell] + list(self.path)
        for i in range(min(self.LOOKAHEAD, len(self.path))):
            node = self.path[i]
            enter = now + etas[i] - self.CLEARANCE
            leave = now + (etas[i + 1] if i + 1 < len(etas) else etas[i] + dwell) + self.CLEARANCE
            wanted[node_key(node)] = (enter, leave, self.priority)
            # the aisle segment used to get there -- booked in BOTH directions,
            # which is what stops a head-on
            prev = chain[i]
            edge_start = now + (etas[i - 1] if i > 0 else 0.0) - self.CLEARANCE
            wanted[edge_key(prev, node)] = (edge_start, now + etas[i] + self.CLEARANCE,
                                            self.priority)
        return wanted

    def reserve_ahead(self, bus, now: float) -> None:
        """Claim what we need, give back what we no longer do, tell everyone."""
        if self.status is RobotStatus.FAILED:
            # A broken robot must not keep squares booked, or the rest of the
            # fleet drives around holes that nobody is in.
            # 03_ROBOT_AND_ROS2 section 5: release its future reservations.
            self.release_all(bus, now)
            return

        wanted = self._wanted_slots(now)

        # give back anything we have moved past
        for key in list(self._claimed.keys()):
            if key in wanted:
                continue
            self.table.release(self.robot_id, key)
            old = self._claimed.pop(key)
            self._broadcast_reservation(bus, now, "RELEASE", old)

        for key, (start, end, priority) in wanted.items():
            existing = self._claimed.get(key)
            # only re-broadcast when the slot has actually moved
            if (existing and existing.priority == priority
                    and abs(existing.start - start) < 0.2 and abs(existing.end - end) < 0.2):
                continue
            res = Reservation(robot_id=self.robot_id, resource=key,
                              start=start, end=end, priority=priority)
            self.table.put(res)
            self._claimed[key] = res
            self._broadcast_reservation(bus, now, "CLAIM", res)

        self.table.prune(now)

    def forget_bookings(self) -> None:
        """Throw away every booking, ours and everyone else's.

        Used when the clock is restarted. Bookings hold absolute times, so any
        that survive a clock reset are nonsense -- they would claim squares for
        a time that will never come round again.
        """
        self.table = ReservationTable()
        self._claimed.clear()
        self.hold = False
        self.emergency = False
        self.blocked_by = None
        self.waiting_since = None
        self.wait_credit = 0.0
        self.priority = self.base_priority
        self._next_heartbeat = 0.0
        self._next_pose = 0.0
        self._last_intent_key = None
        self._reroute_at = -99.0
        self._reroute_cooldown_until = 0.0
        self._announced.clear()
        self.waits = WaitForGraph()
        self._suspended_goal = None
        self._resume_at = 0.0
        self._last_wait_report = None
        self._asked_at.clear()
        self._make_way_for = None
        self._pending_ask = None
        self.blocked_map = BlockedMap()
        self._pending_blocks.clear()
        self.board = TaskBoard()
        self.task = None
        self._my_bids.clear()
        self._task_notes.clear()
        self.fleet = FleetView(self.robot_id, self.fleet.stale_after)

    def release_all(self, bus, now: float) -> None:
        """Give every booking back. Used when a robot fails or is switched off."""
        for key, res in list(self._claimed.items()):
            self.table.release(self.robot_id, key)
            del self._claimed[key]
            if bus is not None and self.status is not RobotStatus.FAILED:
                self._broadcast_reservation(bus, now, "RELEASE", res)
        self.hold = False
        self.blocked_by = None
        self.waiting_since = None

    def _broadcast_reservation(self, bus, now: float, action: str, res: Reservation) -> None:
        if bus is None:
            return
        if res.resource[0] == "N":
            kind, cells = "NODE", [(res.resource[1], res.resource[2])]
        else:
            kind = "EDGE"
            cells = [(res.resource[1], res.resource[2]), (res.resource[3], res.resource[4])]
        self._publish(bus, PathReservation(
            robot_id=self.robot_id, timestamp=now, seq=self.seq,
            action=action, kind=kind, cells=cells,
            start=res.start, end=res.end, priority=res.priority,
        ))

    def check_clearance(self, now: float, dt: float) -> None:
        """May we drive onto the next square? THE rule that stops crashes.

        A robot may not enter a square it does not own. If somebody outranks it,
        it stops before the square and waits.

        Note it keeps holding the square it is standing on while it waits. That
        is correct -- and it is exactly how two robots end up waiting for each
        other forever. Phase 6 lets the loser reroute, Phase 7 breaks the
        remaining deadlocks.
        """
        self.blocked_by = None

        if self.status is RobotStatus.FAILED or not self.path:
            self._set_hold(False, now, dt)
            return

        nxt = self.path[0]

        # Something is physically in that square. Nothing else matters.
        if self.blocked_map.is_blocked(nxt, now):
            self.blocked_by = "obstacle"
            self._set_hold(True, now, dt, emergency=True)
            return

        if self._local_safety_says_stop(nxt, now, dt):
            return

        checks = [
            (node_key(nxt), self._claimed.get(node_key(nxt))),
            (edge_key(self.cell, nxt), self._claimed.get(edge_key(self.cell, nxt))),
        ]
        for key, mine in checks:
            if mine is None:
                continue
            other = self.table.blocked_by(self.robot_id, key, mine.start, mine.end)
            if other is not None:
                self.blocked_by = other
                self._set_hold(True, now, dt)
                return

        self._set_hold(False, now, dt)

    def _local_safety_says_stop(self, nxt: Cell, now: float, dt: float) -> bool:
        """The reflex that stops a robot driving into an occupied space.

        BOTH fleets have this, identically. 03_ROBOT_AND_ROS2 §8: "The local
        safety layer must have authority to stop/slow the robot" and "should
        not wait for the dashboard" -- on real hardware it is the LiDAR, which
        needs no messages at all. Taking it away from the baseline would not
        make the baseline dumber, it would make it crash, and a fleet that
        crashes is not a comparison.

        What FLEET-X adds ON TOP of this -- intent sharing, booking,
        negotiation, rerouting, deadlock breaking -- is what the benchmark
        actually measures.

        Checking POSITION rather than which square somebody is registered on
        matters. Deciding takes an instant but driving takes time, so two
        robots approaching one square from opposite ends can each be cleared at
        a different moment and meet in the middle. Measuring the real gap
        closes that hole.
        """
        for note in self.fleet.fresh(now):
            # Where it was when it last spoke, AND where it has probably got to
            # since. Checking only the first was how a robot already part way
            # into a square looked like it was still safely outside it.
            here = (note.x, note.y)
            soon = note.position_at(now)
            for px, py in (here, soon):
                gap_x, gap_y = px - nxt.x, py - nxt.y
                if gap_x * gap_x + gap_y * gap_y < self.SAFE_GAP * self.SAFE_GAP:
                    self.blocked_by = note.robot_id
                    self._set_hold(True, now, dt, emergency=True)
                    return True

            # Anything sitting next to the square I am about to enter could get
            # there before me, so treat it as a contest.
            #
            # Being CAUTIOUS WHEN IN DOUBT is the whole point. Two robots two
            # squares apart, both aiming at the gap between them, can set off in
            # the very same instant -- before either has heard the other's plan.
            # Waiting for proof would be waiting for a message that has not
            # arrived yet. So the rule is: unless I have recent news that this
            # robot is going somewhere else, assume it wants my square.
            gap_x, gap_y = soon[0] - nxt.x, soon[1] - nxt.y
            if gap_x * gap_x + gap_y * gap_y >= self.CONTEST_RADIUS ** 2:
                continue

            # ...unless it is parked, and we know that recently. Parked robots
            # cannot move into my square, and yielding to one forever would be
            # a deadlock of my own making.
            pose_fresh = (now - note.last_pose) <= self.POSE_TRUST
            if pose_fresh and note.velocity <= 0.01:
                continue

            intent_fresh = ((now - note.last_intent) <= self.INTENT_TRUST
                            and note.next_cell is not None)
            if intent_fresh and note.next_cell != (nxt.x, nxt.y):
                continue                       # we know it is going elsewhere

            # Settled WITHOUT the booking table, because the table is exactly
            # what the two of them can disagree about. yields_to() always gives
            # the two robots opposite answers, so it can never let both through,
            # and can never stop both either.
            if yields_to(self.priority, self.robot_id, note.priority, note.robot_id):
                self.blocked_by = note.robot_id
                self._set_hold(True, now, dt, emergency=True)
                return True

        return False

    def _set_hold(self, hold: bool, now: float, dt: float,
                  emergency: bool = False) -> None:
        self.emergency = hold and emergency
        if hold:
            if not self.hold:
                self.refusals += 1
                self.waiting_since = now
            self.wait_time += dt
            self.hold = True
            if self.status is RobotStatus.MOVING:
                self.status = RobotStatus.WAITING
        else:
            if self.hold:
                self.grants += 1
            self.hold = False
            self.waiting_since = None
            if self.status is RobotStatus.WAITING and self.path:
                self.status = RobotStatus.MOVING

    def waited_for(self, now: float) -> float:
        """How long this robot has been stuck right now."""
        return 0.0 if self.waiting_since is None else max(0.0, now - self.waiting_since)

    # ------------------------------------- giving way properly (Phase 6)

    PATIENCE = 1.5              # wait this long before even considering a detour
    REROUTE_COOLDOWN = 2.5      # do not keep re-deciding every tick
    REROUTE_SHOW = 0.8          # how long the REROUTING label stays up
    DETOUR_SLACK = 4            # squares of detour we accept for free
    DETOUR_FACTOR = 2.5         # ...plus this multiple of what is left

    def update_priority(self, now: float, dt: float) -> None:
        """Recompute standing. The longer we wait, the more we are owed."""
        self.wait_credit = next_wait_credit(self.wait_credit, self.hold, dt)
        # Rounded to a whole point so the number other robots hear steps up
        # cleanly instead of drifting every tick.
        self.priority = quantise(effective_priority(
            base=self.base_priority,
            waiting=self.wait_credit,
            battery=self.battery,
            task_priority=self.task_priority,
        ))

    def consider_reroute(self, grid: Grid, now: float) -> bool:
        """Held up: wait it out, or take the long way round?

        04_DECENTRALIZED_FLEET_PROTOCOL section 5:
            compare priorities -> can we reserve? -> NO -> negotiate
            -> wait OR reroute -> broadcast decision

        Patience first. Most hold-ups clear on their own, and a robot that
        recalculated its route 20 times a second would just flap about.
        """
        if not self.hold or not self.path or self.goal is None:
            return False
        if self.status is RobotStatus.FAILED:
            return False
        if self.waited_for(now) < self.PATIENCE:
            return False                       # be patient first
        if now < self._reroute_cooldown_until:
            return False

        # Only the robot that LOSES the argument should go around. Without
        # this, both of them politely detour, meet again, detour again, and
        # dance around each other indefinitely -- each covering a lot of ground
        # and neither getting to where it was going.
        #
        # A robot stuck far longer than normal tries anyway: better an
        # unnecessary detour than sitting there for ever.
        desperate = self.waited_for(now) >= self.PATIENCE * 3
        if not desperate and self.blocked_by:
            other = self.fleet.get(self.blocked_by)
            if other is not None and not yields_to(
                    self.priority, self.robot_id, other.priority, other.robot_id):
                self.last_decision = (
                    f"{self.robot_id} holds its line - {self.blocked_by} should go around"
                )
                return False

        self._reroute_cooldown_until = now + self.REROUTE_COOLDOWN

        # Squares that physically have a robot standing on them right now.
        # Those are near-impassable, not merely expensive.
        occupied = [Cell(n.cell[0], n.cell[1]) for n in self.fleet.fresh(now)]

        cost_fn = avoidance_cost(
            self.table, self.robot_id, now,
            avoid_robot=self.blocked_by, occupied=occupied,
        )
        alternative = find_path(grid, self.cell, self.goal, cost_fn,
                                blocked=self.blocked_map.cells(now))

        if alternative is None or len(alternative) < 2:
            self.last_decision = f"{self.robot_id} waits - no way round"
            return False

        new_path = alternative[1:]

        # Same first step means it is the same plan; nothing gained.
        if new_path[0] == self.path[0]:
            self.last_decision = f"{self.robot_id} waits - every route goes the same way"
            return False

        # Is the detour worth it? Going 40 squares to save 2 is silly.
        budget = len(self.path) * self.DETOUR_FACTOR + self.DETOUR_SLACK
        if len(new_path) > budget:
            self.last_decision = (
                f"{self.robot_id} waits - detour too long "
                f"({len(new_path)} vs {len(self.path)} squares)"
            )
            return False

        extra = len(new_path) - len(self.path)
        self.last_decision = (
            f"{self.robot_id} gave way to {self.blocked_by or 'traffic'} - "
            f"going around ({extra:+d} squares)"
        )
        self.path = new_path
        self.replans += 1
        self.reroutes += 1
        self.status = RobotStatus.REROUTING
        self._reroute_at = now
        # Its old bookings are stale now. Keep it still for one tick; the next
        # reserve_ahead claims the new route before it moves.
        self.waiting_since = None
        return True

    # ------------------------------------- real jobs (Phase 9)

    def _ingest_task_news(self, message, now: float) -> None:
        """Keep our own copy of the job board up to date."""
        if isinstance(message, TaskAnnounce):
            task = self.board.get(message.task_id)
            if task is None:
                self.board.add(Task(
                    task_id=message.task_id,
                    pickup=Cell(*message.pickup), dropoff=Cell(*message.dropoff),
                    product=message.product, priority=message.priority,
                    created_at=now, announced_at=now,
                    status=TaskStatus.ANNOUNCED,
                ))
            elif task.status is TaskStatus.QUEUED:
                task.status = TaskStatus.ANNOUNCED
                task.announced_at = now

        elif isinstance(message, TaskBid):
            task = self.board.get(message.task_id)
            if task is not None and task.open_for_bids:
                task.bids[message.robot_id] = message.cost

        elif isinstance(message, TaskClaim):
            task = self.board.get(message.task_id)
            if task is None:
                return
            if message.action == "CLAIM":
                # Somebody says they have it. If we thought it was ours and
                # their bid is better, let it go -- two robots doing one job is
                # wasteful, and the rule says the cheaper bid wins.
                if (task.assigned_robot == self.robot_id
                        and self.task is not None
                        and self.task.task_id == task.task_id):
                    mine = self._my_bids.get(task.task_id, float("inf"))
                    if bid_rank(message.cost, message.robot_id) < bid_rank(mine, self.robot_id):
                        self._task_notes.append(
                            f"{self.robot_id} gave up {task.task_id} - "
                            f"{message.robot_id} bid lower")
                        self._drop_task()
                    else:
                        return
                task.assigned_robot = message.robot_id
                task.status = TaskStatus.ASSIGNED
                task.claimed_at = now
                task.winning_bid = message.cost
            elif message.action == "PICKED":
                task.status = TaskStatus.CARRYING
                task.picked_at = now
            elif message.action == "DONE":
                task.status = TaskStatus.DONE
                task.done_at = now
                task.assigned_robot = message.robot_id
            elif message.action == "RELEASE":
                if task.assigned_robot == message.robot_id:
                    task.release()

    def work_on_tasks(self, grid: Grid, bus, now: float) -> List[str]:
        """Bid for jobs, claim what we win, and get on with it."""
        notes = self._task_notes
        self._task_notes = []

        if self.status is RobotStatus.FAILED:
            if self.task is not None:
                self._drop_task()
            return notes

        # A peer has gone quiet while holding a job. Put it back up for
        # auction -- 06_TASK_ALLOCATION section 4. Whoever notices first does
        # it; a repeat is harmless because the job keeps its id.
        for note in self.fleet.stale(now):
            for task in self.board.held_by(note.robot_id):
                task.release()
                notes.append(f"{note.robot_id} has gone quiet - "
                             f"{task.task_id} back up for auction")
                if bus is not None:
                    self._publish(bus, TaskClaim(
                        robot_id=note.robot_id, timestamp=now, seq=self.seq,
                        task_id=task.task_id, action="RELEASE"))

        if self.task is not None:
            notes += self._progress_task(grid, bus, now)
            return notes

        notes += self._bid_and_claim(grid, bus, now)
        return notes

    def _reopen_stale_auctions(self, now: float) -> None:
        """Put a job back up for auction if nobody ever claimed it.

        This happens all the time: three jobs go out at once, one robot has the
        best bid on two of them, takes the first, and the second is left with a
        winning bid naming a robot that is already busy. Without this the job
        sits there for ever, and it did.
        """
        for task in self.board.open_tasks():
            if task.assigned_robot is not None:
                continue
            if now - task.announced_at <= CLAIM_TIMEOUT:
                continue
            task.bids.clear()
            task.announced_at = now
            self._my_bids.pop(task.task_id, None)

    def _bid_and_claim(self, grid: Grid, bus, now: float) -> List[str]:
        notes: List[str] = []
        self._reopen_stale_auctions(now)
        for task in self.board.open_tasks():
            if task.task_id not in self._my_bids:
                cost = self.cost_of(task, grid, now)
                if cost is None:
                    continue                  # cannot reach it, so do not bid
                self._my_bids[task.task_id] = cost
                task.bids[self.robot_id] = cost
                if bus is not None:
                    self._publish(bus, TaskBid(
                        robot_id=self.robot_id, timestamp=now, seq=self.seq,
                        task_id=task.task_id, cost=cost))
                continue

            # Auction has been open long enough. Did we win it?
            if now - task.announced_at < BID_WINDOW:
                continue
            if auction_winner(task.bids) != self.robot_id:
                continue

            self._take_task(task, now)
            self.bids_won += 1
            others = ", ".join(f"{r} {c:.0f}" for r, c in sorted(task.bids.items()))
            notes.append(f"{task.task_id} auction - {others} -> {self.robot_id} wins")
            if bus is not None:
                self._publish(bus, TaskClaim(
                    robot_id=self.robot_id, timestamp=now, seq=self.seq,
                    task_id=task.task_id, action="CLAIM",
                    cost=self._my_bids.get(task.task_id, 0.0)))
            break                              # one job at a time
        return notes

    def cost_of(self, task: Task, grid: Grid, now: float) -> Optional[float]:
        """What this job would cost us. None if we simply cannot reach it."""
        blocked = self.blocked_map.cells(now)
        to_pickup = find_path(grid, self.cell, task.pickup, blocked=blocked)
        if to_pickup is None:
            return None
        leg = find_path(grid, task.pickup, task.dropoff, blocked=blocked)
        if leg is None:
            return None
        congestion = float(len(blocked)) + len(self.fleet.fresh(now)) * 0.5
        return bid_cost(
            distance_to_pickup=len(to_pickup) - 1,
            leg_distance=len(leg) - 1,
            battery=self.battery,
            waiting=self.waited_for(now),
            congestion=congestion,
            task_priority=task.priority,
        )

    def _take_task(self, task: Task, now: float) -> None:
        task.assigned_robot = self.robot_id
        task.status = TaskStatus.ASSIGNED
        task.claimed_at = now
        task.winning_bid = self._my_bids.get(task.task_id)
        self.task = task
        self.set_goal(task.pickup)

    def _drop_task(self) -> None:
        if self.task is not None:
            self._my_bids.pop(self.task.task_id, None)
        self.task = None
        self.clear_goal()

    def _progress_task(self, grid: Grid, bus, now: float) -> List[str]:
        """Collect it, then deliver it."""
        notes: List[str] = []
        task = self.task

        if task.status is TaskStatus.ASSIGNED:
            if self.cell == task.pickup and self._progress == 0.0:
                task.status = TaskStatus.CARRYING
                task.picked_at = now
                self.set_goal(task.dropoff)
                notes.append(f"{self.robot_id} collected {task.task_id} "
                             f"at ({task.pickup.x}, {task.pickup.y})")
                if bus is not None:
                    self._publish(bus, TaskClaim(
                        robot_id=self.robot_id, timestamp=now, seq=self.seq,
                        task_id=task.task_id, action="PICKED"))
            elif (self.goal is None and self.status is not RobotStatus.BLOCKED
                  and self._suspended_goal is None):
                self.set_goal(task.pickup)     # nudged aside, get back to it

        elif task.status is TaskStatus.CARRYING:
            if self.cell == task.dropoff and self._progress == 0.0:
                task.status = TaskStatus.DONE
                task.done_at = now
                self.tasks_done += 1
                notes.append(f"{self.robot_id} delivered {task.task_id} "
                             f"to ({task.dropoff.x}, {task.dropoff.y})")
                if bus is not None:
                    self._publish(bus, TaskClaim(
                        robot_id=self.robot_id, timestamp=now, seq=self.seq,
                        task_id=task.task_id, action="DONE"))
                self.task = None
                self._my_bids.pop(task.task_id, None)
            elif (self.goal is None and self.status is not RobotStatus.BLOCKED
                  and self._suspended_goal is None):
                self.set_goal(task.dropoff)

        return notes

    # -------------------------- things in the way (Phase 8)

    def _ingest_blocked_aisle(self, message, now: float) -> None:
        """Somebody else found something. Believe them, for a while."""
        if not isinstance(message, BlockedAisle):
            return
        cell = Cell(message.cell[0], message.cell[1])
        if message.cleared:
            self.blocked_map.clear(cell)
        else:
            self.blocked_map.mark(cell, message.robot_id, now,
                                  ttl=message.ttl, confidence=message.confidence)

    def sense(self, blocked_now: Iterable[Cell], clear_now: Iterable[Cell],
              now: float) -> List[str]:
        """What the robot's own sensors can see right now.

        The simulator plays the part of the laser scanner: it hands over only
        what is physically within range. That is the whole point -- a robot has
        to GO somewhere to find out what is there. If every robot magically
        knew the instant a box landed, none of this would be decentralised.

        On real hardware this same method is fed by the LiDAR, and nothing
        else in the brain changes.
        """
        news: List[str] = []

        for cell in blocked_now:
            fresh = self.blocked_map.mark(cell, self.robot_id, now)
            if fresh:
                self.obstacles_found += 1
                self._pending_blocks.append((cell, False))
                news.append(f"{self.robot_id} found something blocking "
                            f"({cell.x}, {cell.y}) - telling the fleet")

        # Squares we can see plainly, with nothing in them. If we had one down
        # as blocked, that is out of date -- somebody moved the box.
        for cell in clear_now:
            if self.blocked_map.clear(cell):
                self._pending_blocks.append((cell, True))
                news.append(f"{self.robot_id} sees ({cell.x}, {cell.y}) is "
                            f"clear again - telling the fleet")

        for cell in self.blocked_map.expire(now):
            news.append(f"block on ({cell.x}, {cell.y}) expired - "
                        f"nobody has seen it lately")

        return news

    def blocked_cells(self, now: float):
        return self.blocked_map.cells(now)

    # ------------------------------ breaking a hopeless jam (Phase 7)

    ASK_COOLDOWN = 3.0          # do not pester the same robot every tick
    ASIDE_PAUSE = 2.5           # stay out of the way this long before resuming

    def _ingest_jam_news(self, message, now: float) -> None:
        """File what we hear about who is stuck, and answer requests to move."""
        if isinstance(message, WaitReport):
            if message.blocked_by:
                self.waits.add(Waiting(
                    robot_id=message.robot_id, blocked_by=message.blocked_by,
                    since=message.waiting, priority=message.priority,
                    is_moving=message.is_moving,
                ))
            else:
                self.waits.clear(message.robot_id)
                self.waits.add(Waiting(
                    robot_id=message.robot_id, blocked_by="",
                    since=0.0, priority=message.priority,
                    is_moving=message.is_moving,
                ))
                self.waits._blocked_by.pop(message.robot_id, None)

        elif isinstance(message, YieldRequest) and message.target == self.robot_id:
            self._make_way_for = (message.robot_id, Cell(*message.resource), now)

    def report_jam(self, grid: Grid, bus, now: float) -> Optional[str]:
        """Have I been stuck so long that this will never clear itself?

        Two things can be wrong, and they need different answers:

          * the robot in front is PARKED -- it is not waiting for anything, so
            it will never move on its own. Ask it to shift.
          * we are in a CIRCULE of robots each waiting for the next. Somebody
            has to give up, chosen so that everyone picks the same one.
        """
        if self.blocked_by is None or self.waited_for(now) < STUCK_SECONDS:
            return None
        blocker = self.blocked_by

        last_asked = self._asked_at.get(blocker)
        if last_asked is not None and now - last_asked < self.ASK_COOLDOWN:
            return None

        # Our own edge is not in the graph -- the graph is built from what
        # OTHER robots said. Add ourselves, or the search starts nowhere and a
        # simple two-robot standoff is invisible.
        self.waits.add(Waiting(
            robot_id=self.robot_id, blocked_by=blocker,
            since=self.waited_for(now), priority=self.priority, is_moving=False,
        ))
        cycle = self.waits.cycle_containing(self.robot_id)
        if cycle:
            members = []
            for rid in cycle:
                if rid == self.robot_id:
                    members.append((rid, self.priority, self.waited_for(now)))
                else:
                    info = self.waits.info(rid)
                    members.append((rid, info.priority if info else BASE_PRIORITY,
                                    info.since if info else 0.0))
            victim = choose_victim(members)
            if victim == self.robot_id:
                # Our turn to give up. Step aside, remember where we were going.
                if self.step_aside(grid, now, [], reason="cycle"):
                    return (f"{self.robot_id} broke a deadlock "
                            f"({' -> '.join(cycle)}) by moving aside")
                return None
            self._asked_at[victim] = now
            self.asked += 1
            self._ask_to_move(bus, now, victim, "CYCLE")
            return (f"{self.robot_id} stuck in a loop "
                    f"({' -> '.join(cycle)}) - asked {victim} to move")

        if self.waits.is_parked(blocker):
            self._asked_at[blocker] = now
            self.asked += 1
            self._ask_to_move(bus, now, blocker, "PARKED")
            return f"{self.robot_id} asked {blocker} to move - it is parked in the way"

        return None

    def _ask_to_move(self, bus, now: float, target: str, reason: str) -> None:
        if bus is None or not self.path:
            return
        wanted = self.path[0]
        self._publish(bus, YieldRequest(
            robot_id=self.robot_id, timestamp=now, seq=self.seq,
            target=target, resource=(wanted.x, wanted.y), reason=reason,
        ))

    def answer_requests(self, grid: Grid, now: float) -> Optional[str]:
        """Somebody asked us to move. Be a good neighbour about it."""
        pending = getattr(self, "_make_way_for", None)
        if pending is None:
            return None
        asker, wanted, asked_at = pending
        self._make_way_for = None
        if now - asked_at > 3.0:
            return None                          # stale request, ignore
        if self.step_aside(grid, now, [wanted], reason="asked"):
            return (f"{self.robot_id} stepped aside for {asker} "
                    f"-> ({self.goal.x}, {self.goal.y})")

        # Boxed in: every way out has somebody in it. Refusing would leave the
        # queue stuck for ever, so pass the request along to whoever is
        # blocking US -- anyone except the robot that just asked.
        return self._pass_the_request_along(grid, now, asker, wanted)

    def _pass_the_request_along(self, grid: Grid, now: float, asker: str,
                                wanted: Cell) -> Optional[str]:
        neighbours_by_cell = {(n.cell[0], n.cell[1]): n.robot_id
                              for n in self.fleet.fresh(now)}
        for side in grid.neighbours(self.cell):
            who = neighbours_by_cell.get((side.x, side.y))
            if who is None or who == asker:
                continue
            last = self._asked_at.get(who)
            if last is not None and now - last < self.ASK_COOLDOWN:
                continue
            self._asked_at[who] = now
            self.asked += 1
            self._pending_ask = (who, side)
            return (f"{self.robot_id} is boxed in - passing {asker}'s request "
                    f"on to {who}")
        return None

    def step_aside(self, grid: Grid, now: float, avoid: List[Cell],
                   reason: str = "asked") -> bool:
        """Shuffle out of the way, remembering where we were going.

        Remembering matters. Breaking a jam by wandering off and forgetting the
        job would just swap one problem for another.
        """
        blocked = set(avoid)
        for note in self.fleet.fresh(now):
            blocked.add(Cell(note.cell[0], note.cell[1]))
        if self.path:
            blocked.add(self.path[0])

        blocked |= self.blocked_map.cells(now)
        options = [c for c in grid.neighbours(self.cell) if c not in blocked]
        if not options:
            return False

        # Prefer somewhere out of the traffic: furthest from whoever we are
        # getting out of the way of.
        anchor = avoid[0] if avoid else (self.path[0] if self.path else self.cell)
        options.sort(key=lambda c: -((c.x - anchor.x) ** 2 + (c.y - anchor.y) ** 2))
        spot = options[0]

        if self.goal is not None and self._suspended_goal is None:
            self._suspended_goal = self.goal
        self.forget_bookings_soft()
        self.set_goal(spot)
        self.status = RobotStatus.YIELDING
        self._reroute_at = now
        self._resume_at = now + self.ASIDE_PAUSE
        self.yields += 1
        return True

    def forget_bookings_soft(self) -> None:
        """Drop our own claims, keeping what we know about everyone else's."""
        for key in list(self._claimed.keys()):
            self.table.release(self.robot_id, key)
        self._claimed.clear()

    def resume_after_yielding(self, now: float) -> Optional[str]:
        """Once out of the way and the coast is clear, carry on with the job."""
        if self._suspended_goal is None or self.goal is not None:
            return None
        if now < self._resume_at:
            return None
        goal = self._suspended_goal
        self._suspended_goal = None
        self.set_goal(goal)
        return f"{self.robot_id} back on the job -> ({goal.x}, {goal.y})"

    # ------------------------------------------------------------- internals

    def _drain_battery(self, dt: float, moving: bool) -> None:
        rate = 0.35 if moving else 0.05   # percent per second
        self.battery = max(0.0, self.battery - rate * dt)

    # ---------------------------------------------------------------- output

    def to_dict(self, now: float = 0.0) -> Dict[str, object]:
        """Plain data about this robot. Dicts and numbers only -- no web code."""
        return {
            "robot_id": self.robot_id,
            "cell": [self.cell.x, self.cell.y],
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "heading": self.heading,
            "status": self.status.value,
            "battery": round(self.battery, 1),
            "goal": [self.goal.x, self.goal.y] if self.goal else None,
            "path": [[c.x, c.y] for c in self.path],
            "steps": self.steps,
            "distance": round(self.distance, 1),
            "replans": self.replans,
            "priority": self.priority,
            "messages_sent": self.messages_sent,
            "messages_heard": self.messages_heard,
            "conflicts": [c.to_dict() for c in self.conflicts],
            "blocked_by": self.blocked_by,
            "reroutes": self.reroutes,
            "last_decision": self.last_decision,
            "yields": self.yields,
            "asked": self.asked,
            "obstacles_found": self.obstacles_found,
            "task": self.task.task_id if self.task else None,
            "task_status": self.task.status.value if self.task else None,
            "tasks_done": self.tasks_done,
            "suspended_goal": ([self._suspended_goal.x, self._suspended_goal.y]
                               if self._suspended_goal else None),
            "waiting": round(self.waited_for(now), 2),
            "wait_time": round(self.wait_time, 1),
            "refusals": self.refusals,
        }
