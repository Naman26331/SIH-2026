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
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, Iterable, List, Optional

from .astar import find_space_time_path
from .conflicts import (DEFAULT_CLEARANCE, DEFAULT_HORIZON, Conflict, Plan,
                        build_plan, find_conflicts)
from .deadlock import STUCK_SECONDS, WaitForGraph, Waiting, choose_victim
from .demand import DemandModel
from .health import CRITICAL as HEALTH_CRITICAL
from .health import BANDS, HEALTHY, INCIDENT_WINDOW
from .health import HealthReport, compute as compute_health
from .humans import Human
from . import security
from .fleet_view import FleetView
from .grid import Cell, CellKind, Grid
from .messages import (BlockedAisle, CentralCommand, ConflictAlert, Heartbeat,
                       IntentUpdate, PathReservation, PoseUpdate, TaskAnnounce,
                       TaskBid, TaskClaim, WaitReport, YieldRequest)
from .obstacles import DEFAULT_TTL, BlockedMap
from .tasks import (BID_WINDOW, CLAIM_TIMEOUT, Task, TaskBoard, TaskStatus,
                    auction_winner, bid_cost, bid_rank)
from .priority import (BASE_PRIORITY, SCALE, effective_priority,
                       next_wait_credit, quantise, yields_to)
from .reservations import (DEFAULT_LOOKAHEAD, OCCUPANCY_PRIORITY, Reservation,
                           ReservationTable, avoidance_cost, edge_key, node_key,
                           target_key)


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
    messages_rejected: int = 0       # forged, replayed or stale -- Phase 23
    _replay_guard: "security.ReplayGuard" = field(default_factory=security.ReplayGuard)

    # --- seeing trouble coming (Phase 4) ---
    conflicts: List[Conflict] = field(default_factory=list)
    conflicts_seen: int = 0
    _announced: Dict[tuple, float] = field(default_factory=dict)

    # --- booking squares before driving onto them (Phase 5) ---
    table: Optional[ReservationTable] = None   # its OWN copy of the bookings
    hold: bool = False              # true = do not start onto the next square
    emergency: bool = False         # true = stop NOW, even mid-aisle
    halted: bool = False            # true = emergency stop pulled by a person

    # --- running out of charge (Phase 13) ---
    # --- guessing where the work will be (Phase 12) ---
    demand: DemandModel = field(default_factory=DemandModel)
    staging: Optional[Cell] = None      # a spot we moved to on a hunch
    staging_why: str = ""               # in words, for the dashboard
    prepositions: int = 0
    _last_preposition_at: float = -99.0

    charger: Optional[Cell] = None    # the bay assigned to us
    # None with a charger means we are queued for that occupied bay. Once it
    # becomes free this becomes the slot we own; the charger itself stays fixed.
    _charge_slot: Optional[tuple] = None
    _charge_checked_at: float = -99.0
    _charge_wanted: bool = False

    # --- driven entirely by a central boss, not itself (Phase 18) ---
    # True means: do not plan, do not bid, do not reroute. Follow the last
    # route the boss actually sent, and nothing else -- exactly the
    # architecture this mode exists to test the fragility of.
    centrally_controlled: bool = False
    _central_task_seen: Optional[str] = None   # last task_id actually RECEIVED
    _central_path_seen: Optional[tuple] = None # last route actually ADOPTED

    # --- people on the floor (Phase 22) ---
    # human_id -> (x, y, sensed_at). Sensed locally, exactly like local_contacts
    # -- never from a message, because a safety system built around people
    # cannot depend on the network staying up.
    _known_humans: Dict[str, tuple] = field(default_factory=dict)
    near_person: Optional[str] = None       # who, if anyone, we are slowing for

    # --- is it starting to struggle? (Phase 23) ---
    health_score: float = 100.0
    health_band: str = HEALTHY
    health_reasons: List[str] = field(default_factory=list)
    _reroute_times: "deque" = field(default_factory=deque)
    _backout_times: "deque" = field(default_factory=deque)
    _last_health_check: float = -99.0
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
    _still_for: float = 0.0         # seconds spent going nowhere, see stalled_for
    _reversing: bool = False        # backing out of a segment it cannot finish

    # --- carrying on when the radio dies (Phase 14) ---
    # Robots this one can SEE with its own sensors, as (x, y, cell). Nothing to
    # do with messages: on real hardware this is the LiDAR, which keeps working
    # when the network does not.
    local_contacts: List[tuple] = field(default_factory=list)
    # square -> when something was first seen sitting on it. Used to tell a
    # parked robot from a moving one using sensors alone.
    _contact_since: Dict[tuple, float] = field(default_factory=dict)
    safe_mode: bool = False
    last_heard_any: float = -1.0    # when ANY message last arrived
    _ever_heard: bool = False
    _resync_pending: bool = False

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

    def _forget_hunch(self) -> None:
        """Drop any pre-positioning guess. A guess is never worth defending."""
        self.staging = None
        self.staging_why = ""

    def halt(self) -> None:
        """Emergency stop, pulled by a person. The robot stops where it is.

        This is not the same as clearing its destination. Clearing the
        destination only says "forget where you were going" -- a robot that is
        carrying a parcel gives itself the same destination back on the very
        next tick, because finishing what you are carrying is deliberately
        hard to interrupt. An emergency stop is the button on the side of a
        real robot: it stops, and it stays stopped until somebody releases it.
        """
        self.halted = True
        self._forget_hunch()

    def resume(self) -> None:
        """Release the emergency stop and let it get on with things again."""
        self.halted = False

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

    def plan_path(self, grid: Grid, start: Cell, goal: Cell, now: float,
                  cost_fn: Optional[Callable] = None,
                  blocked: Optional[set] = None) -> Optional[List[Cell]]:
        """The one bounded space-time planner used by every robot journey."""
        return find_space_time_path(
            grid, start, goal, table=self.table, robot_id=self.robot_id,
            now=now, speed=self.travel_speed(), cost_fn=cost_fn,
            blocked=blocked if blocked is not None else self.blocked_map.cells(now),
            clearance=self.CLEARANCE,
        )

    def decide(self, grid: Grid, cost_fn: Optional[Callable] = None,
               now: float = 0.0, traffic_aware: bool = True) -> None:
        """Work out what to do next. BOTH the simulator and ROS 2 call this.

        It only changes the robot's plan and status. It never moves anything.
        """
        if self.status is RobotStatus.FAILED:
            return

        # Sitting on a charger. Deciding runs AFTER the battery pass in a tick,
        # so without this it would see "no destination", call the robot idle,
        # and undo the charging status every single tick -- which it did, and
        # ten robots sat on four chargers draining to nothing.
        if self.status is RobotStatus.CHARGING:
            return

        # Stopped by a person. Keep the route on screen so you can see what it
        # was in the middle of, but decide nothing and go nowhere.
        if self.halted:
            self.status = (RobotStatus.WAITING if self.path or self.goal
                           else RobotStatus.IDLE)
            return

        # Phase 18. Driven by a central boss, not itself: it does not plan,
        # does not replan around anything it notices, and does not ask for
        # work. It runs the route it was last actually TOLD to run, and
        # nothing more -- arrival detection is the one piece of "thinking"
        # left, because recognising you have stopped moving is not
        # intelligence, it is just noticing.
        if self.centrally_controlled:
            if self.goal is None:
                self.status = RobotStatus.IDLE
                return
            if self.cell == self.goal and not self.path and self._progress == 0.0:
                self.goal = None
                self.status = RobotStatus.IDLE
                self._progress = 0.0
                return
            if self.path:
                if not self.hold:
                    self.status = RobotStatus.MOVING
            else:
                # Has somewhere to be but nothing to drive there with yet --
                # waiting on the boss, the same as a robot waiting on
                # anything else.
                self.status = RobotStatus.WAITING
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

        # Something has appeared on the route we were following. Tear it up --
        # but never mid-segment, or the robot snaps back onto the square behind
        # it. If it has set off, it finishes that step and replans on arrival.
        # (The safety layer emergency-stops it if the square it is entering is
        # the blocked one.)
        if self.path:
            in_the_way = self.blocked_map.blocks_any(self.path, now)
            if in_the_way is not None:
                if self._progress > 0.0:
                    self.path = [self.path[0]]
                else:
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
        # Normal FLEET-X planning starts from this robot's OWN received
        # reservations, intents and sensor contacts. No world/global traffic
        # count is consulted. Baseline/central experiments explicitly disable
        # this in World.tick() so their route choice remains unchanged.
        route_cost = cost_fn
        if route_cost is None and traffic_aware and self.table is not None:
            route_cost = avoidance_cost(
                self.table, self.robot_id, now,
                occupied=self.known_occupied(now),
            )

        route = self.plan_path(grid, self.cell, self.goal, now, route_cost)
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
        if self.halted:
            # The stop button is out. Wheels off, even part way down an aisle.
            self._drain_battery(dt, moving=False)
            self._still_for += dt
            return

        if self.status is RobotStatus.CHARGING:
            self.battery = min(100.0, self.battery + self.CHARGE_RATE * dt)
            self._still_for += dt
            return
        before = (self.cell, self.x, self.y)
        self._advance(dt)
        if self.goal is None or before != (self.cell, self.x, self.y):
            self._still_for = 0.0
        else:
            self._still_for += dt

    def _advance(self, dt: float) -> None:
        """The driving itself."""
        if self._reversing:
            # Backing up the way we came. Real robots have reverse gear, and
            # without it two robots part way into the same square from opposite
            # sides are wedged there for ever -- neither can go on, and neither
            # can get out of the way.
            self._progress -= self.travel_speed() * dt
            if self._progress <= 0.0:
                self._progress = 0.0
                self._reversing = False
                self.path = []              # plan afresh from where we are
                self.x = float(self.cell.x)
                self.y = float(self.cell.y)
            elif self.path:
                nxt = self.path[0]
                self.x = self.cell.x + (nxt.x - self.cell.x) * self._progress
                self.y = self.cell.y + (nxt.y - self.cell.y) * self._progress
            self._drain_battery(dt, moving=True)
            return

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
            # Stand still -- but a robot that is PART WAY along a step is
            # physically between two squares, and must stay where its wheels
            # actually are. Snapping it back onto the square behind it is a
            # teleport: it jumps backwards straight through every safety check,
            # which is how all three collisions in this project's history
            # happened. It cost 0.375 of a square when a robot stood down from
            # a charging bay mid-aisle and was marked idle on the spot.
            if self._progress == 0.0:
                self.x = float(self.cell.x)
                self.y = float(self.cell.y)
            self._drain_battery(dt, moving=False)
            return

        self._progress += self.travel_speed() * dt

        # Cross as many whole squares as this tick allows. A robot always
        # finishes the segment it started -- it stops cleanly ON a square,
        # never halfway down an aisle.
        while self._progress >= 1.0 and self.path:
            self._progress -= 1.0
            nxt = self.path.pop(0)
            waited = nxt == self.cell
            direction = _heading(self.cell, nxt)
            if direction:
                self.heading = direction
            self.cell = nxt
            if not waited:
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
        if not self.path or self.travel_speed() <= 0:
            return []
        speed = self.travel_speed()
        first = (1.0 - self._progress) / speed          # time to the next square
        return [first + i / speed for i in range(len(self.path))]

    # In safe mode a robot runs at half speed. 03_ROBOT_AND_ROS2 section 9:
    # "NETWORK OFF -> reduce speed -> use local obstacle avoidance". Less
    # information about the world means more caution, not the same caution.
    SAFE_MODE_SPEED_FACTOR = 0.5

    # --- Phase 22: never enter a person's space, always slow down near one ---
    PERSON_STOP_RADIUS = 1.3    # a hard line. Never crossed, whatever else is
                                 # happening -- checked every tick, in BOTH
                                 # fleets, the same as the robot-robot reflex.
    PERSON_SLOW_RADIUS = 3.0    # begin easing off well before that line
    PERSON_SENSE_RANGE = 4.0    # how far the "LiDAR" for people reaches --
                                 # wider than SENSOR_RANGE for ordinary
                                 # obstacles on purpose: a person deserves
                                 # earlier warning than a dropped box does.
    PERSON_MIN_SPEED = 0.35     # floor on the slow-down ramp -- crawl, but
                                 # keep making progress, right up to the point
                                 # the hard stop takes over completely.
    PERSON_BLOCK_TTL = 1.2      # how long a sensed person's square (and the
                                 # ones next to it) stay marked "in the way"
                                 # for ROUTING. Short and constantly renewed
                                 # while they are near, so the map does not
                                 # carry a phantom wall once they walk off.

    def travel_speed(self) -> float:
        """How fast it is allowed to drive right now.

        Network trouble halves it. Being near a person on top of that eases it
        down further still, the closer they are -- multiplied together, so a
        robot already cautious about the network is MORE cautious near a
        person, never less.
        """
        speed = self.speed * (self.SAFE_MODE_SPEED_FACTOR if self.safe_mode else 1.0)
        return speed * self._person_speed_factor()

    def _nearest_person(self, x: float, y: float) -> Optional[tuple]:
        """(distance, human_id) to the closest person we currently know
        about, or None if we do not know of any."""
        best = None
        for human_id, (hx, hy, _seen) in self._known_humans.items():
            d = ((hx - x) ** 2 + (hy - y) ** 2) ** 0.5
            if best is None or d < best[0]:
                best = (d, human_id)
        return best

    def _person_speed_factor(self) -> float:
        """1.0 far from anyone, ramping straight down to PERSON_MIN_SPEED by
        the time the hard stop line is reached."""
        nearest = self._nearest_person(self.x, self.y)
        self.near_person = None
        if nearest is None:
            return 1.0
        dist, human_id = nearest
        if dist >= self.PERSON_SLOW_RADIUS:
            return 1.0
        self.near_person = human_id
        span = self.PERSON_SLOW_RADIUS - self.PERSON_STOP_RADIUS
        if span <= 0:
            return self.PERSON_MIN_SPEED
        t = max(0.0, min(1.0, (dist - self.PERSON_STOP_RADIUS) / span))
        return self.PERSON_MIN_SPEED + t * (1.0 - self.PERSON_MIN_SPEED)

    def current_velocity(self) -> float:
        """How fast it is ACTUALLY going, right now.

        Not "how fast could it go" -- a robot that is stopped and waiting must
        report zero. It used to report its full speed whenever it had a route,
        which made every other robot dead-reckon it creeping forward into
        squares it was not moving into. Two stopped robots would then block each
        other forever over a square neither was going to enter.
        """
        if self.halted or self.hold or not self.path or self.path[0] == self.cell:
            return 0.0
        if self.status in (RobotStatus.FAILED, RobotStatus.IDLE,
                           RobotStatus.WAITING, RobotStatus.CHARGING):
            return 0.0
        return self.travel_speed()

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

    def communicate(self, bus, now: float) -> List[str]:
        """Listen first, then speak. Called once per tick by the world.

        The listening half is identical on a real robot. The speaking half is
        identical too -- only the bus underneath changes.
        """
        if bus is None:
            return []

        # 1. Read the inbox. Phase 23: check every message BEFORE the brain
        # reads it -- a forged or replayed one never reaches fleet.ingest or
        # anything downstream of it. Rejection is not silent: it goes into
        # the returned notes, the same as any other decision worth logging.
        notes: List[str] = []
        for message in bus.poll(self.robot_id):
            claimed = getattr(message, "robot_id", "?")
            kind = getattr(message, "type", "?")
            # Almost always the same as claimed -- see TaskClaim.sender for
            # the one case where a robot speaks about a PEER rather than
            # itself. Replay protection has to key on whoever's counter the
            # sequence number actually came from, or a legitimate message
            # about someone else reads as an impostor in THEIR sequence.
            sender = getattr(message, "sender", "") or claimed

            sig = security.check_signature(message)
            if not sig:
                self.messages_rejected += 1
                notes.append(f"{self.robot_id} rejected a {kind} claiming to "
                            f"be from {claimed} - {sig.reason}")
                continue

            fresh = self._replay_guard.check(
                sender, getattr(message, "seq", 0),
                getattr(message, "timestamp", 0.0), now,
                stream=str(kind))
            if not fresh:
                self.messages_rejected += 1
                notes.append(f"{self.robot_id} rejected a {kind} from "
                            f"{claimed} - {fresh.reason}")
                continue

            self.fleet.ingest(message, now)
            self._ingest_reservation(message)
            self._ingest_jam_news(message, now)
            self._ingest_blocked_aisle(message, now)
            self._ingest_task_news(message, now)
            self._ingest_central_command(message, now)
            self.messages_heard += 1
            # Anything at all arriving means the radio is alive.
            self.last_heard_any = now
            self._ever_heard = True

        if self.status is RobotStatus.FAILED:
            return notes

        if self._resync_pending:
            self._resync(bus, now)

        # 2. Heartbeat -- "I am alive", on a fixed drumbeat.
        if now >= self._next_heartbeat:
            self._next_heartbeat = now + self.HEARTBEAT_PERIOD
            self._publish(bus, Heartbeat(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                battery=self.battery, status=self.status.value,
                health=self.health_band,
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

        return notes

    def _publish(self, bus, message) -> None:
        self.seq += 1
        self.messages_sent += 1
        security.seal(message)          # Phase 23: sign it, then let it go
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
        if self.status is RobotStatus.FAILED:
            self._set_hold(False, now, dt)
            return

        if not self.path:
            # Phase 22. Parked does not mean safe: nothing about NOT having a
            # route stops a person walking up to where we are standing. A
            # person near a robot that never checks anything is exactly the
            # gap that let one through in testing -- an idle robot with no
            # path simply never looked. So look anyway, at our own square,
            # even with nowhere to go.
            person = self._human_too_close(self.cell, now)
            self._set_hold(person is not None, now, dt, emergency=True)
            if person is not None:
                self.blocked_by = f"person:{person}"
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
        self._reroute_times.append(now)
        return f"{self.robot_id} backed off to ({spot.x}, {spot.y}) - stuck too long"

    # --------------------------------------- booking squares (Phase 5)

    # --- Phase 13: charge ---
    DRAIN_MOVING = 0.35               # percent per second while driving
    DRAIN_IDLE = 0.05                 # percent per second just sitting there
    CHARGE_RATE = 4.0                 # percent per second on a charger
    CHARGE_UNTIL = 100.0              # once charging, fill completely
    CHARGE_BELOW = 15.0               # working robot: emergency charge threshold
    IDLE_CHARGE_AT = 80.0             # idle robot may top up at/below this
    BATTERY_RESERVE = 15.0            # must still have this on reaching a bay
    BATTERY_BAND = 5.0                # agreement bucket, see charge_urgency
    CHARGE_RECHECK = 2.0              # seconds between re-doing the sums
    TARGET_LEASE_SECONDS = 8.0        # renewed while pickup/drop/charger is wanted

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
        if message.kind == "NODE":
            key = node_key(cells[0])
        elif message.kind == "TARGET":
            key = target_key(cells[0])
        else:
            key = edge_key(cells[0], cells[1])
        if message.action == "RELEASE":
            self.table.release(message.robot_id, key)
        else:
            self.table.put(Reservation(
                robot_id=message.robot_id, resource=key,
                start=message.start, end=message.end, priority=message.priority,
            ))

    def _goal_needs_lease(self) -> bool:
        """Pickup, drop-off and charger targets must have one owner."""
        if self.goal is None:
            return False
        if self.charger == self.goal:
            return True
        return (self.task is not None
                and self.goal in (self.task.pickup, self.task.dropoff))

    def _wanted_slots(self, now: float) -> Dict[tuple, tuple]:
        """Which squares and aisle segments this robot needs, and for how long.

        Each entry is (start, end, priority).

        Only the next few squares. Booking the whole route would let one robot
        own half the warehouse while everybody else sat still.
        """
        wanted: Dict[tuple, tuple] = {}
        speed = self.travel_speed()
        dwell = (1.0 / speed) if speed > 0 else 1.0
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
            prev = chain[i]
            if node == prev:
                # A space-time WAIT occupies the same square for another time
                # step. Keep one continuous physical-occupancy claim; there is
                # no traversed edge to reserve.
                key = node_key(node)
                old = wanted.get(key)
                wanted[key] = (
                    min(old[0], enter) if old else enter,
                    max(old[1], leave) if old else leave,
                    OCCUPANCY_PRIORITY,
                )
                continue
            wanted[node_key(node)] = (enter, leave, self.priority)
            # the aisle segment used to get there -- booked in BOTH directions,
            # which is what stops a head-on
            edge_start = now + (etas[i - 1] if i > 0 else 0.0) - self.CLEARANCE
            wanted[edge_key(prev, node)] = (edge_start, now + etas[i] + self.CLEARANCE,
                                            self.priority)

        # Pickup, drop-off and charger destinations get a renewable exclusive
        # lease. It naturally expires if this robot fails or changes goal.
        if self._goal_needs_lease():
            eta = etas[-1] if etas else 0.0
            lease_end = now + max(self.TARGET_LEASE_SECONDS, eta + dwell)
            lease_priority = (self.charge_urgency()
                              if self.charger == self.goal else self.priority)
            wanted[target_key(self.goal)] = (
                now - self.CLEARANCE, lease_end + self.CLEARANCE,
                lease_priority,
            )

        # Phase 13: the charging bay we have booked, held for the whole trip
        # plus the top-up. Same table, same rules, same broadcast as any other
        # square -- a bay is just a square somebody wants.
        if self.charger is not None and self._charge_slot is not None:
            start, end = self._charge_slot
            key = node_key(self.charger)
            # Once we are physically on the bay, keep the unbeatable occupancy
            # claim above. Replacing it with charge urgency made an occupied bay
            # look available to a robot with a lower battery.
            if key not in wanted:
                wanted[key] = (start, end, self.charge_urgency())
        return wanted

    def reserve_ahead(self, bus, now: float) -> None:
        """Claim what we need, give back what we no longer do, tell everyone."""
        if self.status is RobotStatus.FAILED:
            # A broken robot must not keep squares booked, or the rest of the
            # fleet drives around holes that nobody is in. release_all() hands
            # its charging bay back too.
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
        self._contact_since.clear()
        self.local_contacts = []
        self.safe_mode = False
        self.last_heard_any = -1.0
        self._ever_heard = False
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
        # Its charging bay goes back the moment it fails, not on the next tick.
        # One of four bays lost for the rest of a run is a quarter of the
        # fleet's ability to recharge.
        self.charger = None
        self._charge_slot = None
        self._charge_wanted = False
        self._charge_checked_at = -99.0

    def _broadcast_reservation(self, bus, now: float, action: str, res: Reservation) -> None:
        if bus is None:
            return
        if res.resource[0] in ("N", "T"):
            kind = "NODE" if res.resource[0] == "N" else "TARGET"
            cells = [(res.resource[1], res.resource[2])]
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

        if self.status is RobotStatus.FAILED:
            self._set_hold(False, now, dt)
            return

        if not self.path:
            # Phase 22. Same reasoning as stop_and_wait_check(): an idle
            # robot with no route never used to check anything at all, so a
            # person could walk right up to a parked robot with nothing
            # noticing until the very last instant, if at all.
            person = self._human_too_close(self.cell, now)
            self._set_hold(person is not None, now, dt, emergency=True)
            if person is not None:
                self.blocked_by = f"person:{person}"
            return

        nxt = self.path[0]

        # Something is physically in that square. Nothing else matters.
        if self.blocked_map.is_blocked(nxt, now):
            self.blocked_by = "obstacle"
            self._set_hold(True, now, dt, emergency=True)
            return

        if self._local_safety_says_stop(nxt, now, dt):
            return

        # WAIT is a timed occupation of the current square, not traversal of a
        # zero-length edge. Requiring a fake self-edge would stop every wait.
        checks = [(node_key(nxt), self._claimed.get(node_key(nxt)))]
        if nxt != self.cell:
            checks.append((edge_key(self.cell, nxt),
                           self._claimed.get(edge_key(self.cell, nxt))))
        if nxt == self.goal and self._goal_needs_lease():
            checks.append((target_key(nxt),
                           self._claimed.get(target_key(nxt))))
        for key, mine in checks:
            if mine is None:
                # Safety invariant: booking failure must stop motion, not
                # silently bypass the reservation system. Mid-edge also stops
                # immediately and existing back-out recovery can unwind it.
                self.blocked_by = "unreserved"
                self._set_hold(True, now, dt, emergency=True)
                return
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
        # Phase 22. Checked FIRST, and unconditionally -- a person is never
        # part of the right-of-way contest below. There is no priority, no
        # negotiation, no "they are parked so I can go": too close to a
        # person always means stop, full stop. Checked against both the
        # square about to be entered AND this robot's own actual position,
        # which the robot-contact checks below only do for `nxt` -- a person
        # can be close to the robot's real, in-between-squares body even when
        # the target square itself is still clear.
        blocking_person = self._human_too_close(nxt, now)
        if blocking_person is not None:
            self.blocked_by = f"person:{blocking_person}"
            self._set_hold(True, now, dt, emergency=True)
            return True

        for contact in self._all_contacts(now):
            # Where it was when it last spoke, AND where it has probably got to
            # since. Checking only the first was how a robot already part way
            # into a square looked like it was still safely outside it.
            here = (contact["x"], contact["y"])
            soon = contact["soon"]
            for px, py in (here, soon):
                gap_x, gap_y = px - nxt.x, py - nxt.y
                if gap_x * gap_x + gap_y * gap_y < self.SAFE_GAP * self.SAFE_GAP:
                    self.blocked_by = contact["id"]
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
            if contact["known_still"]:
                continue

            if (contact["next_cell"] is not None
                    and contact["next_cell"] != (nxt.x, nxt.y)):
                continue                       # we know it is going elsewhere

            # Settled WITHOUT the booking table, because the table is exactly
            # what the two of them can disagree about. The rule always gives the
            # two robots opposite answers, so it can never let both through, and
            # can never stop both either.
            if self._yields_to_contact(contact, nxt):
                self.blocked_by = contact["id"]
                self._set_hold(True, now, dt, emergency=True)
                return True

        return False

    def _human_too_close(self, nxt: Cell, now: float) -> Optional[str]:
        """Is any known person within the hard stop line -- of the square we
        are about to enter, or of where our own body actually is right now?"""
        for human_id, (hx, hy, _seen) in self._known_humans.items():
            for px, py in ((nxt.x, nxt.y), (self.x, self.y)):
                gap_x, gap_y = hx - px, hy - py
                if gap_x * gap_x + gap_y * gap_y < self.PERSON_STOP_RADIUS ** 2:
                    return human_id
        return None

    def _all_contacts(self, now: float) -> List[Dict[str, object]]:
        """Every other robot this one knows about, however it found out.

        Two sources, and the difference is the whole point of this phase:

          RADIO   -- rich (name, priority, where it says it is going), but it
                     stops dead the moment the network does.
          SENSORS -- only a shape at a position, but it never stops working.

        Before Phase 14 the "local safety reflex" read the radio and nothing
        else. Cut the network and every robot believed it was alone in the
        warehouse. The claim that safety did not depend on the network was
        simply not true. It is now.
        """
        contacts: List[Dict[str, object]] = []
        by_radio = set()

        for note in self.fleet.fresh(now):
            pose_fresh = (now - note.last_pose) <= self.POSE_TRUST
            intent_fresh = ((now - note.last_intent) <= self.INTENT_TRUST
                            and note.next_cell is not None)
            contacts.append({
                "id": note.robot_id,
                "x": note.x, "y": note.y, "soon": note.position_at(now),
                "cell": Cell(note.cell[0], note.cell[1]),
                "priority": note.priority,
                "known_still": pose_fresh and note.velocity <= 0.01,
                "next_cell": note.next_cell if intent_fresh else None,
                "by_radio": True,
            })
            by_radio.add((note.cell[0], note.cell[1]))

        # Anything the sensors saw that the radio did not already cover.
        for cx, cy, cell in self.local_contacts:
            key = (cell.x, cell.y)
            if key in by_radio:
                continue
            since = self._contact_since.get(key, now)
            contacts.append({
                "id": "unknown", "x": cx, "y": cy, "soon": (cx, cy),
                "cell": cell, "priority": None,
                "known_still": (now - since) >= self.CONTACT_STILL_AFTER,
                "next_cell": None, "by_radio": False,
            })

        return contacts

    def _yields_to_contact(self, contact: Dict[str, object], nxt: Cell) -> bool:
        """Do I give way to this one?

        If we are talking: the usual rule -- priority, then the lower name.
        Both robots compute it identically, so it can never say "both go".

        If the radio is down: a sensor sees a shape, not a name. So fall back to
        something both can work out from POSITION alone -- whoever is further
        from the contested square gives way, and if that is level, the one
        further down-and-right gives way. Cruder and slower, but both reach the
        same answer with nothing said between them, and it cannot stop both,
        because it is never true for both at once.
        """
        if contact["by_radio"] and contact["priority"] is not None:
            return yields_to(self.priority, self.robot_id,
                             int(contact["priority"]), str(contact["id"]))

        theirs = contact["cell"]
        my_gap = abs(self.cell.x - nxt.x) + abs(self.cell.y - nxt.y)
        their_gap = abs(theirs.x - nxt.x) + abs(theirs.y - nxt.y)
        if my_gap != their_gap:
            return my_gap > their_gap
        return (self.cell.y, self.cell.x) > (theirs.y, theirs.x)

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

    def geometry_brake(self, now: float, dt: float, other_id: str) -> None:
        """Forced to a hard stop by World._enforce_geometry_gap (Phase 19) --
        physically too close to another robot's ACTUAL position, regardless
        of what square booking, negotiation, or a central command decided.

        The same kind of hard, unnegotiated stop _human_too_close() already
        uses: no priority, no reasoning about who goes first, just "too
        close -- stop." Whichever robot in the pair is mid-segment (at least
        one always is, or they would not have been close enough to trigger
        this) is left exactly where back_out_if_wedged() already knows how
        to find it.
        """
        self.blocked_by = other_id
        self._set_hold(True, now, dt, emergency=True)

    BACK_OUT_AFTER = 3.0        # wedged this long mid-segment, so reverse

    def back_out_if_wedged(self, now: float) -> Optional[str]:
        """Part way into a square it cannot enter, and going nowhere.

        Two robots can each commit to the same square from opposite ends, stop
        dead a fraction of a square in, and then neither can finish nor get out
        of the way. The only move left is the one a real robot has and this one
        did not: reverse back onto the square it came from.
        """
        if self._reversing:
            return None
        if not (self.hold and self.emergency):
            return None
        if self._progress <= 0.0:
            return None                 # standing on a square, nothing to back out of
        if self.stalled_for(now) < self.BACK_OUT_AFTER:
            return None

        self._reversing = True
        self._backout_times.append(now)
        self._suspended_goal = self._suspended_goal or self.goal
        return (f"{self.robot_id} was wedged part way into "
                f"({self.path[0].x}, {self.path[0].y}) - backing out")

    def waited_for(self, now: float) -> float:
        """How long this robot has been HELD right now."""
        return 0.0 if self.waiting_since is None else max(0.0, now - self.waiting_since)

    def stalled_for(self, now: float) -> float:
        """How long it has been going nowhere, whatever it has been doing.

        This is not the same as being held. A robot that keeps rerouting is
        busy, not blocked -- so its "held" timer keeps resetting, and it can
        shuffle about fruitlessly for ever without ever looking stuck.

        That actually happened: a robot sat outside the packing station for
        800 seconds, replanning the whole time, reporting 0.0s stuck, and the
        deadlock recovery never fired because it never looked jammed. Measuring
        actual progress closes that hole.
        """
        return 0.0 if self.goal is None else self._still_for

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

        # Squares that physically have a robot standing on them right now --
        # heard OR seen. In a blackout the radio half is empty and the sensor
        # half is all there is.
        occupied = self.known_occupied(now)

        # If it is already part way along a segment it is COMMITTED to reaching
        # that square -- a real robot cannot jump back onto the one behind it.
        # So plan the new route FROM there, not from the square it has left.
        #
        # Getting this wrong teleported a robot 1.37 squares sideways in a
        # single tick, straight through another robot. Nothing predicts
        # teleportation, so the crash was not seen coming either.
        committed = self.path[0] if (self._progress > 0.0 and self.path) else None
        start = committed if committed is not None else self.cell

        cost_fn = avoidance_cost(
            self.table, self.robot_id, now,
            avoid_robot=self.blocked_by, occupied=occupied,
        )
        alternative = self.plan_path(
            grid, start, self.goal, now, cost_fn,
            blocked=self.blocked_map.cells(now),
        )

        if alternative is None or len(alternative) < 2:
            self.last_decision = f"{self.robot_id} waits - no way round"
            return False

        # When committed, the square we are entering stays the first step, so
        # progress along it still means what it did a moment ago.
        new_path = list(alternative) if committed is not None else alternative[1:]

        # Has anything actually changed about where we go NEXT? If the first
        # free step is the same, this is the same plan wearing a hat: the robot
        # would walk back into the same jam, having burnt its cooldown.
        #
        # When committed, the first step is fixed (we are already crossing to
        # it), so the step that matters is the one after it.
        skip = 1 if committed is not None else 0
        mine = self.path[skip] if len(self.path) > skip else None
        theirs = new_path[skip] if len(new_path) > skip else None
        if mine == theirs:
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
        # "unknown" is what a robot spotted on sensors alone is called
        # internally. Do not print that at a person.
        who = self.blocked_by
        if who in (None, "unknown"):
            who = "a robot it can only see"
        self.last_decision = (
            f"{self.robot_id} gave way to {who} - "
            f"going around ({extra:+d} squares)"
        )
        self.path = new_path
        self.replans += 1
        self.reroutes += 1
        self._reroute_times.append(now)
        self.status = RobotStatus.REROUTING
        self._reroute_at = now
        # Its old bookings are stale now. Keep it still for one tick; the next
        # reserve_ahead claims the new route before it moves.
        self.waiting_since = None
        return True

    # ----------------------------- getting out of the way when idle

    def vacate_station(self, grid: Grid, now: float) -> Optional[str]:
        """Finished, with nothing to do? Then do not stand on a station.

        A robot that finishes a delivery simply stops -- on the delivery bay.
        With three robots there is usually room. With five, four idle robots
        end up parked on and around the four packing squares and a robot
        carrying the last parcel physically cannot get in. That is exactly how
        a 20-order batch ended at 19/20.

        Real warehouses do not let robots idle in the pick face either. So an
        idle robot steps off onto ordinary floor and waits there.
        """
        if self.charger is not None or self.status is RobotStatus.CHARGING:
            return None               # it is there on purpose
        if self.task is not None or self.goal is not None:
            return None
        if self.status is not RobotStatus.IDLE:
            return None
        if grid.kind(self.cell) is CellKind.FLOOR:
            return None                       # already out of the way

        spot = self._nearest_parking(grid, now)
        if spot is None:
            return None
        self.set_goal(spot)
        return (f"{self.robot_id} cleared the station at "
                f"({self.cell.x}, {self.cell.y}) -> ({spot.x}, {spot.y})")

    # ------------------------------------ Phase 12: standing somewhere useful

    PREPOSITION_EVERY = 4.0       # seconds between having the thought
    PREPOSITION_MIN_GAIN = 4.0    # squares closer it must make us, to be worth it
    HUNCH_PATIENCE = 8.0          # stuck this long on a guess, drop the guess
    HUNCH_SPREAD = 24             # different waiting squares to scatter over

    def consider_prepositioning(self, grid: Grid, now: float,
                                inventory=None) -> Optional[str]:
        """Nothing to do? Then wait somewhere the next order is likely to be.

        Everything about this is optional and reversible on purpose:

          * only a robot with no job, no charging trip and nowhere to be even
            considers it;
          * it moves to ordinary floor, never onto a shelf face, a packing bay
            or a charger, so a robot guessing wrong is never in anybody's way;
          * the moment a real job turns up, the job wins -- work_on_tasks sets
            a goal and the hunch is forgotten;
          * if the model has no opinion, or the move would not gain much, it
            does nothing at all.

        So a wrong guess costs one short empty drive. That is the whole risk.
        """
        if (self.task is not None or self.charger is not None
                or self.halted or self.status is RobotStatus.FAILED
                or self.safe_mode
                or self.status is RobotStatus.CHARGING):
            # Anything real to do beats a hunch, so forget the hunch. Clearing
            # it here as well as when it is taken matters: a robot that booked
            # a charger while parked on a guess kept showing the guess, and the
            # dashboard said it was sightseeing on 10% battery.
            #
            # safe_mode is in that list because out of touch means no new work
            # is reaching anybody, so there is nothing to get ahead of -- and
            # because a robot that cannot hear the others has no idea where
            # they are going. Guessing is a luxury for when the radio works.
            self.staging = None
            self.staging_why = ""
            return None
        if self.staging is not None and self._still_for > self.HUNCH_PATIENCE:
            # Been trying to reach it and getting nowhere. A guess is never
            # worth defending: drop it and stand where we are. Without this,
            # robots that wanted the same spot sat holding for each other
            # indefinitely, because nothing ever told them to stop wanting it.
            spot = self.staging
            self._forget_hunch()
            self.set_goal(None)
            return (f"{self.robot_id} gave up waiting for ({spot.x}, {spot.y}) "
                    f"- not worth it for a guess")

        if self.goal is not None or self.path:
            return None                       # already going somewhere
        if inventory is None or self._progress > 0.0:
            return None
        if now - self._last_preposition_at < self.PREPOSITION_EVERY:
            return None
        self._last_preposition_at = now

        pred = self.demand.predict(now)
        if pred is None:
            self.staging = None
            self.staging_why = ""
            return None                       # honestly no idea; stay put

        # Do not all pile into the same aisle. Robots ALREADY closer to it than
        # we are have it covered -- a robot counts the others from its own view
        # of the fleet, so this needs no agreement and no messages of its own.
        spot = self._staging_spot(grid, now, pred.zone, inventory)
        if spot is None:
            return None

        here = self._distance_to(grid, self.cell, now)
        mine = self._distance_to(grid, spot, now, start=self.cell)
        if mine is None:
            return None
        nearer = sum(1 for n in self.fleet.fresh(now)
                     if abs(n.x - spot.x) + abs(n.y - spot.y) < mine)
        if nearer >= 2:
            return None                       # two robots are closer; leave it

        gain = self._zone_gain(grid, now, pred.zone, spot, inventory)
        if gain is None or gain < self.PREPOSITION_MIN_GAIN:
            return None                       # not worth the drive

        self.staging = spot
        shelf_name = self._nearest_shelf_name(spot, inventory)
        self.staging_why = (f"pre-positioned near {shelf_name}, busy zone "
                            f"({pred.reason})")
        self.prepositions += 1
        self.set_goal(spot)
        return f"{self.robot_id} {self.staging_why}"

    def _distance_to(self, grid: Grid, cell: Cell, now: float,
                     start: Optional[Cell] = None) -> Optional[float]:
        leg = self.plan_path(
            grid, start or self.cell, cell, now,
            blocked=self.blocked_map.cells(now),
        )
        return None if leg is None else float(len(leg) - 1)

    def _nearest_shelf_name(self, spot: Cell, inventory) -> str:
        """The rack this spot is waiting next to, for the explanation."""
        best, best_d = "", 1e9
        for shelf in inventory.by_name.values():
            d = abs(shelf.face.x - spot.x) + abs(shelf.face.y - spot.y)
            if d < best_d:
                best, best_d = shelf.name, d
        return best

    def _staging_spot(self, grid: Grid, now: float, zone: str,
                      inventory) -> Optional[Cell]:
        """An out-of-the-way square to wait on, in the middle of a busy aisle.

        The middle, because we do not know WHICH rack in the aisle the next
        order will be for, and standing in the middle is the best answer to
        that question. Never on a shelf face -- that is where a picking robot
        has to stand.
        """
        faces = [s.face for s in inventory.by_name.values() if s.name[0] == zone]
        if not faces:
            return None
        mid_x = sum(f.x for f in faces) / len(faces)
        mid_y = sum(f.y for f in faces) / len(faces)
        # EVERY shelf's face, not just this aisle's. A spot in the middle of
        # aisle B is easily the picking square for a rack in A or C, and a
        # robot waiting on a hunch must never be standing where a robot with an
        # actual job has to stand.
        busy = {sh.face for sh in inventory.by_name.values()}

        # The candidate list is built from the MAP ALONE -- no occupancy, no
        # blockages, nothing that one robot might know and another might not.
        # That matters: every robot must compute the identical list, or the
        # same index means a different square to each of them and they collide
        # on a choice anyway. It did exactly that.
        near = sorted(
            (c for c in grid.cells_of_kind(CellKind.FLOOR) if c not in busy),
            key=lambda c: ((c.x - mid_x) ** 2 + (c.y - mid_y) ** 2, c.x, c.y)
        )[:self.HUNCH_SPREAD]
        if not near:
            return None

        # Then take the slot our own NAME points at. Deliberately not "is
        # anybody else already going there": that reads the radio, and with the
        # radio cut every robot believes it is alone -- four of them drove to
        # the very same square and sat holding for each other for two minutes.
        # A name needs no messages and cannot go stale, which is the same
        # reasoning that decides who gives way.
        digits = "".join(ch for ch in self.robot_id if ch.isdigit())
        mine = (int(digits) - 1) if digits else 0
        spot = near[mine % len(near)]

        # Only now look at what we personally know, and simply decline if our
        # square is not free. Declining is always safe; queueing for it is not.
        if spot in self.known_occupied(now) or spot in self.blocked_map.cells(now):
            return None
        return spot

    def _zone_gain(self, grid: Grid, now: float, zone: str, spot: Cell,
                   inventory) -> Optional[float]:
        """How many squares closer to that aisle the move would leave us."""
        faces = [s.face for s in inventory.by_name.values() if s.name[0] == zone]
        if not faces:
            return None
        def typical(c: Cell) -> float:
            return sum(abs(f.x - c.x) + abs(f.y - c.y) for f in faces) / len(faces)
        return typical(self.cell) - typical(spot)

    def _nearest_parking(self, grid: Grid, now: float) -> Optional[Cell]:
        """Closest ordinary floor square with nobody on it."""
        taken = set(self.known_occupied(now))
        blocked = self.blocked_map.cells(now)
        seen = {self.cell}
        queue = [self.cell]
        while queue:
            cell = queue.pop(0)
            for nxt in grid.neighbours(cell):
                if nxt in seen or nxt in blocked:
                    continue
                seen.add(nxt)
                if grid.kind(nxt) is CellKind.FLOOR and nxt not in taken:
                    return nxt
                queue.append(nxt)
            if len(seen) > 60:
                break                          # do not search the whole warehouse
        return None

    # ------------------------------------- real jobs (Phase 9)

    def _ingest_task_news(self, message, now: float) -> None:
        """Keep our own copy of the job board up to date."""
        if isinstance(message, TaskAnnounce):
            # Phase 12: every order that goes past teaches us something about
            # where the work comes from. Each robot keeps its own model, built
            # from the same announcements everybody hears -- no server holds it.
            if message.shelf:
                self.demand.record(message.shelf[0], now)

            task = self.board.get(message.task_id)
            if task is None:
                self.board.add(Task(
                    task_id=message.task_id,
                    pickup=Cell(*message.pickup), dropoff=Cell(*message.dropoff),
                    product=message.product, priority=message.priority,
                    shelf=message.shelf, quantity=message.quantity,
                    created_at=now, announced_at=now,
                    status=TaskStatus.ANNOUNCED, flexible=message.flexible,
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
                # Pickup is a physical commitment and DONE is terminal. A
                # delayed claim from another bidder must not roll either state
                # backwards on this robot's local board.
                if task.status in (TaskStatus.CARRYING, TaskStatus.DONE):
                    if (task.assigned_robot == self.robot_id
                            and task.status is TaskStatus.CARRYING):
                        self._task_notes.append(
                            f"{self.robot_id} ignored late claim for "
                            f"{task.task_id} - parcel already collected")
                    return
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

    def _ingest_central_command(self, message, now: float) -> None:
        """Phase 18. The ONE message a centrally-planned robot acts on: "do
        this job, drive exactly this route." Not for us, ignored -- the boss
        addresses every robot on the floor, same as any broadcast.

        Building a fresh local Task from the message, rather than reaching
        into a shared object somewhere, is deliberate: it is the SAME thing
        _ingest_task_news() already does for TaskAnnounce, and it means a
        centrally-controlled robot knows only what it has actually been
        told, exactly like every other robot in this project.

        The boss RESENDS a command it has not seen confirmed, the same way
        _reannounce_forgotten_tasks() already does for orders -- the network
        it travels over can lose a message just as easily as any other. A
        resend carries the identical path as before, and adopting it again
        would reset _progress to 0 while the robot is honestly part way
        across a square, snapping it backwards onto the square behind it --
        exactly the teleport this project has broken on before. So the new
        path is only ever adopted when it is actually DIFFERENT from the one
        already being driven, never merely because a message arrived.
        """
        if not isinstance(message, CentralCommand):
            return
        if message.robot_id != self.robot_id:
            return

        route = tuple(message.path)
        if route == self._central_path_seen:
            return                      # a resend of what we are already doing

        if message.task_id and message.task_id != self._central_task_seen:
            self._central_task_seen = message.task_id
            task = Task(
                task_id=message.task_id,
                pickup=Cell(*message.pickup), dropoff=Cell(*message.dropoff),
                product=message.product, priority=5,
                created_at=now, announced_at=now,
                assigned_robot=self.robot_id, status=TaskStatus.ASSIGNED,
            )
            self.board.add(task)
            self.staging = None
            self.staging_why = ""
            self.task = task

        self._central_path_seen = route
        self.path = [Cell(x, y) for x, y in message.path]
        self.goal = self.path[-1] if self.path else None
        self._progress = 0.0

    def work_on_tasks(self, grid: Grid, bus, now: float) -> List[str]:
        """Bid for jobs, claim what we win, and get on with it."""
        notes = self._task_notes
        self._task_notes = []

        if self.status is RobotStatus.FAILED:
            if self.task is not None:
                self._drop_task()
            return notes

        # Stopped by a person. It keeps the job it is holding -- it has not
        # failed, it is just standing still -- but it takes no new work and,
        # crucially, does not hand itself its destination back.
        if self.halted:
            return notes

        # On the way to a charging bay, or sitting on one. It keeps a parcel it
        # is already carrying and picks the delivery back up afterwards, but it
        # must not re-aim itself at the drop-off while it is going to charge.
        if self.charger is not None or self.status is RobotStatus.CHARGING:
            return notes

        # Phase 18. Driven by a central boss: it does not bid (there is no
        # auction to bid into -- the boss decides, alone, with everything it
        # can see), and it does not reassign a quiet peer's job (it has no
        # peer relationship with anyone to notice that with). It only ever
        # does the one thing any robot does once it is actually carrying
        # something: finish the trip.
        if self.centrally_controlled:
            if self.task is not None:
                notes += self._progress_task(grid, bus, now)
            return notes

        # A peer has gone quiet while holding a job. Put it back up for
        # auction -- 06_TASK_ALLOCATION section 4. Whoever notices first does
        # it; a repeat is harmless because the job keeps its id.
        #
        # BUT only while our own radio is clearly working. If we cannot hear
        # ANYBODY, the one that has gone quiet is probably us, and "everyone
        # else has failed, so I will take all their jobs" is exactly the wrong
        # conclusion. Three robots reached it at once, took the same parcel to
        # the same square, and wedged each other there.
        if self.link_quiet_for(now) <= self.LINK_TRUSTED_WITHIN:
            for note in self.fleet.stale(now):
                for task in self.board.held_by(note.robot_id):
                    task.release()
                    notes.append(f"{note.robot_id} has gone quiet - "
                                 f"{task.task_id} back up for auction")
                    if bus is not None:
                        # Phase 23: robot_id names the quiet robot, so the
                        # release matches its held task on every board -- but
                        # `sender` names US, the one actually speaking, since
                        # `seq` below comes from OUR OWN counter and only
                        # means something as part of OUR sequence, not theirs.
                        self._publish(bus, TaskClaim(
                            robot_id=note.robot_id, sender=self.robot_id,
                            timestamp=now, seq=self.seq,
                            task_id=task.task_id, action="RELEASE"))

        if self.task is not None:
            # Finish what we are already carrying, network or no network.
            notes += self._progress_task(grid, bus, now)
            return notes

        if self.safe_mode:
            # No radio means no auction: nobody would hear the bid, and nobody
            # would hear the claim. Taking a job anyway is how three robots
            # ended up carrying the same parcel to the same square and wedging
            # each other there. So a robot out of touch finishes what it has
            # and then waits. There is genuinely no new work reaching it.
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

    # How much a bay is penalised for each robot already on it or heading to
    # it. Big enough that a slightly further EMPTY bay beats a nearer busy one.
    BAY_BUSY_PENALTY = 8.0
    REPICK_BAY_AFTER = 6.0      # stuck this long heading for a bay, try another

    def choose_dropoff(self, task: Task, grid: Grid, now: float) -> Optional[Cell]:
        """Which packing station to take this parcel to.

        Only for flexible jobs. Nearest by road, but a bay somebody else is
        using or heading for costs extra -- otherwise every robot picks the
        same nearest bay and they queue for it while the others stand empty.

        Worked out locally from what this robot has heard, like everything
        else here. No dispatcher.
        """
        if not task.flexible:
            return task.dropoff
        bays = grid.cells_of_kind(CellKind.DROP)
        if not bays:
            return task.dropoff

        busy: Dict[Cell, int] = {}
        for note in self.fleet.fresh(now):
            here = Cell(note.cell[0], note.cell[1])
            if here in bays:
                busy[here] = busy.get(here, 0) + 1
            if note.destination:
                heading = Cell(note.destination[0], note.destination[1])
                if heading in bays:
                    busy[heading] = busy.get(heading, 0) + 1

        blocked = self.blocked_map.cells(now)
        best, best_cost = None, float("inf")
        for bay in bays:
            route = self.plan_path(grid, self.cell, bay, now, blocked=blocked)
            if route is None:
                continue
            cost = (len(route) - 1) + self.BAY_BUSY_PENALTY * busy.get(bay, 0)
            if cost < best_cost:
                best, best_cost = bay, cost
        return best if best is not None else task.dropoff

    def cost_of(self, task: Task, grid: Grid, now: float) -> Optional[float]:
        """What this job would cost us. None if we simply cannot reach it."""
        blocked = self.blocked_map.cells(now)
        to_pickup = self.plan_path(
            grid, self.cell, task.pickup, now, blocked=blocked,
        )
        if to_pickup is None:
            return None
        pickup_eta = now + ((len(to_pickup) - 1)
                            / max(self.travel_speed(), 0.1))
        leg = self.plan_path(
            grid, task.pickup, task.dropoff, pickup_eta, blocked=blocked,
        )
        if leg is None:
            return None
        # Phase 13. The question asked in the right place: at the auction.
        # A robot that cannot finish this job AND still reach a charger simply
        # does not bid, so the job goes to somebody who can rather than being
        # abandoned half done.
        if not self.can_finish_and_still_reach_a_charger(
                grid, now, [task.pickup, task.dropoff]):
            return None

        # Phase 23. CRITICAL health means struggling right now -- stuck a
        # long time, or its radio has gone quiet, or it keeps having to
        # replan. Doc 23's own maintenance policy says "reduce new task
        # assignments" before things get worse, so it stops bidding for NEW
        # work. It still finishes whatever it is already carrying: this only
        # ever affects which robot a job goes to, never a robot mid-delivery,
        # and touches nothing about how it drives or gives way.
        if self.health_band == HEALTH_CRITICAL:
            return None

        congestion = self.local_route_congestion(to_pickup, leg, now)
        return bid_cost(
            distance_to_pickup=len(to_pickup) - 1,
            leg_distance=len(leg) - 1,
            battery=self.battery,
            waiting=self.waited_for(now),
            congestion=congestion,
            task_priority=task.priority,
        )

    def local_route_congestion(self, to_pickup: List[Cell],
                               pickup_to_dropoff: List[Cell],
                               now: float) -> float:
        """Route-specific congestion from this robot's local knowledge only.

        One contested route cell counts once even when both intent and
        reservation report the same robot. Unrelated robots and obstacles on
        the other side of the warehouse contribute nothing.
        """
        route = list(to_pickup[1:]) + list(pickup_to_dropoff[1:])
        route_cells = set(route)
        if not route_cells:
            return 0.0

        contested = route_cells.intersection(self.known_occupied(now))

        # Fresh peer intent: only cells intersecting this candidate trip.
        for note in self.fleet.fresh(now):
            intended = {Cell(*pair) for pair in note.planned_nodes}
            intended.add(Cell(*note.cell))
            contested.update(route_cells.intersection(intended))

        # Locally received time reservations near our estimated arrival.
        speed = max(self.travel_speed(), 0.1)
        dwell = 1.0 / speed
        for index, cell in enumerate(route):
            arrival = now + (index + 1) * dwell
            holder = self.table.owner(
                node_key(cell), arrival - self.CLEARANCE,
                arrival + dwell + self.CLEARANCE,
            )
            if holder is not None and holder.robot_id != self.robot_id:
                contested.add(cell)

        return float(len(contested))

    def _take_task(self, task: Task, now: float) -> None:
        # Real work beats a guess, always and immediately.
        self.staging = None
        self.staging_why = ""
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
                chosen = self.choose_dropoff(task, grid, now)
                if chosen is not None:
                    task.dropoff = chosen
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
            elif task.flexible and self.stalled_for(now) > self.REPICK_BAY_AFTER:
                # Stuck on the way to a bay. Try a different one -- exactly the
                # same idea as rerouting, one level up.
                chosen = self.choose_dropoff(task, grid, now)
                if chosen is not None and chosen != task.dropoff:
                    notes.append(f"{self.robot_id} switching to the packing "
                                 f"station at ({chosen.x}, {chosen.y})")
                    task.dropoff = chosen
                    self.set_goal(chosen)

        return notes

    # ------------------------- carrying on without a radio (Phase 14)

    # Heard nothing at all for this long, and we know we are not alone:
    # assume the network has gone.
    NETWORK_TIMEOUT = 3.0

    # A shape that has not left its square for this long is parked, not
    # about to move.
    CONTACT_STILL_AFTER = 1.0

    # We only trust our own judgement about other robots having failed while we
    # have heard SOMETHING this recently. Otherwise the silence is ours.
    LINK_TRUSTED_WITHIN = 1.5

    def sense_robots(self, contacts: Iterable[tuple], now: float) -> None:
        """What this robot can SEE of the others. No messages involved.

        On real hardware this is the laser: another robot reflects a beam
        exactly like a dropped pallet does. The simulator plays the part of the
        scanner and hands over only what is physically within range.

        This is the difference between claiming a local safety reflex and
        having one. Until now the "reflex" read the notebook of things heard on
        the RADIO -- so cutting the network made every robot believe it was
        alone in the warehouse, and they would have driven straight through
        each other. Sensors do not care whether the network is up.
        """
        self.local_contacts = list(contacts)

        # A sensor sees a shape, not a status. Without this, a robot that has
        # simply parked looks exactly like one about to pull out in front of
        # us, so everybody yields to it -- for ever. Watching whether a shape
        # STAYS on the same square tells us the difference, and needs no
        # messages. A moving robot crosses a square in well under a second.
        here = {(c.x, c.y) for (_, _, c) in self.local_contacts}
        for key in list(self._contact_since):
            if key not in here:
                del self._contact_since[key]
        for key in here:
            self._contact_since.setdefault(key, now)

    def sense_humans(self, people: Iterable[tuple], now: float) -> None:
        """What this robot can see of the people nearby. No messages -- the
        same local-sensor pattern as sense_robots(), for the same reason:
        cutting the network must never mean a robot stops noticing a person.

        `people` is (human_id, x, y, cell) tuples already filtered to sensor
        range by the world -- the simulator plays the part of the scanner.

        Two things happen here, and they serve different purposes:
          * `_known_humans` feeds the HARD safety stop and the speed ramp,
            checked fresh every tick, in both fleets;
          * marking the person's square (and its four neighbours) in
            blocked_map feeds ROUTING -- the exact same mechanism a dropped
            box uses, so "reroute around a person" is not new logic, it is
            this old logic fed a new kind of thing to avoid. Marked with a
            short TTL and re-marked every tick they stay in range, so it
            fades within about a second of them moving on -- never broadcast,
            never confused with a real, static obstacle.
        """
        seen_ids = set()
        for human_id, x, y, cell in people:
            seen_ids.add(human_id)
            self._known_humans[human_id] = (x, y, now)
            for nb in (cell, Cell(cell.x + 1, cell.y), Cell(cell.x - 1, cell.y),
                      Cell(cell.x, cell.y + 1), Cell(cell.x, cell.y - 1)):
                self.blocked_map.mark(nb, f"person:{human_id}", now,
                                      ttl=self.PERSON_BLOCK_TTL)
        # Forget anyone no longer in range -- they may just have walked off,
        # not vanished, so this is a plain drop, not a "cleared" report.
        for human_id in list(self._known_humans):
            if human_id not in seen_ids:
                del self._known_humans[human_id]

    def known_occupied(self, now: float) -> List[Cell]:
        """Squares that have a robot on them, from BOTH sources.

        Everything that needs to route around other robots must use this, not
        just the radio. Getting that wrong meant a robot in a blackout would
        replan straight through the robot blocking it, conclude the new route
        was the same as the old one, and wait for ever.
        """
        cells = {Cell(n.cell[0], n.cell[1]) for n in self.fleet.fresh(now)}
        for x, y, cell in self.local_contacts:
            cells.add(cell)
            # A robot can fail between cell centres. Once sensors confirm the
            # body is stationary, also mark every neighbouring centre inside
            # its physical safety radius. Otherwise A* repeatedly chooses a
            # technically different cell that the safety brake can never let
            # this robot enter.
            since = self._contact_since.get((cell.x, cell.y), now)
            if now - since < self.CONTACT_STILL_AFTER:
                continue
            for neighbour in (
                    Cell(cell.x + 1, cell.y), Cell(cell.x - 1, cell.y),
                    Cell(cell.x, cell.y + 1), Cell(cell.x, cell.y - 1)):
                gap_x, gap_y = x - neighbour.x, y - neighbour.y
                if gap_x * gap_x + gap_y * gap_y < self.SAFE_GAP ** 2:
                    cells.add(neighbour)
        return list(cells)

    def link_quiet_for(self, now: float) -> float:
        """Seconds since ANY message arrived from anybody."""
        if self.last_heard_any < 0:
            return 0.0
        return max(0.0, now - self.last_heard_any)

    def update_link_health(self, now: float) -> Optional[str]:
        """Decide whether we are on our own, and say so when it changes.

        01_PRODUCT_AND_SYSTEM_DESIGN section 7:
            Normal network      -> decentralised coordination
            Network degraded    -> local coordination / reduced speed
            Network unavailable -> local safety mode
            Network restored    -> state synchronisation
        """
        lonely = (self._ever_heard
                  and self.link_quiet_for(now) > self.NETWORK_TIMEOUT)

        if lonely and not self.safe_mode:
            self.safe_mode = True
            if self.status is not RobotStatus.FAILED:
                self.status = RobotStatus.SAFE_MODE
            return (f"{self.robot_id} has lost the network - SAFE MODE, "
                    f"half speed, finishing the job it is holding")

        if not lonely and self.safe_mode:
            self.safe_mode = False
            self._resync_pending = True
            if self.status is RobotStatus.SAFE_MODE:
                self.status = RobotStatus.MOVING if self.path else RobotStatus.IDLE
            return f"{self.robot_id} is back on the network - resyncing"

        return None

    def _resync(self, bus, now: float) -> None:
        """Tell everyone everything again, after being out of touch.

        03_ROBOT_AND_ROS2 section 9: SYNC STATE, SYNC TASK, SYNC RESERVATIONS,
        RESUME. Nothing clever -- the periodic broadcasts are simply forced to
        go out at once instead of waiting for their next turn.
        """
        self._resync_pending = False
        self._next_heartbeat = 0.0
        self._next_pose = 0.0
        self._last_intent_key = None
        self._last_wait_report = None
        for res in list(self._claimed.values()):
            self._broadcast_reservation(bus, now, "CLAIM", res)
        if self.task is not None and bus is not None:
            self._publish(bus, TaskClaim(
                robot_id=self.robot_id, timestamp=now, seq=self.seq,
                task_id=self.task.task_id, action="CLAIM",
                cost=self._my_bids.get(self.task.task_id, 0.0)))

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
                                  ttl=DEFAULT_TTL,
                                  confidence=message.confidence)

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

        # Permanent obstacles do not disappear merely because nobody has
        # revisited the aisle. Finite-lived safety blocks (for example a
        # moving person) may still expire through this shared map.
        self.blocked_map.expire(now)

        return news

    def blocked_cells(self, now: float):
        return self.blocked_map.cells(now)

    # ------------------------------ breaking a hopeless jam (Phase 7)

    ASK_COOLDOWN = 3.0          # do not pester the same robot every tick

    # Two robots each blocked by the other cannot possibly clear on their own,
    # so there is nothing to be gained by waiting the full STUCK_SECONDS to
    # find out. Waiting anyway is what made a pair sit nose to nose for four
    # seconds, which to anyone watching looks like the system has frozen.
    # Longer chains still get the full patience -- those often do clear.
    MUTUAL_STUCK_SECONDS = 1.0
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
        stuck = max(self.waited_for(now), self.stalled_for(now))
        if self.blocked_by is None:
            return None
        blocker = self.blocked_by

        # Is this a straight two-robot standoff -- me behind them, them behind
        # me? That can never sort itself out, so act on it quickly.
        theirs = self.waits.blocker_of(blocker)
        head_on = theirs == self.robot_id
        patience = self.MUTUAL_STUCK_SECONDS if head_on else STUCK_SECONDS
        if stuck < patience:
            return None

        last_asked = self._asked_at.get(blocker)
        if last_asked is not None and now - last_asked < self.ASK_COOLDOWN:
            return None

        # Our own edge is not in the graph -- the graph is built from what
        # OTHER robots said. Add ourselves, or the search starts nowhere and a
        # simple two-robot standoff is invisible.
        self.waits.add(Waiting(
            robot_id=self.robot_id, blocked_by=blocker,
            since=stuck, priority=self.priority, is_moving=False,
        ))
        cycle = self.waits.cycle_containing(self.robot_id)
        if cycle:
            members = []
            for rid in cycle:
                if rid == self.robot_id:
                    members.append((rid, self.priority, stuck))
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
        blocked.update(self.known_occupied(now))
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
        rate = self.DRAIN_MOVING if moving else self.DRAIN_IDLE
        self.battery = max(0.0, self.battery - rate * dt)

    # --------------------------------------------------- Phase 13: charging

    def percent_per_square(self) -> float:
        """What one square of driving costs in charge."""
        return self.DRAIN_MOVING / max(self.speed, 0.1)

    def charge_urgency(self) -> int:
        """How badly we want a charger, as a booking priority.

        The emptier the robot, the higher. Rounded into whole 5% bands on
        purpose: every robot holds a slightly stale copy of everyone else's
        battery, and a comparison that turns on a fraction of a percent lets
        two robots each decide they won the bay. Robots must differ by a clear
        band before one outranks the other, and below that the lower robot ID
        decides -- a name cannot go stale. Same rule as giving way.
        """
        empty = max(0.0, 100.0 - self.battery)
        return int(empty / self.BATTERY_BAND) * SCALE

    def _walk(self, grid: Grid, blocked, start: Cell, stops,
              now: float) -> Optional[float]:
        """Squares driven going from start through each stop in turn."""
        total, here = 0.0, start
        for stop in stops:
            leg = self.plan_path(grid, here, stop, now, blocked=blocked)
            if leg is None:
                return None
            total += len(leg) - 1
            now += (len(leg) - 1) / max(self.travel_speed(), 0.1)
            here = stop
        return total

    def _nearest_charger(self, grid: Grid, blocked, start: Cell, now: float):
        """(squares, cell) of the closest charging bay, ignoring who booked it."""
        best = None
        for bay in grid.cells_of_kind(CellKind.CHARGER):
            leg = self.plan_path(grid, start, bay, now, blocked=blocked)
            if leg is None:
                continue
            if best is None or len(leg) - 1 < best[0]:
                best = (len(leg) - 1, bay)
        return best

    def can_finish_and_still_reach_a_charger(self, grid: Grid, now: float,
                                             stops) -> bool:
        """THE question, asked before taking on work and again while doing it.

        Not "am I below 20%" -- a fixed line is wrong in both directions. It is
        "if I drive this whole job and then drive on to the nearest charging
        bay, do I still arrive with something in hand?"
        """
        blocked = self.blocked_map.cells(now)
        work = self._walk(grid, blocked, self.cell, stops, now)
        if work is None:
            return False
        end = stops[-1] if stops else self.cell
        arrival = now + work / max(self.travel_speed(), 0.1)
        bay = self._nearest_charger(grid, blocked, end, arrival)
        if bay is None:
            return True                # no chargers on this map, carry on
        need = (work + bay[0]) * self.percent_per_square() + self.BATTERY_RESERVE
        return self.battery >= need

    def _should_charge(self, grid: Grid, now: float) -> bool:
        """Use separate working and idle thresholds with hysteresis."""
        if now - self._charge_checked_at < self.CHARGE_RECHECK:
            return self._charge_wanted
        self._charge_checked_at = now

        # Any robot below 15% must charge, including one carrying a parcel.
        if self.battery < self.CHARGE_BELOW:
            self._charge_wanted = True
            return True

        # A genuinely idle robot may use spare charger capacity, but above 80%
        # it stays available for work instead of repeatedly topping itself up.
        if (self.task is None and self.goal is None
                and self.status is RobotStatus.IDLE):
            self._charge_wanted = self.battery <= self.IDLE_CHARGE_AT
            return self._charge_wanted

        self._charge_wanted = False
        return False

    def _book_charger(self, grid: Grid, bus, now: float) -> Optional[str]:
        """Take the nearest free bay, or keep one fixed queue assignment."""
        blocked = self.blocked_map.cells(now)
        occupied = set(self.known_occupied(now))
        top_up = max(0.0, self.CHARGE_UNTIL - self.battery) / self.CHARGE_RATE
        free_best = None
        queue_best = None
        for bay in grid.cells_of_kind(CellKind.CHARGER):
            leg = self.plan_path(grid, self.cell, bay, now, blocked=blocked)
            if leg is None:
                continue
            squares = len(leg) - 1
            travel = squares / max(self.travel_speed(), 0.1)
            start, end = now, now + travel + top_up + self.CLEARANCE
            key = node_key(bay)
            owner = self.table.owner(key, start, end)
            lease_owner = self.table.owner(target_key(bay), start, end)
            taken = ((bay != self.cell and bay in occupied)
                     or (owner is not None and owner.robot_id != self.robot_id)
                     or (lease_owner is not None
                         and lease_owner.robot_id != self.robot_id))
            candidate = (squares, bay, start, end)
            if taken:
                if queue_best is None or squares < queue_best[0]:
                    queue_best = candidate
            elif free_best is None or squares < free_best[0]:
                free_best = candidate

        # Free always wins, even when an occupied charger is closer. If every
        # bay is taken, remember one queue assignment and do not reconsider a
        # different charger every tick.
        best = free_best or queue_best
        if best is None:
            return None
        queued = free_best is None

        squares, bay, start, end = best
        # Not picked up yet? Put the job back so somebody with charge takes it.
        handed_back = None
        if self.task is not None and self.task.status is TaskStatus.ASSIGNED:
            handed_back = self.task.task_id
            self._drop_task()
            if bus is not None:
                self._publish(bus, TaskClaim(
                    robot_id=self.robot_id, timestamp=now, seq=self.seq,
                    task_id=handed_back, action="RELEASE"))

        self.charger = bay
        self._charge_slot = None if queued else (start, end)
        self.staging = None
        self.staging_why = ""
        self.set_goal(bay)
        action = "queued for" if queued else "-> charging bay"
        return (f"{self.robot_id} is low ({self.battery:.0f}%) {action} "
                f"({bay.x}, {bay.y})"
                + (f", gave {handed_back} back" if handed_back else ""))

    def _leave_charger(self, bus, now: float) -> str:
        bay = self.charger
        self.charger = None
        self._charge_slot = None
        self._charge_checked_at = -99.0
        self._charge_wanted = False
        if bay is not None:
            for key in (node_key(bay), target_key(bay)):
                self.table.release(self.robot_id, key)
                old = self._claimed.pop(key, None)
                if old is not None:
                    self._broadcast_reservation(bus, now, "RELEASE", old)
        # Only "idle" if it is actually standing on a square. Part way along a
        # step it is stopped, not idle, and saying otherwise used to snap it
        # backwards onto the square behind it.
        self.status = (RobotStatus.IDLE if self._progress == 0.0
                       else RobotStatus.WAITING)
        return f"{self.robot_id} is charged ({self.battery:.0f}%) and back on the job"

    # --------------------------------------------- Phase 23: how it is doing

    HEALTH_RECHECK = 2.0     # seconds between recomputing the score

    def update_health(self, now: float) -> Optional[str]:
        """Recompute the health score from what this robot has actually
        measured about itself. Returns a note only when the BAND changes, so
        the event feed says something when it matters and stays quiet the
        rest of the time.
        """
        if now - self._last_health_check < self.HEALTH_RECHECK:
            return None
        self._last_health_check = now

        cutoff = now - INCIDENT_WINDOW
        while self._reroute_times and self._reroute_times[0] < cutoff:
            self._reroute_times.popleft()
        while self._backout_times and self._backout_times[0] < cutoff:
            self._backout_times.popleft()

        report = compute_health(
            battery=self.battery,
            stalled_for=self.stalled_for(now) if self.goal is not None else 0.0,
            comms_silence=self.link_quiet_for(now),
            reroutes_recent=len(self._reroute_times),
            backouts_recent=len(self._backout_times),
        )
        was = self.health_band
        self.health_score, self.health_band, self.health_reasons = (
            report.score, report.band, report.reasons)
        if self.health_band == was:
            return None
        arrow = "v" if BANDS.index(self.health_band) > BANDS.index(was) else "^"
        return (f"{self.robot_id} health {was} -> {self.health_band} "
                f"({self.health_score:.0f}/100) {arrow} {report.sentence()}")

    def manage_battery(self, grid: Grid, bus, now: float) -> Optional[str]:
        """Decide about charge. Called once per tick, before anything moves."""
        if self.status is RobotStatus.FAILED or self.halted:
            return None

        if self.status is RobotStatus.CHARGING:
            if self.battery >= self.CHARGE_UNTIL:
                return self._leave_charger(bus, now)
            return None

        if (self.charger is not None and self.cell == self.charger
                and self._progress == 0.0):
            self.status = RobotStatus.CHARGING
            self.goal = None
            self.path = []
            return (f"{self.robot_id} reached the bay on {self.battery:.0f}% "
                    f"and is charging")

        if self.charger is not None:
            # Somebody else has pointed us somewhere else -- the patrol loop
            # does exactly this to any robot that is briefly without a
            # destination. Going flat because scaffolding overwrote the trip to
            # the charger is not acceptable, so say again where we are going.
            if self.goal != self.charger:
                self.set_goal(self.charger)

            # Queued robots keep the charger they chose. When that exact bay
            # becomes free, promote the queue assignment into a real booking;
            # never rerun nearest-charger selection and jump to another bay.
            if self._charge_slot is None:
                key = node_key(self.charger)
                occupied = (self.charger != self.cell
                            and self.charger in set(self.known_occupied(now)))
                top_up = max(0.0, self.CHARGE_UNTIL - self.battery) / self.CHARGE_RATE
                leg = self.plan_path(
                    grid, self.cell, self.charger, now,
                    blocked=self.blocked_map.cells(now),
                )
                if leg is None:
                    return None
                travel = (len(leg) - 1) / max(self.travel_speed(), 0.1)
                start, end = now, now + travel + top_up + self.CLEARANCE
                owner = self.table.owner(key, start, end)
                lease_owner = self.table.owner(
                    target_key(self.charger), start, end,
                )
                if (occupied
                        or (owner is not None
                            and owner.robot_id != self.robot_id)
                        or (lease_owner is not None
                            and lease_owner.robot_id != self.robot_id)):
                    return None
                self._charge_slot = (start, end)
                return (f"{self.robot_id}'s queued bay "
                        f"({self.charger.x}, {self.charger.y}) is free")

            # Still on our way. Check we have not been outbid in the meantime.
            #
            # Every robot decides in the same tick, before anybody has heard
            # anybody, so ten robots can each book the same four bays in the
            # same instant -- and they did, and then all ten drove at four
            # squares and wedged. Booking is only half of it: you have to keep
            # listening, and stand down when you hear that somebody emptier
            # than you wanted the bay. That is the whole point of sharing the
            # table rather than asking a server.
            key = node_key(self.charger)
            start, end = self._charge_slot
            mine = Reservation(robot_id=self.robot_id, resource=key,
                               start=start, end=end,
                               priority=self.charge_urgency())
            owner = self.table.owner(key, start, end)
            target_owner = self.table.owner(
                target_key(self.charger), start, end,
            )
            winner = min(
                (claim for claim in (owner, target_owner)
                 if claim is not None and claim.robot_id != self.robot_id),
                key=lambda claim: claim.rank,
                default=None,
            )
            if winner is not None and winner.rank < mine.rank:
                bay = self.charger
                self._leave_charger(bus, now)
                return (f"{self.robot_id} stood down from bay "
                        f"({bay.x}, {bay.y}) for {winner.robot_id}, who is emptier")
            return None

        if not self._should_charge(grid, now):
            return None
        return self._book_charger(grid, bus, now)

    # ---------------------------------------------------------------- output

    def to_dict(self, now: float = 0.0) -> Dict[str, object]:
        """Plain data about this robot. Dicts and numbers only -- no web code."""
        return {
            "robot_id": self.robot_id,
            "halted": self.halted,
            "charging": self.status is RobotStatus.CHARGING,
            "health_score": round(self.health_score, 1),
            "health_band": self.health_band,
            "health_reasons": self.health_reasons,
            "messages_rejected": self.messages_rejected,
            "near_person": self.near_person,
            "staging": [self.staging.x, self.staging.y] if self.staging else None,
            "staging_why": self.staging_why,
            "charger": [self.charger.x, self.charger.y] if self.charger else None,
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
            # Just the count. The full list of every robot's conflicts was
            # three quarters of the robot payload and the dashboard never read
            # it -- it uses the merged list the world publishes instead.
            "conflict_count": len(self.conflicts),
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
