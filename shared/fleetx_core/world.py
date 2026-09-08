"""The whole warehouse: the map, every robot, and the clock.

PURE LOGIC ONLY. No web code, no ROS 2 code.

This is the thing the dashboard watches and the benchmark measures. In Part 2
the ROS 2 gateway will build the same object from live robot messages, so the
dashboard code does not change at all.
"""

import time
from typing import Callable, Dict, List, Optional

from .bus import FleetBus, InMemoryBus
from .conflicts import DEFAULT_HORIZON, Conflict
from .deadlock import STUCK_SECONDS
from .obstacles import SENSOR_RANGE, within_range
from .messages import TaskAnnounce, TaskClaim
from .tasks import Task, TaskBoard, TaskStatus
from .reservations import Reservation
from .grid import Cell, CellKind, Grid, default_grid
from .robot import Robot, RobotStatus

# Two robots closer together than this (in squares) are touching.
# Each robot is drawn about 0.68 squares wide, so this is roughly the moment
# their bodies overlap on screen.
COLLISION_DISTANCE = 0.7

# How many recent crashes to remember for the dashboard's event feed.
_MAX_EVENTS = 40


class World:
    """Everything happening on the warehouse floor right now."""

    def __init__(self, grid: Optional[Grid] = None, bus: Optional[FleetBus] = None,
                 reservations_enabled: bool = True,
                 negotiation_enabled: bool = True,
                 coordination: str = "FLEETX"):
        self.grid: Grid = grid if grid is not None else default_grid()
        # The group chat the robots talk over. Swap this for a ROS 2 bus and
        # the robots do not notice the difference.
        self.bus: FleetBus = bus if bus is not None else InMemoryBus()
        self.robots: Dict[str, Robot] = {}
        # Turn booking off to get the old, blind behaviour back. Phase 15 uses
        # this to run the "before" side of the benchmark on the same code.
        self.reservations_enabled: bool = reservations_enabled
        # Turn negotiation off to get Phase 5 behaviour back: the loser of a
        # booking just waits, instead of going around.
        self.negotiation_enabled: bool = negotiation_enabled

        # "FLEETX"        -- intent, booking, negotiation, deadlock breaking
        # "STOP_AND_WAIT" -- the baseline: see a robot, stop. Nothing else.
        # Everything OUTSIDE this setting is identical between the two.
        self.coordination: str = coordination
        self.sim_time: float = 0.0     # seconds since the run started
        self.ticks: int = 0
        self.started_at: float = time.time()

        # The number that has to reach zero. See 00_README "Success metrics".
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
        self.deadlock_enabled: bool = True
        self.deadlocks_broken: int = 0
        if self.coordination == "STOP_AND_WAIT":
            # The baseline has none of the clever parts. Only the traffic rule
            # differs -- the map, A*, the speed and the job auction are shared.
            self.reservations_enabled = False
            self.negotiation_enabled = False
            self.deadlock_enabled = False

        # --- Phase 8: things that should not be there ---
        # The TRUTH about what is on the floor. Robots cannot read this. They
        # only learn about it by driving close enough to see it -- see
        # _run_sensors below. That is what makes finding one worth broadcasting.
        self.obstacles: Dict[Cell, float] = {}
        self.obstacle_reports: int = 0

        # --- Phase 9: real jobs ---
        # The order book. Orders come from OUTSIDE the fleet -- somebody bought
        # something. Who does each job is settled by the robots between
        # themselves, which is the part that has to survive the server dying.
        self.board: TaskBoard = TaskBoard()
        self._task_seq: int = 0

        # Which pairs of robots are touching RIGHT NOW. Used so that one crash
        # is counted once, instead of once per tick while they overlap.
        self._touching: set = set()
        self._event_id: int = 0

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
        """Robots that are switched on. A FAILED robot cannot be crashed into."""
        return [r for r in self.robots.values() if r.status is not RobotStatus.FAILED]

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

        for robot in self.robots.values():
            robot.communicate(self.bus, self.sim_time)

        # Phase 9: bid for jobs, claim what we win, get on with them.
        for robot in self.robots.values():
            for note in robot.work_on_tasks(self.grid, self.bus, self.sim_time):
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]
        self._sync_board()

        for robot in self.robots.values():
            robot.decide(self.grid, cost_fn, self.sim_time)

        # Phase 4: every robot looks at its own plan against everyone else's.
        # It only LOOKS. Phase 5 books the square, Phase 6 decides who yields.
        for robot in self.robots.values():
            robot.check_conflicts(self.grid, self.sim_time, DEFAULT_HORIZON)
            robot.announce_conflicts(self.bus, self.sim_time)
        self._record_warnings()
        self._expire_warnings()

        # Phase 5: book the squares ahead, then check we are allowed onto the
        # next one. A robot that is not the owner stops before it.
        if self.coordination == "STOP_AND_WAIT":
            for robot in self.robots.values():
                robot.stop_and_wait_check(self.sim_time, dt)
            for robot in self.robots.values():
                note = robot.resume_after_yielding(self.sim_time)
                if note is None:
                    note = robot.stop_and_wait_backoff(self.grid, self.sim_time)
                if note:
                    self.decisions.append({
                        "sim_time": round(self.sim_time, 2),
                        "robot_id": robot.robot_id, "text": note,
                    })
                    del self.decisions[:-20]
            self._watch_for_jams()
        elif self.reservations_enabled:
            # Phase 6: standing is recomputed first -- the longer a robot has
            # been stuck, the more it is owed (05_PATH_PLANNING section 9).
            for robot in self.robots.values():
                robot.update_priority(self.sim_time, dt)
            for robot in self.robots.values():
                robot.reserve_ahead(self.bus, self.sim_time)
            for robot in self.robots.values():
                robot.check_clearance(self.sim_time, dt)
            # Phase 6: anyone held up decides whether to wait or go around.
            if self.negotiation_enabled:
                for robot in self.robots.values():
                    if robot.consider_reroute(self.grid, self.sim_time):
                        self.reroutes += 1
                        self.decisions.append({
                            "sim_time": round(self.sim_time, 2),
                            "robot_id": robot.robot_id,
                            "text": robot.last_decision,
                        })
                        del self.decisions[:-20]
            # Phase 7: anyone stuck far too long works out WHY, and either
            # asks the robot in front to move or gets out of the way itself.
            if self.deadlock_enabled:
                for robot in self.robots.values():
                    note = robot.answer_requests(self.grid, self.sim_time)
                    if note is None:
                        note = robot.resume_after_yielding(self.sim_time)
                    if note is None:
                        note = robot.report_jam(self.grid, self.bus, self.sim_time)
                    if note:
                        # Only count it as BROKEN when a robot actually moved.
                        # Counting every request would report 37 "recoveries"
                        # for one jam that never cleared.
                        if "stepped aside" in note or "moving aside" in note:
                            self.deadlocks_broken += 1
                        self.decisions.append({
                            "sim_time": round(self.sim_time, 2),
                            "robot_id": robot.robot_id, "text": note,
                        })
                        del self.decisions[:-20]

            self._watch_for_jams()
        elif self.coordination != "STOP_AND_WAIT":
            for robot in self.robots.values():
                robot.hold = False
                robot.blocked_by = None
                if robot.status is RobotStatus.WAITING and robot.path:
                    robot.status = RobotStatus.MOVING

        # Remember where everyone was, so we can spot two robots swapping places.
        was_at: Dict[str, Cell] = {rid: r.cell for rid, r in self.robots.items()}

        for robot in self.robots.values():
            robot.advance(dt)

        self._detect_collisions(was_at)

        self.sim_time += dt
        self.ticks += 1

    # -------------------------------------------- real jobs (Phase 9)

    def announce_task(self, pickup: Cell, dropoff: Cell, product: str = "",
                      priority: int = 5) -> Task:
        """Put a new job on the air. Nobody is told who should do it."""
        self._task_seq += 1
        task = Task(
            task_id=f"T-{self._task_seq:03d}",
            pickup=pickup, dropoff=dropoff, product=product, priority=priority,
            created_at=self.sim_time, announced_at=self.sim_time,
            status=TaskStatus.ANNOUNCED,
        )
        self.board.add(task)
        self.bus.publish(TaskAnnounce(
            robot_id="ORDERS", timestamp=self.sim_time, seq=self._task_seq,
            task_id=task.task_id,
            pickup=(pickup.x, pickup.y), dropoff=(dropoff.x, dropoff.y),
            product=product, priority=priority,
        ))
        return task

    def _sync_board(self) -> None:
        """Keep the dashboard's copy of the board in step with the robots'.

        The dashboard is an observer. 02_ARCHITECTURE section 3: it watches,
        it does not make the safety decisions -- and here it does not hand out
        the work either.
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

    def fail_robot(self, robot_id: str) -> Dict[str, object]:
        """Switch a robot off mid-job. 08_SIMULATION Scenario 6."""
        robot = self.get(robot_id)
        if robot is None:
            return {"ok": False, "message": f"There is no robot called {robot_id}."}
        robot.status = RobotStatus.FAILED
        robot.release_all(self.bus, self.sim_time)
        held = self.board.release_all(robot_id)
        # Tell everyone, or the other robots keep the job down as "R2's" and
        # nobody ever picks it up. 06_TASK_ALLOCATION section 4.
        for task in held:
            self.bus.publish(TaskClaim(
                robot_id=robot_id, timestamp=self.sim_time, seq=0,
                task_id=task.task_id, action="RELEASE"))
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

            seen_blocked = [c for c in self.obstacles
                            if within_range(robot.x, robot.y, c, SENSOR_RANGE)]

            # Squares it believes are blocked, can see plainly, and are empty.
            seen_clear = [c for c in robot.blocked_cells(self.sim_time)
                          if c not in self.obstacles
                          and within_range(robot.x, robot.y, c, SENSOR_RANGE)]

            for note in robot.sense(seen_blocked, seen_clear, self.sim_time):
                self.obstacle_reports += 1
                self.decisions.append({
                    "sim_time": round(self.sim_time, 2),
                    "robot_id": robot.robot_id, "text": note,
                })
                del self.decisions[:-20]

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
            stuck = robot.waited_for(self.sim_time)
            if stuck >= self.deadlock_after:
                if robot.robot_id not in self._deadlocked:
                    self._deadlocked.add(robot.robot_id)
                    self.deadlocks_seen += 1
            elif robot.waiting_since is None:
                self._deadlocked.discard(robot.robot_id)

    def stuck_robots(self) -> List[str]:
        return sorted(self._deadlocked)

    # ----------------------------------------------------------- collisions

    def _detect_collisions(self, was_at: Dict[str, Cell]) -> None:
        """Count crashes honestly.

        Three ways two robots can crash, and we check all three:

        1. SAME SQUARE   -- both standing on the same square.

        2. HEAD-ON SWAP  -- they drove straight through each other and traded
                            places. They never share a square, so checking
                            squares alone would MISS this. 05_PATH_PLANNING §5
                            calls it an edge conflict: "Do not check only nodes."

                                R1 -> <- R2      becomes      <- R2   R1 ->

        3. TOUCHING      -- their bodies overlap part-way between squares.
                            This is the one your eyes see on the dashboard.

        One crash counts ONCE. The world ticks 20 times a second, so without
        this the same crash would be counted 20 times and the benchmark would
        be nonsense.
        """
        robots = self.active_robots
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
        self.board = TaskBoard()
        self._task_seq = 0
        self.sim_time = 0.0
        self.ticks = 0
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
            "tasks_completed": self.board.stats(self.sim_time)["done"],
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
            **{f"task_{k}": v for k, v in self.board.stats(self.sim_time).items()},
            "blocked_known": len(set().union(*[r.blocked_cells(self.sim_time)
                                               for r in robots]) if robots else set()),
            "yields": sum(r.yields for r in robots),
            "avg_priority": (round(sum(r.priority for r in robots) / len(robots) / 10.0, 1)
                             if robots else 0.0),
            "avg_warning": (round(sum(self.warning_times) / len(self.warning_times), 2)
                            if self.warning_times else 0.0),
        }

    # ---------------------------------------------------------------- output

    def snapshot(self) -> Dict[str, object]:
        """Everything the dashboard needs, as plain data."""
        return {
            "sim_time": round(self.sim_time, 2),
            "ticks": self.ticks,
            "robots": [r.to_dict(self.sim_time) for r in self.robots.values()],
            "collision_events": self.collision_events[-12:],
            "conflicts": self.active_conflicts(),
            "views": {
                rid: dict(r.fleet.to_dict(self.sim_time),
                          table=r.table.rows(self.sim_time) if r.table else [],
                          holds=[[c.x, c.y] for c in r.table.held_nodes(
                              rid, self.sim_time, self.sim_time + 3.0)] if r.table else [],
                          blocked=r.blocked_map.to_rows(self.sim_time) if r.blocked_map else [])
                for rid, r in self.robots.items() if r.fleet
            },
            "stuck": self.stuck_robots(),
            "obstacles": [[c.x, c.y] for c in self.obstacles],
            "tasks": self.board.rows(),
            "wait_graph": (list(self.robots.values())[0].waits.to_rows()
                           if self.robots else []),
            "decisions": self.decisions[-8:][::-1],
            "bus": self.bus.stats() if hasattr(self.bus, "stats") else {},
            "messages": [
                m.to_dict() for m in list(getattr(self.bus, "recent", []))[-14:]
            ][::-1],
            "kpis": self.kpis(),
        }


# --------------------------------------------------------------- setups


def phase1_world() -> World:
    """Phase 1: one warehouse, one robot, parked at a pick station.

    Roadmap Phase 1 asks for exactly one thing: R1 can move from A to B.
    """
    world = World()
    start = world.grid.cells_of_kind(CellKind.PICK)[0]
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

    08_SIMULATION section 3 asks for 3, 5, 10 and 20 robots. More robots on the
    same floor means more contention, which is the whole point: coordination
    only earns its keep when robots actually get in each other's way.
    """
    world = World(grid=grid, **kwargs)
    if robots > len(FLEET_START_CELLS):
        raise ValueError(f"No start position for more than {len(FLEET_START_CELLS)} robots.")
    for i in range(robots):
        world.add_robot(Robot(robot_id=f"R{i + 1}", cell=FLEET_START_CELLS[i]))
    return world


def phase2_world() -> World:
    """Phase 2: three robots, each completely unaware of the other two.

    Roadmap Phase 2 asks for: R1, R2, R3 navigating simultaneously.

    There is deliberately NO collision handling here. These robots will drive
    straight through each other, and the dashboard will count it. That count is
    the "before" picture that Phases 3-7 have to drive down to zero.

    Note this is NOT the stop-and-wait baseline for the 20% speed target --
    that gets built separately in Phase 15.
    """
    world = World()
    world.add_robot(Robot(robot_id="R1", cell=Cell(2, 8)))
    world.add_robot(Robot(robot_id="R2", cell=Cell(26, 8)))
    world.add_robot(Robot(robot_id="R3", cell=Cell(13, 0)))
    return world
