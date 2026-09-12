"""PART 1 and PART 2 -- the rival this project measures itself against.

26_IMPROVEMENTS_AND_UPGRADES calls this "the one experiment that directly
answers 'why not centralized?' with data instead of an argument," and it is
right: FLEET-X being faster than stop-and-wait proves the ALGORITHM works. It
does not prove the ARCHITECTURE was necessary -- a centralised planner could
run a similar algorithm and plausibly do just as well, AS LONG AS ITS LINK TO
THE ROBOTS STAYS UP.

That "as long as" is the entire case for decentralisation, and this file is
built to test it honestly rather than assert it. CentralPlanner is not a
strawman: it sees the whole warehouse at once and plans real, conflict-free
routes, using the SAME A* and the SAME kind of reservation table FLEET-X's own
robots use to book a square -- just computed by ONE ENTITY with PERFECT
information, instead of negotiated between many robots each working from a
partial, sometimes-stale picture. Assigning work uses the SAME cost
calculation (Robot.cost_of) FLEET-X's own auction uses; there is no separate,
weaker version of that logic for the rival to lose to.

The one thing it does NOT have -- by design, because this is what "central"
actually means -- is any local intelligence on the robot end. A robot under
central control does not plan, does not reroute, does not negotiate, does not
bid. It follows the path it was last told, with exactly the same local safety
reflex any real AMR has regardless of architecture (it will not run into
something in front of it -- see Robot.stop_and_wait_check, reused verbatim).
Everything else -- which way to go, what happens next -- comes from the boss,
over the SAME simulated radio every other message in this project travels
over, subject to the SAME packet loss and the SAME "Cut the network" button.
Cut it, and a central-mode robot has nothing left to decide with. FLEET-X's
robots keep exactly the intelligence they always had, because they never
borrowed it from anywhere else.

Even the boss's own knowledge of a robot's progress travels this way. It does
not peek at a robot's private state to find out it has picked up a parcel --
it has to be TOLD, over the same link, the same way a real control room finds
out what is happening on the floor. Lose that message and the boss genuinely
does not know the delivery leg is due; nothing here fakes the failure.

Pure Python. No web, no ROS 2.
"""

from typing import Dict, List, Optional, Tuple

from .astar import find_path
from .grid import Cell
from . import security
from .security import ReplayGuard
from .messages import CentralCommand, TaskClaim
from .reservations import Reservation, ReservationTable, avoidance_cost, node_key
from .robot import RobotStatus
from .tasks import Task, TaskStatus


class CentralPlanner:
    """One boss computer, planning every robot's route from one place."""

    BOSS_ID = "BOSS"
    RESEND_EVERY = 2.0        # how often an unconfirmed command is repeated --
                               # the same idea as re-announcing a forgotten
                               # order, because the link it travels over can
                               # lose a message just as easily as any other

    def __init__(self, bus) -> None:
        self.bus = bus
        if bus is not None and hasattr(bus, "register"):
            bus.register(self.BOSS_ID)
        self.table = ReservationTable()
        self._seq = 0
        self._replay_guard = ReplayGuard()

        # robot_id -> (task_id, leg, path) -- what the boss currently believes
        # this robot should be doing. Resent on a timer until superseded.
        self._current: Dict[str, Tuple[str, str, Tuple[Tuple[int, int], ...]]] = {}
        self._last_sent: Dict[str, float] = {}

        # for the dashboard and the benchmark to report honestly
        self.commands_sent = 0

    # ---------------------------------------------------------------- I/O

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _listen(self, world, now: float) -> None:
        """Learn what has actually happened, from messages -- never by
        reaching into a robot's own state. The same authentication and
        replay protection every other participant on the bus gets."""
        if self.bus is None:
            return
        for message in self.bus.poll(self.BOSS_ID):
            if not security.check_signature(message):
                continue
            sender = getattr(message, "sender", "") or getattr(message, "robot_id", "")
            fresh = self._replay_guard.check(
                sender, getattr(message, "seq", 0),
                getattr(message, "timestamp", 0.0), now,
                stream=str(getattr(message, "type", type(message).__name__)))
            if not fresh:
                continue

            if isinstance(message, TaskClaim):
                task = world.board.get(message.task_id)
                if task is None:
                    continue
                if message.action == "PICKED":
                    task.status = TaskStatus.CARRYING
                    task.picked_at = now
                elif message.action == "DONE":
                    task.status = TaskStatus.DONE
                    task.done_at = now
                    self._current.pop(message.robot_id, None)
                    self._last_sent.pop(message.robot_id, None)

    def _publish(self, world, robot_id: str, now: float) -> None:
        """Send (or resend) whatever the boss currently believes this robot
        should be doing. One method for both cases, because a resend is not
        a different message -- it is the same decision, said again, because
        the boss cannot tell "never arrived" from "arrived, just slow"."""
        if self.bus is None or robot_id not in self._current:
            return
        task_id, _leg, path = self._current[robot_id]
        task = world.board.get(task_id)
        if task is None:
            self._current.pop(robot_id, None)
            self._last_sent.pop(robot_id, None)
            return
        cmd = CentralCommand(
            robot_id=robot_id, timestamp=now, seq=self._next_seq(),
            task_id=task_id,
            pickup=(task.pickup.x, task.pickup.y),
            dropoff=(task.dropoff.x, task.dropoff.y),
            product=task.product,
            path=[list(c) for c in path],
        )
        security.seal(cmd)
        self.bus.publish(cmd)
        self._last_sent[robot_id] = now
        self.commands_sent += 1

    # -------------------------------------------------------- the decision

    def _plan_route(self, world, robot, target: Cell, now: float) -> Optional[List[Cell]]:
        """A real route, computed with perfect information, steered away
        from everything this ONE table already has booked -- the same idea
        as FLEET-X's avoidance_cost, just applied by one planner instead of
        negotiated between many."""
        cost_fn = avoidance_cost(self.table, robot.robot_id, now)
        route = find_path(world.grid, robot.cell, target, cost_fn)
        if route is None:
            return None
        return route[1:]

    def _book(self, robot_id: str, path: List[Cell], now: float, speed: float) -> None:
        """Reserve the whole route at once -- the boss plans the entire trip
        in one go, not square by square, because it is not limited to a
        robot's own short lookahead the way a negotiated plan is.

        Each square gets its OWN arrival-to-departure window, not the whole
        route stamped with the same start time -- that bug once made a
        robot's entire path, start to finish, look occupied RIGHT NOW to
        everyone else's planning, so every other robot tried to avoid all of
        it at once and throughput collapsed to a fraction of what it should
        have been.
        """
        t = now
        for cell in path:
            arrive = t
            t += 1.0 / max(speed, 0.1)
            self.table.put(Reservation(
                robot_id=robot_id, resource=node_key(cell),
                start=arrive, end=t + 0.3, priority=100_000))

    def _dispatch(self, world, robot, task: Task, target: Cell, leg: str,
                  now: float) -> bool:
        path = self._plan_route(world, robot, target, now)
        if path is None:
            return False
        self._book(robot.robot_id, path, now, robot.speed)
        self._current[robot.robot_id] = (
            task.task_id, leg, tuple((c.x, c.y) for c in path))
        self._publish(world, robot.robot_id, now)
        return True

    # -------------------------------------------------------------- tick

    def tick(self, world, now: float) -> None:
        """Assign work, plan routes, and keep telling robots what to do --
        for as long as anyone is listening. Called once per world tick,
        the same as everything else that drives the fleet."""
        self._listen(world, now)

        busy_or_working = set(self._current.keys())
        idle = [r for r in world.robots.values()
               if getattr(r, "centrally_controlled", False)
               and r.status is not RobotStatus.FAILED
               and r.robot_id not in busy_or_working]

        for task in world.board.open_tasks():
            if not idle:
                break
            costed = []
            for r in idle:
                cost = r.cost_of(task, world.grid, now)
                if cost is not None:
                    costed.append((cost, r))
            if not costed:
                continue
            costed.sort(key=lambda cr: (cr[0], cr[1].robot_id))
            cost, robot = costed[0]

            chosen = robot.choose_dropoff(task, world.grid, now)
            task.dropoff = chosen if chosen is not None else task.dropoff
            task.assigned_robot = robot.robot_id
            task.status = TaskStatus.ASSIGNED
            task.claimed_at = now
            task.winning_bid = cost

            if self._dispatch(world, robot, task, task.pickup, "pickup", now):
                idle.remove(robot)
            else:
                # No way through right now -- put it back rather than strand
                # it silently assigned to nobody who can reach it.
                task.assigned_robot = None
                task.status = TaskStatus.ANNOUNCED

        # The delivery leg: only once pickup has been CONFIRMED, never
        # because the robot merely looks idle -- see the module docstring.
        for task in world.board.tasks.values():
            if task.status is not TaskStatus.CARRYING:
                continue
            robot_id = task.assigned_robot
            if robot_id is None:
                continue
            current = self._current.get(robot_id)
            if current is not None and current[0] == task.task_id and current[1] == "dropoff":
                continue                    # already dispatched this leg
            robot = world.robots.get(robot_id)
            if robot is None:
                continue
            self._dispatch(world, robot, task, task.dropoff, "dropoff", now)

        # A stalled robot is deliberately NOT replanned mid-flight. That was
        # tried -- detect no movement for a while, release its bookings, give
        # it a fresh route -- and measured, not assumed: at 5 robots it turned
        # a single stuck robot other traffic could route around into a
        # fleet-wide gridlock that finished 33% FEWER orders (26 vs 39 over
        # 400s), because each "fresh" route was replanned against a table that
        # was itself changing shape as neighbours did the same thing, so
        # robots kept re-committing to routes that immediately re-collided
        # with each other's plans. A boss that commits to a plan and only
        # reconsiders when the robot itself reports new facts (PICKED/DONE)
        # is slower to escape any one stall, but it does not manufacture new
        # ones -- see 26_IMPROVEMENTS_AND_UPGRADES's own point that this rival
        # should not be a strawman built to lose, including to itself.

        # Anyone still waiting on a command they have not confirmed gets it
        # again -- the boss cannot tell the difference between "never
        # arrived" and "arrived, robot is just slow", so it simply repeats
        # itself, the same as _reannounce_forgotten_tasks() already does.
        for robot_id in list(self._current):
            if now - self._last_sent.get(robot_id, -1e9) >= self.RESEND_EVERY:
                self._publish(world, robot_id, now)

    def to_dict(self) -> Dict[str, object]:
        """Plain data for the dashboard."""
        return {
            "commands_sent": self.commands_sent,
            "tracking": len(self._current),
            "table_size": len(self.table),
        }
