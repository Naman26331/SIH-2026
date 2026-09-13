"""ROS-free projection of fleet messages into dashboard state.

This module is deliberately independent of rclpy.  ROS callbacks translate a
message and hand it here; HTTP code only asks for snapshots.  That boundary
keeps the bridge testable on any development machine.
"""

from collections import deque
import threading
import time
from typing import Any, Dict, Optional

from .brain import add_brain_to_path

add_brain_to_path()

from fleetx_core import (  # noqa: E402
    BlockedAisle, BlockedMap, Cell, ConflictAlert, Heartbeat, InMemoryBus,
    IntentUpdate, PathReservation, PoseUpdate, Reservation, ReservationTable,
    Robot, RobotStatus, Task, TaskAnnounce, TaskBid, TaskClaim, TaskStatus,
    WaitForGraph, WaitReport, Waiting, World, YieldRequest, edge_key, node_key,
    target_key,
)
from fleetx_core import security  # noqa: E402


class GatewayBus(InMemoryBus):
    """Metrics/event buffer only; never used to coordinate robots."""

    def __init__(self) -> None:
        super().__init__()
        self.received = 0
        self.rejected = 0

    def observe(self, message: Any) -> None:
        self.received += 1
        self.delivered += 1
        self.recent.append(message)

    def stats(self) -> Dict[str, Any]:
        return {
            "transport": "ROS 2 DDS",
            "received": self.received,
            "rejected": self.rejected,
            "sent": self.sent,
            "delivered": self.delivered,
            "dropped": self.dropped,
            "packet_loss": 0.0,
            "latency_ms": 0,
            "silenced": [],
        }


class TelemetryProjector:
    """Thread-safe, read-only dashboard model built from signed bus traffic."""

    def __init__(self, world: Optional[World] = None) -> None:
        self.bus = GatewayBus()
        self.world = world if world is not None else World(bus=self.bus)
        if world is not None:
            self.world.bus = self.bus
        self.lock = threading.RLock()
        self._started = time.monotonic()
        self._guard = security.ReplayGuard()
        self._table = ReservationTable()
        self._blocked = BlockedMap()
        self._waits = WaitForGraph()
        self._velocities: Dict[str, float] = {}
        self._conflicts: Dict[tuple, Dict[str, Any]] = {}
        self._rejections = deque(maxlen=20)

    def now(self) -> float:
        return time.monotonic() - self._started

    @staticmethod
    def timestamp() -> float:
        """Wall-clock domain used by all ROS nodes and signature freshness."""
        return time.time()

    def _ensure_robot(self, robot_id: str, cell: Optional[Cell] = None) -> Robot:
        robot = self.world.get(robot_id)
        if robot is not None:
            return robot
        start = cell if cell is not None and self.world.grid.is_walkable(cell) else Cell(0, 0)
        if not self.world.grid.is_walkable(start):
            start = next(
                Cell(x, y)
                for y in range(self.world.grid.height)
                for x in range(self.world.grid.width)
                if self.world.grid.is_walkable(Cell(x, y))
            )
        robot = self.world.add_robot(Robot(robot_id=robot_id, cell=start))
        # Gateway sees one DDS stream. Shared observer views are intentional;
        # robot control still uses each agent's private copies on-device.
        robot.table = self._table
        robot.blocked_map = self._blocked
        robot.waits = self._waits
        return robot

    @staticmethod
    def _status(value: str) -> RobotStatus:
        try:
            return RobotStatus(value)
        except ValueError:
            return RobotStatus.IDLE

    def ingest(self, message: Any) -> bool:
        """Verify then project one brain message. False means rejected."""
        with self.lock:
            sender = getattr(message, "sender", "") or getattr(message, "robot_id", "")
            verdict = security.check_signature(message)
            if verdict:
                verdict = self._guard.check(
                    sender, int(getattr(message, "seq", 0)),
                    float(getattr(message, "timestamp", 0.0)), self.timestamp(),
                    stream=str(getattr(message, "type", type(message).__name__)))
            if not verdict:
                self.bus.rejected += 1
                self._rejections.append({"sender": sender, "reason": verdict.reason})
                return False

            self.bus.observe(message)
            now = self.now()
            claimed = getattr(message, "robot_id", "")
            if claimed and claimed not in {"ORDERS", "OPERATOR", "UI", "BOSS"}:
                self._ensure_robot(claimed)

            for robot in self.world.robots.values():
                robot.fleet.ingest(message, now)
                note = robot.fleet.get(claimed)
                # FleetView normally receives same clock domain as sender.
                # Dashboard uses elapsed time, so retain arrival age in that
                # domain without changing signed message contents.
                if note is not None:
                    note.last_heard = now
                    if isinstance(message, PoseUpdate):
                        note.last_pose = now
                    elif isinstance(message, IntentUpdate):
                        note.last_intent = now
                        note.intent_sent_at = now
                robot.messages_heard += 1

            if isinstance(message, PoseUpdate):
                cell = Cell(*message.cell)
                robot = self._ensure_robot(message.robot_id, cell)
                robot.update_pose(message.x, message.y, cell)
                robot.heading = message.heading
                self._velocities[message.robot_id] = message.velocity

            elif isinstance(message, IntentUpdate):
                robot = self._ensure_robot(message.robot_id)
                robot.x, robot.y = message.x, message.y
                robot.goal = Cell(*message.destination) if message.destination else None
                robot.path = [Cell(*node) for node in message.planned_nodes]
                robot.priority = message.priority
                robot.status = self._status(message.status)
                robot.hold = message.velocity <= 0.0 and bool(robot.path)
                self._velocities[message.robot_id] = message.velocity

            elif isinstance(message, Heartbeat):
                robot = self._ensure_robot(message.robot_id)
                robot.battery = message.battery
                robot.status = self._status(message.status)

            elif isinstance(message, PathReservation):
                self._reservation(message)

            elif isinstance(message, BlockedAisle):
                cell = Cell(*message.cell)
                if message.cleared:
                    self._blocked.clear(cell)
                else:
                    self._blocked.mark(cell, message.robot_id, now,
                                       ttl=0.0, confidence=message.confidence)

            elif isinstance(message, WaitReport):
                robot = self._ensure_robot(message.robot_id)
                robot.blocked_by = message.blocked_by
                if message.blocked_by:
                    self._waits.add(Waiting(message.robot_id, message.blocked_by,
                                            message.waiting, message.priority,
                                            message.is_moving))
                else:
                    self._waits.clear(message.robot_id)

            elif isinstance(message, YieldRequest):
                robot = self._ensure_robot(message.robot_id)
                robot.asked += 1
                self.world.decisions.append({
                    "sim_time": round(now, 2), "robot_id": message.robot_id,
                    "text": f"{message.robot_id} asked {message.target} to yield: {message.reason}",
                })
                self.world.decisions[:] = self.world.decisions[-20:]

            elif isinstance(message, ConflictAlert):
                key = (tuple(sorted((message.robot_a, message.robot_b))), message.resource)
                self._conflicts[key] = {
                    "robot_a": message.robot_a, "robot_b": message.robot_b,
                    "cell": list(message.resource), "kind": message.kind,
                    "lead_time": round(message.estimated_time, 2),
                    "seen_by": [message.robot_id], "seen_at": now,
                }
                self.world.conflicts_raised += 1

            elif isinstance(message, (TaskAnnounce, TaskBid, TaskClaim)):
                self._task(message, now)

            return True

    def _reservation(self, message: PathReservation) -> None:
        cells = [Cell(*pair) for pair in message.cells]
        if message.kind == "NODE" and cells:
            resource = node_key(cells[0])
        elif message.kind == "TARGET" and cells:
            resource = target_key(cells[0])
        elif message.kind == "EDGE" and len(cells) >= 2:
            resource = edge_key(cells[0], cells[1])
        else:
            return
        if message.action == "RELEASE":
            self._table.release(message.robot_id, resource)
        else:
            now = self.now()
            start = now + (message.start - message.timestamp)
            end = now + (message.end - message.timestamp)
            self._table.put(Reservation(message.robot_id, resource,
                                        start, end, message.priority))

    def _task(self, message: Any, now: float) -> None:
        board = self.world.board
        if isinstance(message, TaskAnnounce):
            task = board.get(message.task_id)
            if task is None:
                board.add(Task(
                    task_id=message.task_id, pickup=Cell(*message.pickup),
                    dropoff=Cell(*message.dropoff), product=message.product,
                    shelf=message.shelf, quantity=message.quantity,
                    priority=message.priority, flexible=message.flexible,
                    created_at=now, announced_at=now,
                    status=TaskStatus.ANNOUNCED,
                ))
        elif isinstance(message, TaskBid):
            task = board.get(message.task_id)
            if task is not None and task.open_for_bids:
                task.bids[message.robot_id] = message.cost
        elif isinstance(message, TaskClaim):
            task = board.get(message.task_id)
            if task is None:
                return
            robot = self._ensure_robot(message.robot_id)
            if message.action == "CLAIM":
                task.assigned_robot = message.robot_id
                task.status = TaskStatus.ASSIGNED
                task.claimed_at = now
                task.winning_bid = message.cost
                robot.task = task
            elif message.action == "PICKED":
                task.status = TaskStatus.CARRYING
                task.picked_at = now
                robot.task = task
            elif message.action == "DONE":
                task.status = TaskStatus.DONE
                task.done_at = now
                task.assigned_robot = message.robot_id
                robot.tasks_done += 1
                robot.task = None
                self.world.tasks_completed += 1
            elif message.action == "RELEASE" and task.assigned_robot == message.robot_id:
                task.release()
                robot.task = None

    def snapshot(self, focus: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            now = self.now()
            self.world.sim_time = now
            self._table.prune(now)
            self._blocked.expire(now)
            self._conflicts = {
                key: row for key, row in self._conflicts.items()
                if now - row["seen_at"] <= 2.0
            }
            snap = self.world.snapshot(focus=focus)
            snap["mode"] = "ros2"
            snap["conflicts"] = [
                {key: value for key, value in row.items() if key != "seen_at"}
                for row in self._conflicts.values()
            ]
            snap["obstacles"] = [[c.x, c.y] for c in self._blocked.cells(now)]
            snap["wait_graph"] = self._waits.to_rows()
            snap["stuck"] = sorted(
                row["robot_id"] for row in snap["wait_graph"]
                if row["waiting"] >= 4.0
            )
            for row in snap["robots"]:
                row["velocity"] = round(self._velocities.get(row["robot_id"], 0.0), 2)
            snap["kpis"]["messages_rejected"] = self.bus.rejected
            snap["security_rejections"] = list(self._rejections)
            return snap
