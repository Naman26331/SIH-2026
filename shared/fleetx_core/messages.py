"""What robots say to each other.

PURE LOGIC ONLY. No web code, no ROS 2 code.

Three messages, taken straight from 04_DECENTRALIZED_FLEET_PROTOCOL section 2
and 03_ROBOT_AND_ROS2 sections 2-3. The field names deliberately match those
documents so the ROS 2 version is a direct translation:

    Heartbeat      ->  fleet_msgs/BatteryStatus + health   -> /fleet/robot_states
    PoseUpdate     ->  fleet_msgs/RobotState               -> /fleet/robot_states
    IntentUpdate   ->  fleet_msgs/RobotIntent              -> /fleet/robot_intents

Every message carries:
  timestamp -- when it was said, so a listener can tell old news from new
  seq       -- a counter, so a listener can tell a message went missing

04_DECENTRALIZED_FLEET_PROTOCOL section 8 insists on both: the protocol must
never assume every message arrives.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


class MessageType(str, Enum):
    HEARTBEAT = "HEARTBEAT"
    POSE_UPDATE = "POSE_UPDATE"
    INTENT_UPDATE = "INTENT_UPDATE"
    CONFLICT_ALERT = "CONFLICT_ALERT"
    PATH_RESERVATION = "PATH_RESERVATION"
    WAIT_REPORT = "WAIT_REPORT"
    YIELD_REQUEST = "YIELD_REQUEST"
    BLOCKED_AISLE = "BLOCKED_AISLE"
    TASK_ANNOUNCE = "TASK_ANNOUNCE"
    TASK_BID = "TASK_BID"
    TASK_CLAIM = "TASK_CLAIM"


@dataclass
class Heartbeat:
    """"I am alive." Sent constantly, whether anything is happening or not.

    If these stop arriving, the other robots conclude this one is gone.
    """

    robot_id: str
    timestamp: float
    seq: int
    battery: float
    status: str
    health: str = "OK"

    type: str = field(default=MessageType.HEARTBEAT.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "battery": round(self.battery, 1), "status": self.status,
            "health": self.health,
        }


@dataclass
class PoseUpdate:
    """"This is where I am right now." Sent often, because it changes fast."""

    robot_id: str
    timestamp: float
    seq: int
    x: float
    y: float
    cell: Tuple[int, int]
    velocity: float
    heading: str = "E"

    type: str = field(default=MessageType.POSE_UPDATE.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "x": round(self.x, 3), "y": round(self.y, 3),
            "cell": [self.cell[0], self.cell[1]],
            "velocity": round(self.velocity, 2), "heading": self.heading,
        }


@dataclass
class IntentUpdate:
    """"This is where I am ABOUT to be." The important one.

    A pose says where a robot is. An intent says which squares it is going to
    occupy and WHEN it expects to be on each one. That is the difference
    between reacting to a crash and seeing it coming.

    04_DECENTRALIZED_FLEET_PROTOCOL section 3:
        Position-only:  "R1 is here."
        Intent-aware:   "R1 is here. R1 wants to go there. R1 expects to occupy
                         these nodes. R1 expects to arrive at this time."
    """

    robot_id: str
    timestamp: float
    seq: int
    x: float
    y: float
    velocity: float
    destination: Optional[Tuple[int, int]]
    planned_nodes: List[Tuple[int, int]]
    node_etas: List[float]          # seconds from now, one per planned node
    eta_destination: Optional[float]
    priority: int
    status: str

    type: str = field(default=MessageType.INTENT_UPDATE.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "x": round(self.x, 3), "y": round(self.y, 3),
            "velocity": round(self.velocity, 2),
            "destination": list(self.destination) if self.destination else None,
            "planned_nodes": [[n[0], n[1]] for n in self.planned_nodes],
            "node_etas": [round(e, 2) for e in self.node_etas],
            "eta_destination": (round(self.eta_destination, 2)
                                if self.eta_destination is not None else None),
            "priority": self.priority, "status": self.status,
        }

    def eta_for(self, cell: Tuple[int, int]) -> Optional[float]:
        """When does this robot think it will be standing on that square?

        Returns seconds from when the message was sent, or None if the square
        is not on its plan. This is what Phase 4 uses to spot a conflict before
        it happens.
        """
        target = (cell[0], cell[1])
        for node, eta in zip(self.planned_nodes, self.node_etas):
            if (node[0], node[1]) == target:
                return eta
        return None


@dataclass
class ConflictAlert:
    """"You and I are going to want the same square at the same time."

    Spelled out in 04_DECENTRALIZED_FLEET_PROTOCOL section 2. Phase 4 only
    announces it. Phase 6 is where the two robots argue about who yields.
    """

    robot_id: str          # the robot raising the alarm
    timestamp: float
    seq: int
    robot_a: str
    robot_b: str
    resource: Tuple[int, int]        # the contested square
    kind: str                        # SAME_SQUARE / HEAD_ON / JUNCTION
    estimated_time: float            # seconds from now until the clash

    type: str = field(default=MessageType.CONFLICT_ALERT.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "robot_a": self.robot_a, "robot_b": self.robot_b,
            "resource": [self.resource[0], self.resource[1]],
            "kind": self.kind,
            "estimated_time": round(self.estimated_time, 2),
        }


@dataclass
class PathReservation:
    """"I am booking this square for this stretch of time."

    04_DECENTRALIZED_FLEET_PROTOCOL section 2. Everyone who hears it writes it
    into their own copy of the reservation table. Nobody grants it -- ownership
    is worked out separately, by a rule every robot applies identically.

    cells holds one square for a NODE booking, or the two ends for an EDGE.
    """

    robot_id: str
    timestamp: float
    seq: int
    action: str                      # CLAIM or RELEASE
    kind: str                        # NODE or EDGE
    cells: List[Tuple[int, int]]
    start: float                     # absolute time, not "seconds from now"
    end: float
    priority: int = 5

    type: str = field(default=MessageType.PATH_RESERVATION.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "action": self.action, "kind": self.kind,
            "cells": [[c[0], c[1]] for c in self.cells],
            "start": round(self.start, 3), "end": round(self.end, 3),
            "priority": self.priority,
        }


@dataclass
class WaitReport:
    """"I am stuck, and this is who I am stuck behind."

    Everyone collects these and builds the little map of who is waiting for
    whom. 05_PATH_PLANNING section 8 calls it a wait-for graph; a loop in it
    is a deadlock. An extension to the protocol in 04, which stops at
    conflict alerts.
    """

    robot_id: str
    timestamp: float
    seq: int
    blocked_by: Optional[str]        # None = not stuck
    waiting: float                   # seconds stuck
    priority: int
    is_moving: bool                  # False = parked, will never clear alone

    type: str = field(default=MessageType.WAIT_REPORT.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "blocked_by": self.blocked_by, "waiting": round(self.waiting, 2),
            "priority": self.priority, "is_moving": self.is_moving,
        }


@dataclass
class YieldRequest:
    """"Excuse me, you are in my way -- please move."

    Sent to a robot that is parked on a square somebody needs, or to the one
    chosen to give way and break a circular wait.
    """

    robot_id: str                    # who is asking
    timestamp: float
    seq: int
    target: str                      # who is being asked to move
    resource: Tuple[int, int]        # the square wanted
    reason: str                      # PARKED or CYCLE

    type: str = field(default=MessageType.YIELD_REQUEST.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "target": self.target,
            "resource": [self.resource[0], self.resource[1]],
            "reason": self.reason,
        }


@dataclass
class BlockedAisle:
    """"There is something in the way here."

    04_DECENTRALIZED_FLEET_PROTOCOL section 2 spells this one out, ttl and all:

        {"type": "BLOCKED_AISLE", "aisle": "A07", "confidence": 0.96, "ttl": 15}

    We name the exact square rather than an aisle, because our warehouse is a
    grid. ttl is how long the block should be believed without anybody seeing
    it again -- boxes get picked up, and a block that never faded would leave a
    phantom wall in everyone's map for ever.
    """

    robot_id: str
    timestamp: float
    seq: int
    cell: Tuple[int, int]
    confidence: float = 1.0
    ttl: float = 20.0
    cleared: bool = False        # True = "I looked, and it is gone"

    type: str = field(default=MessageType.BLOCKED_AISLE.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "cell": [self.cell[0], self.cell[1]],
            "confidence": round(self.confidence, 2),
            "ttl": round(self.ttl, 1), "cleared": self.cleared,
        }


@dataclass
class TaskAnnounce:
    """"There is a job going: collect from here, deliver to there."

    The order comes from outside the fleet -- somebody bought something. Who
    DOES it is settled by the robots between themselves.
    03_ROBOT_AND_ROS2 section 2 calls this Task.msg on /fleet/tasks.
    """

    robot_id: str                    # whoever put it on the air
    timestamp: float
    seq: int
    task_id: str
    pickup: Tuple[int, int]
    dropoff: Tuple[int, int]
    product: str = ""
    priority: int = 5
    flexible: bool = False       # any packing station will do

    type: str = field(default=MessageType.TASK_ANNOUNCE.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "task_id": self.task_id,
            "pickup": [self.pickup[0], self.pickup[1]],
            "dropoff": [self.dropoff[0], self.dropoff[1]],
            "product": self.product, "priority": self.priority,
            "flexible": self.flexible,
        }


@dataclass
class TaskBid:
    """"That job would cost me this much." Lower is a better bid."""

    robot_id: str
    timestamp: float
    seq: int
    task_id: str
    cost: float

    type: str = field(default=MessageType.TASK_BID.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "task_id": self.task_id, "cost": round(self.cost, 2),
        }


@dataclass
class TaskClaim:
    """"I am taking that job" -- or picked it up, delivered it, or gave it up.

    action is CLAIM, PICKED, DONE or RELEASE.
    """

    robot_id: str
    timestamp: float
    seq: int
    task_id: str
    action: str
    cost: float = 0.0

    type: str = field(default=MessageType.TASK_CLAIM.value, init=False)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": self.type, "robot_id": self.robot_id,
            "timestamp": round(self.timestamp, 3), "seq": self.seq,
            "task_id": self.task_id, "action": self.action,
            "cost": round(self.cost, 2),
        }


def from_dict(data: Dict[str, Any]):
    """Rebuild a message from plain data.

    Needed when messages arrive over something that only carries text or JSON,
    which is what the ROS 2 bridge and any log replay will do.
    """
    kind = data.get("type")
    if kind == MessageType.HEARTBEAT.value:
        return Heartbeat(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            battery=data["battery"], status=data["status"], health=data.get("health", "OK"),
        )
    if kind == MessageType.POSE_UPDATE.value:
        return PoseUpdate(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            x=data["x"], y=data["y"], cell=tuple(data["cell"]),
            velocity=data["velocity"], heading=data.get("heading", "E"),
        )
    if kind == MessageType.INTENT_UPDATE.value:
        dest = data.get("destination")
        return IntentUpdate(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            x=data["x"], y=data["y"], velocity=data["velocity"],
            destination=tuple(dest) if dest else None,
            planned_nodes=[tuple(n) for n in data.get("planned_nodes", [])],
            node_etas=list(data.get("node_etas", [])),
            eta_destination=data.get("eta_destination"),
            priority=data.get("priority", 5), status=data.get("status", "IDLE"),
        )
    if kind == MessageType.CONFLICT_ALERT.value:
        return ConflictAlert(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            robot_a=data["robot_a"], robot_b=data["robot_b"],
            resource=tuple(data["resource"]), kind=data["kind"],
            estimated_time=data["estimated_time"],
        )
    if kind == MessageType.PATH_RESERVATION.value:
        return PathReservation(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            action=data["action"], kind=data["kind"],
            cells=[tuple(c) for c in data["cells"]],
            start=data["start"], end=data["end"], priority=data.get("priority", 5),
        )
    if kind == MessageType.WAIT_REPORT.value:
        return WaitReport(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            blocked_by=data.get("blocked_by"), waiting=data.get("waiting", 0.0),
            priority=data.get("priority", 5), is_moving=data.get("is_moving", True),
        )
    if kind == MessageType.YIELD_REQUEST.value:
        return YieldRequest(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            target=data["target"], resource=tuple(data["resource"]),
            reason=data.get("reason", "PARKED"),
        )
    if kind == MessageType.BLOCKED_AISLE.value:
        return BlockedAisle(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            cell=tuple(data["cell"]), confidence=data.get("confidence", 1.0),
            ttl=data.get("ttl", 20.0), cleared=data.get("cleared", False),
        )
    if kind == MessageType.TASK_ANNOUNCE.value:
        return TaskAnnounce(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            task_id=data["task_id"], pickup=tuple(data["pickup"]),
            dropoff=tuple(data["dropoff"]), product=data.get("product", ""),
            priority=data.get("priority", 5), flexible=data.get("flexible", False),
        )
    if kind == MessageType.TASK_BID.value:
        return TaskBid(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            task_id=data["task_id"], cost=data["cost"],
        )
    if kind == MessageType.TASK_CLAIM.value:
        return TaskClaim(
            robot_id=data["robot_id"], timestamp=data["timestamp"], seq=data["seq"],
            task_id=data["task_id"], action=data["action"], cost=data.get("cost", 0.0),
        )
    raise ValueError(f"Unknown message type: {kind!r}")
