"""The ROS 2 end of the socket in the wall.

`FleetBus` is the brain's only way of talking to other robots. It has three
methods -- register, publish, poll -- and knows nothing about how a message
actually travels. The laptop simulator plugs in an in-memory bus. This plugs in
ROS 2 topics. The brain cannot tell the difference, and that is the whole point:

        THE BRAIN (written once, tested on a laptop)
      publish(msg)  /  poll(robot_id)
                   |
        -----------+-----------
        |                     |
    InMemoryBus           Ros2Bus
    (simulator)           (here, on DDS)

Topics mirror the brain's message set, one DDS topic per message kind.

A note on quality of service
----------------------------
Positions and heartbeats use BEST_EFFORT with a shallow queue: they are
replaced constantly, and a late one is worse than none. Bookings, claims and
requests use RELIABLE with a deeper queue, because losing one of those changes
who owns a square. The brain already copes with loss either way -- everything
carries a sequence number and a timestamp, and it never assumes a message
arrives -- but there is no reason to throw away the ones that matter.
"""

from collections import deque
from typing import Any, Deque, Dict, List

from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from fleet_msgs.msg import (BatteryStatus, BlockedAisle, ConflictAlert,
                            PathReservation, RobotIntent, RobotState, Task,
                            TaskBid, TaskClaim, WaitReport, YieldRequest)

from . import translate as T
from .brain import (BlockedAisle as BrainBlockedAisle,
                    ConflictAlert as BrainConflictAlert, FleetBus,
                    Heartbeat as BrainHeartbeat,
                    IntentUpdate as BrainIntentUpdate,
                    PathReservation as BrainPathReservation,
                    PoseUpdate as BrainPoseUpdate,
                    TaskAnnounce as BrainTaskAnnounce,
                    TaskBid as BrainTaskBid, TaskClaim as BrainTaskClaim,
                    WaitReport as BrainWaitReport,
                    YieldRequest as BrainYieldRequest)

# Topic names, one per message kind.
TOPICS = {
    "states": "/fleet/robot_states",
    "intents": "/fleet/robot_intents",
    "conflicts": "/fleet/conflicts",
    "reservations": "/fleet/reservations",
    "tasks": "/fleet/tasks",
    "blocked": "/fleet/blocked_aisles",
    "battery": "/fleet/battery",
    "bids": "/fleet/task_bids",
    "claims": "/fleet/task_claims",
    "waits": "/fleet/wait_reports",
    "yields": "/fleet/yield_requests",
}


def _fast(depth: int = 5) -> QoSProfile:
    """For things that are replaced constantly. A stale one is worse than none."""
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def _sure(depth: int = 50) -> QoSProfile:
    """For things that change who owns a square. Worth resending."""
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class Ros2Bus(FleetBus):
    """Carries brain messages over ROS 2 topics.

    One of these per robot node. It subscribes to everything, drops anything
    this robot said itself, and hands the rest to the brain when it asks.
    """

    def __init__(self, node: Node, robot_id: str, inbox_limit: int = 400):
        self.node = node
        self.robot_id = robot_id
        self._inbox: Deque[Any] = deque(maxlen=inbox_limit)
        self.sent = 0
        self.received = 0
        self.dropped_own = 0

        # One row per message type. Spelled out rather than inferred, because
        # publish() is handed a BRAIN object and has to find the right topic
        # from its class -- and a wiring table you can read beats a clever
        # lookup that fails silently when somebody adds a message.
        #
        #  brain class            ros class         topic key       qos        to_ros                 from_ros
        self._wiring = [
            (BrainPoseUpdate,      RobotState,      "states",       _fast(),    T.pose_to_ros,        T.ros_to_pose),
            (BrainIntentUpdate,    RobotIntent,     "intents",      _sure(),    T.intent_to_ros,      T.ros_to_intent),
            (BrainHeartbeat,       BatteryStatus,   "battery",      _fast(),    T.heartbeat_to_ros,   T.ros_to_heartbeat),
            (BrainConflictAlert,   ConflictAlert,   "conflicts",    _sure(),    T.conflict_to_ros,    T.ros_to_conflict),
            (BrainPathReservation, PathReservation, "reservations", _sure(100), T.reservation_to_ros, T.ros_to_reservation),
            (BrainBlockedAisle,    BlockedAisle,    "blocked",      _sure(),    T.blocked_to_ros,     T.ros_to_blocked),
            (BrainTaskAnnounce,    Task,            "tasks",        _sure(),    T.task_to_ros,        T.ros_to_task),
            (BrainTaskBid,         TaskBid,         "bids",         _sure(),    T.bid_to_ros,         T.ros_to_bid),
            (BrainTaskClaim,       TaskClaim,       "claims",       _sure(),    T.claim_to_ros,       T.ros_to_claim),
            (BrainWaitReport,      WaitReport,      "waits",        _sure(),    T.wait_to_ros,        T.ros_to_wait),
            (BrainYieldRequest,    YieldRequest,    "yields",       _sure(),    T.yield_to_ros,       T.ros_to_yield),
        ]

        self._publishers: Dict[str, Any] = {}
        self._to_ros: Dict[type, Any] = {}

        for brain_cls, ros_cls, key, qos, to_ros, from_ros in self._wiring:
            topic = TOPICS[key]
            self._publishers[topic] = node.create_publisher(ros_cls, topic, qos)
            self._to_ros[brain_cls] = (topic, to_ros)
            node.create_subscription(ros_cls, topic,
                                     self._make_callback(from_ros), qos)

    # --------------------------------------------------------- FleetBus API

    def register(self, robot_id: str) -> None:
        """Nothing to do. On DDS, discovery is automatic -- a robot that joins
        the network is simply heard. This exists so the brain can call the same
        method whichever bus it is plugged into."""
        return None

    def publish(self, message: Any) -> None:
        entry = self._to_ros.get(type(message))
        if entry is None:
            self.node.get_logger().warn(
                f"No ROS 2 topic for {type(message).__name__} -- not sent.")
            return
        topic, to_ros = entry
        self._publishers[topic].publish(to_ros(message))
        self.sent += 1

    def poll(self, robot_id: str) -> List[Any]:
        """Hand over everything heard since the last ask, and clear the inbox."""
        out = list(self._inbox)
        self._inbox.clear()
        return out

    # ------------------------------------------------------------ internals

    def _make_callback(self, from_ros):
        def _cb(ros_msg):
            if ros_msg.robot_id == self.robot_id:
                self.dropped_own += 1
                return          # a robot does not talk to itself
            self._inbox.append(from_ros(ros_msg))
            self.received += 1
        return _cb

    def stats(self) -> Dict[str, int]:
        return {"sent": self.sent, "received": self.received,
                "own_dropped": self.dropped_own, "inbox": len(self._inbox)}
