"""Thin ROS 2 adapter: DDS messages in, telemetry projector out."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from fleet_msgs.msg import (BatteryStatus, BlockedAisle, ConflictAlert,
                            PathReservation, RobotIntent,
                            RobotState, Task, TaskBid, TaskClaim, WaitReport,
                            YieldRequest)

from . import translate as T
from .telemetry import TelemetryProjector


def _fast(depth=5):
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def _sure(depth=50):
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


class Ros2GatewayNode(Node):
    def __init__(self, projector=None):
        super().__init__("fleet_gateway")
        self.projector = projector or TelemetryProjector()
        self.world = self.projector.world
        self.lock = self.projector.lock

        wiring = [
            (RobotState, "/fleet/robot_states", _fast(), T.ros_to_pose),
            (RobotIntent, "/fleet/robot_intents", _sure(), T.ros_to_intent),
            (BatteryStatus, "/fleet/battery", _fast(), T.ros_to_heartbeat),
            (ConflictAlert, "/fleet/conflicts", _sure(), T.ros_to_conflict),
            (PathReservation, "/fleet/reservations", _sure(100), T.ros_to_reservation),
            (BlockedAisle, "/fleet/blocked_aisles", _sure(), T.ros_to_blocked),
            (Task, "/fleet/tasks", _sure(), T.ros_to_task),
            (TaskBid, "/fleet/task_bids", _sure(), T.ros_to_bid),
            (TaskClaim, "/fleet/task_claims", _sure(), T.ros_to_claim),
            (WaitReport, "/fleet/wait_reports", _sure(), T.ros_to_wait),
            (YieldRequest, "/fleet/yield_requests", _sure(), T.ros_to_yield),
        ]
        self._subscriptions = [
            self.create_subscription(cls, topic, self._callback(convert), qos)
            for cls, topic, qos, convert in wiring
        ]
        self.get_logger().info("Fleet gateway listening on /fleet/*")

    def _callback(self, convert):
        def receive(ros_message):
            message = convert(ros_message)
            if not self.projector.ingest(message):
                self.get_logger().warning(
                    f"Rejected {type(message).__name__} from {message.robot_id}")
        return receive

    def snapshot(self, focus=None):
        return self.projector.snapshot(focus)

def main(args=None):
    rclpy.init(args=args)
    node = Ros2GatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
