"""Where orders come from.

This is NOT part of the fleet. It stands in for the warehouse's order system --
the website, the ERP, whatever is actually generating work. It announces jobs on
/fleet/tasks and then has nothing more to do with them: which robot takes each
job is settled between the robots themselves, by auction.

That split matters for the pitch. Orders arriving centrally is normal and fine.
Deciding who does them is what must survive the network going down.
"""

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from fleet_msgs.msg import Task

from . import translate as T
from .brain import Cell, TaskAnnounce, default_grid
from .brain import Grid  # noqa: F401  (kept for type clarity)
from fleetx_core import security

PRODUCTS = ["Wireless Mouse", "Keyboard", "USB Hub", "Webcam", "Headphones",
            "Monitor Stand", "Laptop Sleeve", "Power Bank", "HDMI Cable"]


class OrderSource(Node):

    def __init__(self):
        super().__init__("order_source")
        self.declare_parameter("every", 6.0)      # seconds between orders
        self.declare_parameter("limit", 0)        # 0 = keep going for ever
        self.declare_parameter("seed", 1)

        self.every = float(self.get_parameter("every").value)
        self.limit = int(self.get_parameter("limit").value)
        self.rng = random.Random(int(self.get_parameter("seed").value))

        self.grid = default_grid()
        from fleetx_core.grid import CellKind
        self.pickups = [
            c for c in self.grid.cells_of_kind(CellKind.FLOOR)
            if any(not self.grid.is_walkable(n) for n in (
                Cell(c.x + 1, c.y), Cell(c.x - 1, c.y),
                Cell(c.x, c.y + 1), Cell(c.x, c.y - 1)))
        ]
        self.dropoffs = self.grid.cells_of_kind(CellKind.DROP) or self.pickups

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=50)
        self.pub = self.create_publisher(Task, "/fleet/tasks", qos)

        self.count = 0
        self.seq = 0
        self.create_timer(self.every, self.emit)
        self.get_logger().info(
            f"Order source up: one order every {self.every:g}s"
            + (f", {self.limit} in total" if self.limit else ", indefinitely"))

    def emit(self):
        if self.limit and self.count >= self.limit:
            return
        self.count += 1
        self.seq += 1
        pickup = self.rng.choice(self.pickups)
        dropoff = self.rng.choice(self.dropoffs)
        product = self.rng.choice(PRODUCTS)
        now = self.get_clock().now().nanoseconds * 1e-9

        announce = TaskAnnounce(
            robot_id="ORDERS", timestamp=now, seq=self.seq,
            task_id=f"T-{self.count:03d}",
            pickup=(pickup.x, pickup.y), dropoff=(dropoff.x, dropoff.y),
            product=product, priority=self.rng.choice([5, 5, 5, 6, 7]),
            flexible=True)
        security.seal(announce)
        self.pub.publish(T.task_to_ros(announce))
        self.get_logger().info(
            f"ORDER {announce.task_id}: {product} - collect "
            f"({pickup.x},{pickup.y}) deliver to a packing station")


def main(args=None):
    rclpy.init(args=args)
    node = OrderSource()
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
