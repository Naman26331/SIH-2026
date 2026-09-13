"""One FLEET-X robot, as a ROS 2 node.

What this file does NOT do
--------------------------
It does not decide anything. Not the route, not who gets a square, not who
gives way, not which job to bid for. All of that is shared/fleetx_core, the
same code the laptop simulator runs.

What it DOES do is the wiring the simulator faked:

    /odom   -> robot.update_pose()      where am I really
    /scan   -> robot.sense()            what can I actually see
    brain   -> Nav2 or /cmd_vel         make the wheels turn
    brain  <-> Ros2Bus                  talk to the other robots

The loop this runs:

    read local state -> publish state -> publish intent -> check other intents
    -> predict conflicts -> request reservation -> move safely -> replan

...which is exactly what world.tick() does in the simulator, minus the
pretending-to-drive part.

One waypoint at a time
----------------------
The brain plans the whole route. Nav2 is given only the NEXT SQUARE, and a new
one each time the robot arrives. That keeps Nav2 doing what it is good at --
local obstacle avoidance and smooth motion -- while route choice stays with the
code that knows about bookings and other robots' intentions. Custom logic
stays above/beside Nav2 rather than rewriting the entire navigation stack.
"""

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, LaserScan

from . import translate as T
from .brain import Cell, Robot, RobotStatus, default_grid
from .ros2_bus import Ros2Bus

# How often the brain thinks. The simulator runs at 20 Hz; there is no reason
# for a real robot to think faster than it can act.
BRAIN_HZ = 10.0


class FleetAgent(Node):

    def __init__(self):
        super().__init__("fleet_agent")

        self.declare_parameter("robot_id", "R1")
        self.declare_parameter("start_cell", [2, 8])
        self.declare_parameter("speed", 2.5)            # squares per second
        self.declare_parameter("acceleration", 6.0)     # squares / second^2
        self.declare_parameter("deceleration", 8.0)     # squares / second^2
        self.declare_parameter("drive_mode", "simple")  # "simple" or "nav2"
        self.declare_parameter("sensor_range", 3.0)     # squares
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("linear_gain", 0.8)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_linear", 0.45)      # m/s
        self.declare_parameter("max_angular", 1.2)      # rad/s
        self.declare_parameter("arrive_within", 0.18)   # metres

        p = self.get_parameter
        self.robot_id = p("robot_id").value
        self.drive_mode = p("drive_mode").value
        self.sensor_range = float(p("sensor_range").value)

        # ---- the brain -----------------------------------------------------
        self.grid = default_grid()
        start = p("start_cell").value
        # Never advertise/reserve a faster traversal than the wheel controller
        # can physically achieve. ``speed`` may further lower this ceiling.
        physical_grid_speed = (
            float(p("max_linear").value) / max(T.RESOLUTION, 1e-6))
        self.robot = Robot(robot_id=self.robot_id,
                           cell=Cell(int(start[0]), int(start[1])),
                           speed=min(float(p("speed").value),
                                     physical_grid_speed),
                           acceleration=float(p("acceleration").value),
                           deceleration=float(p("deceleration").value))
        self.bus = Ros2Bus(self, self.robot_id)
        self.bus.register(self.robot_id)

        # ---- sensors in ----------------------------------------------------
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.create_subscription(Odometry, p("odom_topic").value,
                                 self.on_odom, sensor_qos)
        self.create_subscription(LaserScan, p("scan_topic").value,
                                 self.on_scan, sensor_qos)
        self.create_subscription(BatteryState, p("battery_topic").value,
                                 self.on_battery, sensor_qos)

        # ---- wheels out ----------------------------------------------------
        self.cmd_pub = self.create_publisher(Twist, p("cmd_vel_topic").value, 10)
        self.goal_pub = self.create_publisher(PoseStamped, p("goal_topic").value, 10)

        # ---- state ---------------------------------------------------------
        self.have_odom = False
        self.world_x = 0.0
        self.world_y = 0.0
        self.yaw = 0.0
        self.seen_blocked: List[Cell] = []
        self.seen_clear: List[Cell] = []
        self.seen_contacts: List[tuple] = []
        self._last_goal_sent: Optional[Cell] = None
        self._wait_until: float = 0.0
        self._last_tick = self.now()

        self.create_timer(1.0 / BRAIN_HZ, self.think)

        self.get_logger().info(
            f"{self.robot_id} up. brain={__import__('fleet_agent.brain', fromlist=['BRAIN_PATH']).BRAIN_PATH} "
            f"drive_mode={self.drive_mode} start={tuple(start)}")

    # ---------------------------------------------------------------- clock

    def now(self) -> float:
        """Shared ROS clock seconds; remains valid when one node restarts."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- sensors

    def on_odom(self, msg: Odometry) -> None:
        """Where the robot really is. This REPLACES the simulator's pretend
        driving -- the real robot reports back, and the brain believes it."""
        self.world_x = msg.pose.pose.position.x
        self.world_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        gx, gy = T.world_to_grid_xy(self.world_x, self.world_y)
        self.robot.update_pose(gx, gy, T.world_to_cell(self.world_x, self.world_y))
        self.have_odom = True

    def on_scan(self, msg: LaserScan) -> None:
        """Turn a laser scan into "these squares have something in them".

        Anything the map already knows about -- shelves, walls -- is ignored.
        What is left is a box somebody dropped, a pallet, a broken-down robot:
        the things worth broadcasting as a blocked aisle. Squares we can see clearly and are empty get reported too, so a
        block can be cleared early instead of waiting for its timer.
        """
        hits = set()
        contacts = {}
        seen = set()
        angle = msg.angle_min
        for r in msg.ranges:
            a = angle
            angle += msg.angle_increment
            if not math.isfinite(r) or r < msg.range_min:
                continue
            world_angle = self.yaw + a
            reach = min(r, self.sensor_range * T.RESOLUTION)

            # Walk the beam: everything short of the hit is empty.
            step = T.RESOLUTION * 0.5
            d = step
            while d < reach - step:
                cell = T.world_to_cell(self.world_x + math.cos(world_angle) * d,
                                       self.world_y + math.sin(world_angle) * d)
                if self.grid.in_bounds(cell) and self.grid.is_walkable(cell):
                    seen.add(cell)
                d += step

            if r <= self.sensor_range * T.RESOLUTION and r < msg.range_max:
                hit_x = self.world_x + math.cos(world_angle) * r
                hit_y = self.world_y + math.sin(world_angle) * r
                cell = T.world_to_cell(hit_x, hit_y)
                # Only report it if the map says that square should be free.
                if self.grid.in_bounds(cell) and self.grid.is_walkable(cell):
                    if cell != self.robot.cell:
                        hits.add(cell)
                        gx, gy = T.world_to_grid_xy(hit_x, hit_y)
                        contacts[(cell.x, cell.y)] = (gx, gy, cell)

        self.seen_blocked = list(hits)
        self.seen_clear = [c for c in seen if c not in hits]
        # A scan cannot reliably identify which return is a robot. Feeding all
        # unexpected occupied cells to the anonymous local-contact reflex is
        # conservative and keeps collision safety independent of DDS.
        self.seen_contacts = list(contacts.values())

    def on_battery(self, msg: BatteryState) -> None:
        """Use physical battery percentage; ROS reports it from 0.0 to 1.0."""
        percentage = float(msg.percentage)
        if math.isfinite(percentage) and percentage >= 0.0:
            self.robot.battery = max(0.0, min(100.0, percentage * 100.0))

    # -------------------------------------------------------------- the loop

    def think(self) -> None:
        if not self.have_odom:
            return          # do not plan from a position we have not been told

        now = self.now()
        dt = max(1e-3, now - self._last_tick)
        self._last_tick = now

        # 1. what our own sensors found, and tell the fleet about it
        self.robot.sense_robots(self.seen_contacts, now)
        for note in self.robot.sense(self.seen_blocked, self.seen_clear, now):
            self.get_logger().info(note)
        self.seen_blocked, self.seen_clear, self.seen_contacts = [], [], []

        # 2. listen to everyone, and speak
        for note in self.robot.communicate(self.bus, now):
            self.get_logger().info(note)

        # Same operational checks as World.tick(), using only this robot's
        # local state. Network loss slows/stops unsafe work, and physical
        # battery telemetry can trigger charging.
        for operation in (
                self.robot.update_link_health,
                lambda current: self.robot.manage_battery(
                    self.grid, self.bus, current)):
            note = operation(now)
            if note:
                self.get_logger().info(note)

        # 3. jobs: bid, claim, collect, deliver
        for note in self.robot.work_on_tasks(self.grid, self.bus, now):
            self.get_logger().info(note)
        note = self.robot.vacate_station(self.grid, now)
        if note:
            self.get_logger().info(note)

        # 4. think: route, conflicts, bookings, who gives way
        self.robot.decide(self.grid, None, now)
        self.robot.check_conflicts(self.grid, now)
        self.robot.announce_conflicts(self.bus, now)
        self.robot.update_priority(now, dt)
        self.robot.consider_predicted_conflict(self.grid, now, self.bus)
        self.robot.reserve_ahead(self.bus, now)
        self.robot.check_clearance(now, dt)
        if self.robot.consider_reroute(self.grid, now, self.bus):
            self.get_logger().info(self.robot.last_decision)

        # 5. jams that will not clear themselves
        note = self.robot.answer_requests(self.grid, now)
        if note is None:
            note = self.robot.resume_after_yielding(self.grid, now)
        if note is None:
            note = self.robot.report_jam(self.grid, self.bus, now)
        if note is None:
            note = self.robot.back_out_if_wedged(now)
        if note:
            self.get_logger().info(note)

        # 6. NOTE: advance() is deliberately NOT called. That is the
        #    simulator's pretend driving. Here the wheels do it, and odometry
        #    tells us what actually happened.
        self.drive()

    # --------------------------------------------------------------- driving

    def drive(self) -> None:
        """Send the robot at the next square on its route -- or stop it dead."""
        if self.robot.hold or not self.robot.path:
            self._wait_until = 0.0
            self.stop()
            return

        target = self.robot.path[0]
        if target == self.robot.cell:
            now = self.now()
            if self._wait_until <= 0.0:
                self._wait_until = now + 1.0 / max(self.robot.travel_speed(), 0.1)
            self.stop()
            if now >= self._wait_until:
                self.robot.path.pop(0)
                self._wait_until = 0.0
            return
        self._wait_until = 0.0
        if self.drive_mode == "nav2":
            self.send_nav2_goal(target)
        else:
            self.drive_simple(target)

    def stop(self) -> None:
        if self.drive_mode != "nav2":
            self.cmd_pub.publish(Twist())
        self._last_goal_sent = None

    def send_nav2_goal(self, cell: Cell) -> None:
        """Hand Nav2 the next square, and only when it changes.

        Re-sending the same goal every tick makes Nav2 restart its planner, so
        the robot stutters.
        """
        if self._last_goal_sent == cell:
            return
        self._last_goal_sent = cell
        wx, wy = T.cell_to_world(cell)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def drive_simple(self, cell: Cell) -> None:
        """A plain turn-then-go controller, so this runs before Nav2 is set up.

        Nav2 is the right answer for a real robot -- it handles local obstacles,
        smooth acceleration and recovery. But configuring it is the fiddliest
        part of any ROS 2 project, and being able to watch robots actually move
        in Gazebo on day one is worth a lot. Switch with drive_mode:=nav2.
        """
        wx, wy = T.cell_to_world(cell)
        dx, dy = wx - self.world_x, wy - self.world_y
        distance = math.hypot(dx, dy)

        cmd = Twist()
        if distance < float(self.get_parameter("arrive_within").value):
            self.cmd_pub.publish(cmd)
            return

        wanted = math.atan2(dy, dx)
        error = math.atan2(math.sin(wanted - self.yaw), math.cos(wanted - self.yaw))

        max_w = float(self.get_parameter("max_angular").value)
        max_v = float(self.get_parameter("max_linear").value)
        cmd.angular.z = max(-max_w, min(max_w,
                            float(self.get_parameter("angular_gain").value) * error))
        # Turn first, then drive. Driving while badly misaligned just makes arcs.
        if abs(error) < 0.5:
            cmd.linear.x = max(0.0, min(max_v,
                               float(self.get_parameter("linear_gain").value) * distance))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FleetAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
