"""Start a whole fleet, plus the order source.

    ros2 launch fleet_agent fleet.launch.py robots:=3
    ros2 launch fleet_agent fleet.launch.py robots:=5 drive_mode:=nav2

Each robot gets its own namespace (/R1, /R2, ...) so its odom, scan and cmd_vel
stay separate -- but the /fleet/* topics are GLOBAL, which is what lets them
hear each other. That is the whole point, so the leading slash on those topic
names in ros2_bus.py matters: without it every robot would talk only to itself.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same starting squares as the simulator's fleet_world(), so a run here lines
# up with a run on the laptop.
START_CELLS = [
    (2, 8), (26, 8), (13, 0), (2, 0), (26, 0),
    (2, 4), (26, 4), (13, 4), (2, 12), (26, 12),
    (19, 0), (7, 0), (19, 4), (7, 4), (19, 12),
    (7, 12), (13, 12), (20, 8), (7, 8), (19, 8),
]


def spawn(context, *args, **kwargs):
    count = int(LaunchConfiguration("robots").perform(context))
    drive_mode = LaunchConfiguration("drive_mode").perform(context)
    orders_every = float(LaunchConfiguration("orders_every").perform(context))

    if count > len(START_CELLS):
        raise RuntimeError(
            f"No starting square for more than {len(START_CELLS)} robots.")

    nodes = []
    for i in range(count):
        rid = f"R{i + 1}"
        cx, cy = START_CELLS[i]
        nodes.append(Node(
            package="fleet_agent", executable="agent",
            name="fleet_agent", namespace=rid, output="screen",
            parameters=[{
                "robot_id": rid,
                "drive_mode": drive_mode,
                "start_cell": [cx, cy],
            }],
        ))

    if orders_every > 0:
        nodes.append(Node(
            package="fleet_agent", executable="order_source",
            name="order_source", output="screen",
            parameters=[{"every": orders_every}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="3"),
        DeclareLaunchArgument("drive_mode", default_value="simple"),
        DeclareLaunchArgument("orders_every", default_value="6.0",
                              description="seconds between orders, 0 to disable"),
        OpaqueFunction(function=spawn),
    ])
