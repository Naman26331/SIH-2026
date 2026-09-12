"""Everything: Gazebo, the warehouse, N robots, their brains, and orders.

    ros2 launch fleet_bringup warehouse.launch.py robots:=3
    ros2 launch fleet_bringup warehouse.launch.py robots:=5 gui:=false

This is the one command your teammate wants. It:
  1. starts Gazebo with the generated warehouse
  2. spawns N differential-drive robots at the same squares the simulator uses
  3. starts one fleet_agent brain per robot
  4. starts the order source

Robots live in their own namespaces (/R1, /R2, ...) so their wheels and lasers
stay separate. The /fleet/* topics are global -- that is what lets them hear
each other, and it is the whole point.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same squares as fleet_world() in the simulator, so a Gazebo run and a laptop
# run start from an identical position.
START_CELLS = [
    (2, 8), (26, 8), (13, 0), (2, 0), (26, 0),
    (2, 4), (26, 4), (13, 4), (2, 12), (26, 12),
    (19, 0), (7, 0), (19, 4), (7, 4), (19, 12),
    (7, 12), (13, 12), (20, 8), (7, 8), (19, 8),
]
RESOLUTION = 1.0        # must match translate.py


def cell_to_world(gx, gy):
    return gx * RESOLUTION, -gy * RESOLUTION


def spawn_everything(context, *args, **kwargs):
    pkg = get_package_share_directory("fleet_bringup")
    count = int(LaunchConfiguration("robots").perform(context))
    drive_mode = LaunchConfiguration("drive_mode").perform(context)
    orders_every = float(LaunchConfiguration("orders_every").perform(context))
    xacro_file = os.path.join(pkg, "models", "amr.urdf.xacro")

    if count > len(START_CELLS):
        raise RuntimeError(f"Only {len(START_CELLS)} starting squares are defined.")

    actions = []
    for i in range(count):
        rid = f"R{i + 1}"
        gx, gy = START_CELLS[i]
        wx, wy = cell_to_world(gx, gy)

        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=rid, name="robot_state_publisher", output="screen",
            parameters=[{
                "robot_description": _xacro(xacro_file, rid),
                "frame_prefix": f"{rid}/",
            }],
        ))

        actions.append(TimerAction(period=2.0 + i * 1.5, actions=[Node(
            package="gazebo_ros", executable="spawn_entity.py",
            name=f"spawn_{rid}", output="screen",
            arguments=["-entity", rid, "-topic", f"/{rid}/robot_description",
                       "-x", str(wx), "-y", str(wy), "-z", "0.1"],
        )]))

        actions.append(TimerAction(period=6.0 + i * 1.5, actions=[Node(
            package="fleet_agent", executable="agent",
            name="fleet_agent", namespace=rid, output="screen",
            parameters=[{
                "robot_id": rid,
                "drive_mode": drive_mode,
                "start_cell": [gx, gy],
            }],
        )]))

    if orders_every > 0:
        actions.append(TimerAction(period=8.0 + count * 1.5, actions=[Node(
            package="fleet_agent", executable="order_source",
            name="order_source", output="screen",
            parameters=[{"every": orders_every}],
        )]))

    return actions


def _xacro(path, robot_id):
    """Run xacro now and return the URDF text.

    Done here rather than with a Command substitution so that a broken xacro
    fails loudly at launch time with a readable error, instead of quietly
    handing robot_state_publisher an empty string.
    """
    import subprocess
    try:
        return subprocess.check_output(
            ["xacro", path, f"robot_id:={robot_id}"]).decode()
    except FileNotFoundError:
        raise RuntimeError(
            "xacro is not installed. Try: sudo apt install ros-$ROS_DISTRO-xacro")
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f"xacro failed on {path}: {exc}")


def generate_launch_description():
    pkg = get_package_share_directory("fleet_bringup")
    world = os.path.join(pkg, "worlds", "warehouse.world")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="3"),
        DeclareLaunchArgument("drive_mode", default_value="simple",
                              description="simple = built-in controller, "
                                          "nav2 = publish goals for Nav2"),
        DeclareLaunchArgument("orders_every", default_value="6.0"),
        DeclareLaunchArgument("gui", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world, "verbose": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, "launch", "gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),

        OpaqueFunction(function=spawn_everything),
    ])
