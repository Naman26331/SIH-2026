"""Start a single robot. The smallest thing that proves the wiring works.

    ros2 launch fleet_agent one_robot.launch.py robot_id:=R1 start_cell:="[2,8]"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    drive_mode = LaunchConfiguration("drive_mode")

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="R1"),
        DeclareLaunchArgument("drive_mode", default_value="simple",
                              description="simple = built-in controller, "
                                          "nav2 = send goals to Nav2"),
        DeclareLaunchArgument("start_x", default_value="2"),
        DeclareLaunchArgument("start_y", default_value="8"),

        Node(
            package="fleet_agent",
            executable="agent",
            name="fleet_agent",
            namespace=robot_id,
            output="screen",
            parameters=[{
                "robot_id": robot_id,
                "drive_mode": drive_mode,
                "start_cell": [
                    LaunchConfiguration("start_x"),
                    LaunchConfiguration("start_y"),
                ],
            }],
            remappings=[
                ("odom", "odom"),
                ("scan", "scan"),
                ("cmd_vel", "cmd_vel"),
            ],
        ),
    ])
