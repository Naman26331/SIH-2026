# Backend

Runs fleet intelligence and robot integration.

Run backend:

```text
uv run backend/run.py
```

```text
shared/fleetx_core/  pure Python planning and coordination brain
simulator/           temporary grid simulation engine and API
ros2_ws/             ROS 2 nodes, messages, robot and simulator integration
tests/               automated behaviour and safety tests
tools/               validation utilities
```

Run tests from repository root:

```text
uv run --no-project -m unittest discover -s backend/tests
```

Run ROS 2 packages on the Raspberry Pi:

```text
cd backend/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch fleet_agent fleet.launch.py robots:=3
```

Gazebo bringup still targets Gazebo Classic and needs migration for Jazzy/Harmonic.
