# 03 — Robot and ROS 2 Implementation

## 1. Workspace

Suggested ROS 2 workspace:

```text
ros2_ws/
└── src/
    ├── fleet_msgs/
    ├── fleet_agent/
    ├── fleet_bringup/
    ├── conflict_manager/
    ├── reservation_manager/
    ├── task_manager/
    ├── inventory_manager/
    ├── warehouse_planner/
    ├── demand_predictor/
    └── robot_controller/
```

## 2. Custom message package

Create messages for:

```text
RobotState.msg
RobotIntent.msg
ConflictAlert.msg
PathReservation.msg
Task.msg
BlockedAisle.msg
BatteryStatus.msg
```

Example RobotIntent:

```text
string robot_id
builtin_interfaces/Time timestamp
float32 x
float32 y
float32 velocity
string destination
string[] planned_nodes
float32 eta_intersection
int32 priority
string status
```

## 3. Important topics

```text
/fleet/robot_states
/fleet/robot_intents
/fleet/conflicts
/fleet/reservations
/fleet/tasks
/fleet/blocked_aisles
/fleet/battery
```

Robot-specific:

```text
/robot_1/odom
/robot_1/scan
/robot_1/cmd_vel
/robot_1/localization
```

## 4. Robot Agent

The agent should run a loop:

```text
1. Read local state
2. Update local map
3. Publish state
4. Publish intent
5. Check nearby robot intents
6. Predict conflicts
7. Request/issue reservation
8. Execute safe movement
9. Detect obstacles
10. Replan if needed
11. Update task status
```

## 5. Heartbeat

Each robot periodically publishes:

```text
robot_id
timestamp
health
battery
position
```

If another robot has not been heard from within a timeout:

```text
mark robot stale
release its future reservations
avoid its predicted trajectory
```

## 6. Localization

For simulation, start with simulator odometry.

Then optionally add:

- AMCL
- LiDAR
- SLAM Toolbox

Do not begin with physical SLAM. First prove fleet coordination.

## 7. Navigation

Use Nav2 for:

- map handling
- localization integration
- costmaps
- controller
- path following

Keep your custom logic above/beside Nav2 rather than rewriting the entire navigation stack.

## 8. Safety layer

The local safety layer must have authority to stop/slow the robot.

Example:

```text
Unexpected obstacle
       ↓
Local sensor
       ↓
Safety controller
       ↓
STOP / SLOW
```

This should not wait for the dashboard.

## 9. Edge mode

When communication is unavailable:

```text
NETWORK OFF
   ↓
reduce speed
   ↓
use local obstacle avoidance
   ↓
finish safe maneuver / stop
   ↓
periodically attempt recovery
```

When network returns:

```text
SYNC STATE
SYNC TASK
SYNC RESERVATIONS
RESUME
```
