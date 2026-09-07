# 02 — Technical Architecture

## 1. Recommended stack

| Layer | Technology |
|---|---|
| OS | Ubuntu 24.04 |
| Robotics | ROS 2 |
| Navigation | Nav2 |
| Simulator | Gazebo |
| Robot algorithms | C++ + Python |
| Global planner | A* |
| Multi-agent planning | Reservation + Cooperative A*/CBS |
| Communication | ROS 2 DDS + custom messages |
| Backend | FastAPI |
| Database | PostgreSQL |
| Realtime | WebSocket |
| Frontend | React + TypeScript |
| UI | Tailwind CSS |
| Visualization | SVG/Canvas first, Three.js later |
| ML | Python, scikit-learn/XGBoost |
| Containers | Docker |
| Testing | pytest + ROS 2 tests |
| Version control | Git/GitHub |

## 2. High-level architecture

```text
                 React Dashboard
                       |
                  WebSocket/API
                       |
                 FastAPI Gateway
                       |
                  PostgreSQL
                       |
             ---------------------
             |                   |
       Warehouse Brain      Fleet Gateway
             |                   |
             -------- ROS 2/DDS --
                       |
       ---------------------------------
       |               |               |
      R1              R2              R3
       |               |               |
   Local Agent     Local Agent     Local Agent
       |               |               |
   Nav2/A*          Nav2/A*          Nav2/A*
       |               |               |
    Sensors         Sensors         Sensors
```

## 3. Critical architecture rule

The dashboard/backend is for:

- monitoring
- analytics
- inventory data
- task visibility
- configuration

It must not be the only component capable of preventing a robot collision.

Safety-critical decisions happen locally.

## 4. Robot software layers

```text
Sensors
  ↓
Localization
  ↓
Local Costmap
  ↓
Global Planner
  ↓
Fleet Conflict Manager
  ↓
Local Controller
  ↓
Motor Interface
```

Parallel services:

```text
Battery Monitor
Fleet Communication
Task Manager
Health/Heartbeat
```

## 5. Data model

### Robot

```text
robot_id
position
orientation
velocity
battery
status
current_task
destination
last_seen
```

### Inventory

```text
product_id
name
quantity
location
demand_score
weight
size
```

### Task

```text
task_id
product_id
pickup
dropoff
priority
assigned_robot
status
created_at
deadline
```

### Reservation

```text
reservation_id
robot_id
node/edge
start_time
end_time
priority
status
```

### Event

```text
event_id
timestamp
robot_id
event_type
payload
```

## 6. Cost function

A generic robot-task cost:

```text
C =
w1 * distance
+ w2 * congestion
+ w3 * battery_cost
+ w4 * urgency_penalty
+ w5 * predicted_wait
```

Tune weights experimentally.

## 7. Deployment modes

### Simulation

Laptop/desktop:

```text
Gazebo + ROS 2 + Dashboard + Database
```

### Edge prototype

```text
Raspberry Pi / Jetson
      ↓
ROS 2 robot agent
```

Use the laptop as the simulator and development machine; do not force the Pi to run a large simulation.

## 8. Scalability

The same robot agent should work for:

```text
3 robots
5 robots
10 robots
20 robots
```

Use simulation to demonstrate scaling.
