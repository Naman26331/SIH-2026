# 09 — Edge Hardware and Deployment

## 1. Recommended hardware

For a practical prototype:

### Option A

Raspberry Pi 5

### Option B

Jetson Orin Nano

### Existing/older prototype

Raspberry Pi 4 can be used for a lightweight robot agent.

## 2. What runs on edge

Each robot should run:

```text
ROS 2
Localization
Local obstacle processing
Fleet communication
Conflict manager
Path planning interface
Task execution
Battery monitor
Safety controller
```

## 3. What can run off-robot

```text
Dashboard
Historical analytics
Database
Demand training
Simulation
```

## 4. Edge architecture

```text
             Wi-Fi
R1 ───────────┬──────────── R2
              │
              └──────────── R3

Each robot is independently capable of safe local behavior.
```

## 5. Network failure

Test:

```text
normal
 ↓
disconnect
 ↓
safe local mode
 ↓
reconnect
 ↓
synchronize
```

## 6. Physical AMR

If building physical robots, use a differential-drive base.

Suggested components:

- Raspberry Pi/Jetson
- motor driver
- DC motors
- wheel encoders
- LiDAR or depth camera
- IMU
- battery
- emergency stop

Do not make the physical robot more complex than necessary.

## 7. Safety

Physical testing must include:

- low-speed operation
- emergency stop
- obstacle detection
- battery protection
- safe motor power
- controlled test area

## 8. Deployment

Use Docker where practical.

Suggested services:

```text
ros2_robot
fleet_gateway
postgres
dashboard
simulation
```

For actual robots, avoid putting safety-critical motor control behind a remote API.

## 9. Scaling

Prototype with one computer simulating multiple robots.

Then move one robot agent to a real Raspberry Pi.

Finally demonstrate multiple edge nodes if hardware permits.
