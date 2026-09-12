# 12 — Team Task Breakdown

Assuming a 6-person SIH team.

## Member 1 — Robotics/ROS Lead

Own:

- ROS 2 workspace
- Gazebo
- robot model
- Nav2
- localization
- robot integration

Deliverable:

```text
simulated robot can navigate
```

## Member 2 — Multi-Agent Algorithms

Own:

- A*
- reservations
- conflict detection
- negotiation
- deadlock

Deliverable:

```text
3+ robots coordinate without collisions
```

## Member 3 — Communication/Backend

Own:

- custom messages
- DDS topics
- heartbeat
- FastAPI
- WebSocket
- PostgreSQL

Deliverable:

```text
real-time fleet state pipeline
```

## Member 4 — Warehouse Intelligence

Own:

- inventory model
- task allocation
- auction
- demand score
- storage optimization
- order batching

Deliverable:

```text
order → optimized tasks
```

## Member 5 — Frontend/Digital Twin

Own:

- React
- map
- robot visualization
- task dashboard
- KPI cards
- event feed

Deliverable:

```text
live fleet dashboard
```

## Member 6 — Testing/Hardware/Documentation

Own:

- benchmarks
- edge deployment
- Raspberry Pi/Jetson
- physical prototype
- test scenarios
- documentation
- SIH presentation

Deliverable:

```text
reproducible demo + benchmark evidence
```

## Team integration rule

Everyone works against defined interfaces.

Example:

```text
Robot team → publishes RobotState
Algorithm team → consumes RobotState
Backend team → consumes events
Frontend team → consumes WebSocket schema
```

Do not make everyone edit the same files.

## Git strategy

```text
main
develop
feature/ros2
feature/conflict-manager
feature/backend
feature/dashboard
feature/inventory
feature/testing
```

Merge only tested features.

## Weekly milestone

### Week 1

Architecture + simulator

### Week 2

Three robots

### Week 3

Communication

### Week 4

Conflict resolution

### Week 5

Dynamic rerouting

### Week 6

Task allocation

### Week 7

Dashboard

### Week 8

Inventory intelligence

### Week 9

Edge failure

### Week 10

Benchmark + polish

Adjust the schedule to the actual SIH timeline.
