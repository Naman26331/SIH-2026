# FLEET-X — Decentralized Autonomous Warehouse

## Purpose

FLEET-X is an SIH 2026 implementation blueprint for a decentralized multi-robot warehouse.

The project combines:

1. Decentralized robot-to-robot communication
2. Multi-agent collision avoidance
3. Intersection reservation and negotiation
4. Deadlock detection and recovery
5. Dynamic task allocation and reassignment
6. Blocked-aisle detection and rerouting
7. Battery-aware fleet management
8. Demand-aware inventory placement
9. Order batching
10. Congestion prediction
11. A real-time fleet dashboard
12. Edge-first operation during network failure

## Core pitch

> The warehouse learns where inventory should be, while every robot learns how it should move.

The central server is **not safety-critical**. Robots should be able to coordinate locally and enter a safe local mode if the network disappears.

## Recommended MVP

Start with:

- 3 simulated AMRs
- A warehouse grid
- ROS 2 + Gazebo
- A* path planning
- ROS 2/DDS peer communication
- Intent sharing
- Intersection reservation
- Conflict resolution
- Dynamic rerouting
- React dashboard

Then add task allocation, inventory intelligence, demand prediction, battery management and failure recovery.

## Success metrics

The SIH target is:

- 0 inter-robot collisions
- At least 20% reduction in task completion time versus stop-and-wait

Also measure:

- deadlocks
- waiting time
- reroutes
- distance travelled
- robot utilization
- network latency
- packet loss
- battery/energy estimate
- tasks per hour

## Repository

```text
FLEET-X/
├── 00_README.md
├── 01_PRODUCT_AND_SYSTEM_DESIGN.md
├── 02_TECHNICAL_ARCHITECTURE.md
├── 03_ROBOT_AND_ROS2_IMPLEMENTATION.md
├── 04_DECENTRALIZED_FLEET_PROTOCOL.md
├── 05_PATH_PLANNING_AND_COLLISION_AVOIDANCE.md
├── 06_TASK_ALLOCATION_AND_WAREHOUSE_INTELLIGENCE.md
├── 07_DASHBOARD_AND_DIGITAL_TWIN.md
├── 08_SIMULATION_AND_TESTING.md
├── 09_EDGE_HARDWARE_AND_DEPLOYMENT.md
├── 10_DEVELOPMENT_ROADMAP.md
├── 11_SIH_DEMO_AND_PRESENTATION.md
└── 12_TEAM_TASK_BREAKDOWN.md
```
