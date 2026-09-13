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
- Browser dashboard (HTML/CSS/JavaScript)

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

Current code layout:

```text
backend/
├── shared/fleetx_core/  planning and decentralized fleet brain
├── simulator/           temporary grid simulation engine and API
├── ros2_ws/             ROS 2 integration for Raspberry Pi/robots
├── tests/               automated safety and behaviour checks
└── tools/               benchmarks
frontend/
├── dashboard/           2D warehouse monitoring UI
└── run.py               frontend server and backend proxy
```

Data path: `ROS 2/DDS → backend gateway → WebSocket → laptop dashboard`.
Backend never serves frontend files. Frontend never imports ROS code.

## Run

Frontend developer; no ROS required. PowerShell:

```powershell
$env:FLEETX_SIMULATION="true"; uv run backend/run.py
```

This is also default when flag is unset. Full Python simulation, A* pathing,
tasks and dashboard controls remain active.

Raspberry Pi live ROS mode, after one ROS workspace build:

```bash
bash backend/run_ros2.sh
```

Laptop:

```powershell
uv run frontend/run.py http://<PI-IP>:8000
```
