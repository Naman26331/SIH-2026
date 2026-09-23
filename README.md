# FLEET-X — Decentralized Autonomous Warehouse

**Smart India Hackathon (SIH) 2026**

> *"The warehouse learns where inventory should be, while every robot learns how it should move."*

FLEET-X is a fully decentralized, multi-agent autonomous mobile robot (AMR) coordination system. The central server is **non-safety-critical**: path planning, collision avoidance, conflict negotiation, deadlock recovery, and job dispatching are executed entirely on edge by peer-to-peer robot collaboration.
> *"Youtube: https://www.youtube.com/@TeamF%C3%BCego2026 "*
---

## 1. System Architecture

The core philosophy of FLEET-X is a **unified shared brain** (`backend/shared/fleetx_core`): pure Python logic without web or ROS dependencies. The exact same decision algorithms drive both the fast browser simulation and the physical/Gazebo ROS 2 robots.

```text
                  ┌────────────────────────────────────────┐
                  │   Shared Robot Brain (fleetx_core)     │
                  │   SIPP Planning · PIBT · Auctions      │
                  └───────────────────┬────────────────────┘
                                      │
                 ┌────────────────────┴────────────────────┐
                 ▼                                         ▼
     ┌───────────────────────┐                 ┌───────────────────────┐
     │     Simulation        │                 │     ROS 2 / Real      │
     │     InMemoryBus       │                 │       Ros2Bus         │
     │ (Simulated delay/drop)│                 │   (DDS Topic Network) │
     └───────────┬───────────┘                 └───────────┬───────────┘
                 │                                         │
                 ▼                                         ▼
         Python Simulator                          Gazebo / Real AMR
        (backend/simulator)                       (/cmd_vel, Nav2, /scan)
                 │                                         │
                 └────────────────────┬────────────────────┘
                                      │
                                      ▼
                        Gateway Node / Backend API
                                      │  (WebSocket)
                                      ▼
                           Laptop 2D Dashboard
```

- **Simulation Mode**: Uses `InMemoryBus` to model imperfect Wi-Fi (synthetic packet loss, latency, jitter) for stress testing.
- **ROS 2 Mode**: Uses `Ros2Bus` to bridge the brain to ROS 2 DDS topics without altering the logic.

---

## 2. Decentralized Task Management

Tasks represent real warehouse jobs: **two-leg journeys** (traverse to shelf face $\rightarrow$ pick product $\rightarrow$ deliver to packing station).

```text
  Customer Order Broadcast (/fleet/tasks)
                 │
                 ├── 1. Battery Feasibility Gate
                 │      (Only robots with enough charge to finish & reach charger bid)
                 │
                 ├── 2. Independent Cost Evaluation
                 │      Cost = 1.0·Distance + 2.0·Congestion + 1.0·Waiting - 1.5·Urgency
                 │
                 ├── 3. P2P Auction Winner Determination
                 │      (Lowest bid wins; deterministic lower robot ID tie-breaker)
                 │
                 └── 4. Dynamic Execution & Reassignment
                        (Flexible drop-off bay selection; automatic peer task release on failure)
```

1. **Battery Feasibility Gate**: A robot will only bid if its remaining battery allows it to reach the pickup, travel to dropoff, and subsequently reach a charging station. Extra battery does not bias distance calculations.
2. **Decentralized Bidding**: Each eligible robot computes a localized cost score:
   $$\text{Cost} = 1.0 \cdot d_{\text{total}} + 2.0 \cdot C_{\text{congestion}} + 1.0 \cdot t_{\text{waiting}} - 1.5 \cdot \max(0, \text{priority} - 5)$$
3. **Deterministic Auction**: Bids are published on `/fleet/task_bids`. At auction close, every robot independently determines the winner (`min(bid_rank)`). No central server assigns the job.
4. **Flexible Drop-offs**: Tasks can be marked flexible, allowing the winning robot to dynamically target the nearest uncongested packing station rather than queuing at a single bay.
5. **Fault Tolerance & Peer Release**: If an assigned robot stops broadcasting heartbeats, peers automatically revoke the claim and return the task to `ANNOUNCED` state for immediate re-auction.

---

## 3. Navigation, Conflict Resolution & Deadlocks

- **SIPP (Safe Interval Path Planning)**: Rather than naive grid A\*, robots compute collision-free routes over safe time intervals extracted from local reservation tables.
- **P2P Space-Time Reservations**: Robots book nodes, edges (head-on protection), and targets across short lookahead horizons ($k=3$ steps). Conflicts are resolved deterministically by effective priority and robot ID.
- **PIBT (Priority Inheritance Backtracking)**: When a higher-priority robot is obstructed, it issues a `YieldRequest`. The obstructing robot inherits the priority and either sidesteps or recursively pushes lower-priority peers ahead of it.
- **Fairness & Aging**: Waiting robots accrue $+1.0$ priority credit per second stuck (capped at 30 s) to eliminate starvation. Credits decay slowly once moving to prevent cycling.
- **Deadlock Detection**: Each robot builds a local `WaitForGraph` from `/fleet/wait_reports`. If a cycle ($R_1 \rightarrow R_2 \rightarrow R_3 \rightarrow R_1$) or an idle blocking robot is detected, the lowest-wait victim yields and moves aside.
- **Dynamic Obstacles**: Obstacles sensed via LIDAR are added to the local obstacle map, broadcast via `BLOCKED_AISLE`, and trigger instant SIPP rerouting.

---

## 4. Running the Project

### Simulation Mode (No ROS Required)
Runs full Python simulation, SIPP pathing, task auctions, and dashboard controls.

**Terminal 1 (Backend):**
```powershell
uv run backend/run.py
```

**Terminal 2 (Frontend Dashboard):**
```powershell
uv run frontend/run.py
```
Open [http://localhost:3000](http://localhost:3000) to view the live dashboard.

---

### ROS 2 / Gazebo Mode (Ubuntu 24.04 / Jazzy)

**1. Build Workspace:**
```bash
cd backend/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Launch Gazebo & Fleet:**
```bash
# Launch Gazebo simulation with 3 robots
ros2 launch fleet_bringup warehouse.launch.py robots:=3

# Or launch agent brains standalone (DDS-only test)
ros2 launch fleet_agent fleet.launch.py robots:=3
```

**3. Run Live Dashboard Gateway:**
```bash
# On robot / Pi:
bash backend/run_ros2.sh

# On laptop:
uv run frontend/run.py http://<ROBOT-IP>:8000
```
