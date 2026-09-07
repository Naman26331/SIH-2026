# 10 — Development Roadmap

## Phase 0 — Team setup

Deliverables:

- Git repository
- issue board
- branch strategy
- architecture diagram
- coding conventions

## Phase 1 — Warehouse simulation

Build:

- ROS 2 workspace
- Gazebo world
- one robot
- map
- basic navigation

Success:

```text
R1 can move from A → B
```

## Phase 2 — Three robots

Build:

- R1
- R2
- R3
- unique namespaces
- independent navigation

Success:

```text
all robots navigate simultaneously
```

## Phase 3 — Communication

Build:

- RobotState
- RobotIntent
- heartbeat
- shared fleet state

Success:

```text
R1 can see R2/R3 intent
```

## Phase 4 — Conflict detection

Detect:

- same node
- same edge
- intersection conflict
- time overlap

## Phase 5 — Reservation

Build:

- reservation table
- intersection reservation
- edge reservation

## Phase 6 — Negotiation

Build:

- priority score
- tie-breaking
- wait/reroute decision
- starvation prevention

## Phase 7 — Deadlock

Build:

- wait-for graph
- cycle detection
- deadlock breaker

## Phase 8 — Dynamic obstacles

Build:

- blocked aisle
- local map update
- broadcast
- replan

## Phase 9 — Task allocation

Build:

- task queue
- robot cost
- auction
- dynamic reassignment

## Phase 10 — Dashboard

Build:

- map
- robot cards
- task list
- event feed
- KPI cards

## Phase 11 — Inventory intelligence

Build:

- inventory DB
- exact item location
- demand score
- storage scoring
- order batching

## Phase 12 — Demand prediction

Build:

- historical dataset
- baseline forecasting
- demand zones
- pre-positioning recommendation

## Phase 13 — Battery

Build:

- battery state
- charger
- charging queue
- task reassignment

## Phase 14 — Edge mode

Build:

- network disconnect test
- safe mode
- recovery
- state synchronization

## Phase 15 — Benchmarking

Compare:

```text
stop-and-wait
VS
FLEET-X
```

Generate graphs.

## Phase 16 — Final demo

Prepare:

1. Normal operation
2. conflict
3. negotiation
4. blocked aisle
5. rerouting
6. robot failure
7. network failure
8. demand spike
9. benchmark

## Priority rule

If time becomes short, cut advanced ML and 3D visualization before cutting:

- decentralized communication
- conflict resolution
- deadlock handling
- rerouting
- benchmark
