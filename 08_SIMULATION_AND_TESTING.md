# 08 — Simulation and Testing

## 1. Simulator

Use Gazebo with ROS 2.

Create:

- warehouse floor
- shelves
- aisles
- intersections
- pickup stations
- packing stations
- charging station
- obstacle objects

## 2. Start with a grid

A simple grid makes algorithm debugging easy.

Example:

```text
S S S . . . S
. . . . . . .
. . X . X . .
. . . . . . .
P . . . . . D
```

Later create a more realistic warehouse.

## 3. Robot count

Test:

```text
3 robots
5 robots
10 robots
20 robots
```

## 4. Scenario library

### Scenario 1 — Normal

No conflicts.

### Scenario 2 — Head-on

```text
R1 → X ← R2
```

Expected:

- no collision
- one reservation
- other robot yields/reroutes

### Scenario 3 — Intersection

Three or four robots approach the same intersection.

### Scenario 4 — Deadlock

Create a cyclic wait.

Expected:

- cycle detected
- one robot yields
- traffic recovers

### Scenario 5 — Blocked aisle

Robot detects an obstacle.

Expected:

- blocked aisle broadcast
- other robots replan

### Scenario 6 — Robot failure

Stop R2.

Expected:

- heartbeat timeout
- R2 marked unavailable
- tasks reassigned

### Scenario 7 — Network failure

Disable fleet communication.

Expected:

- robots enter local safe mode
- no unsafe movement
- recovery after network returns

### Scenario 8 — Low battery

Expected:

- charger reservation
- task reassignment if needed

### Scenario 9 — Demand spike

Increase demand for a product.

Expected:

- demand score changes
- storage/rebalancing recommendation

### Scenario 10 — High congestion

Fill one corridor with many robots.

Expected:

- congestion score rises
- planner chooses alternative routes

## 5. Benchmark

Create identical workloads.

### Baseline

Stop-and-wait.

### FLEET-X

Predictive intent + reservation + negotiation + rerouting.

Run each experiment multiple times.

## 6. Core formula

```text
improvement_percent =
((baseline_time - fleetx_time)
 / baseline_time) * 100
```

Target:

```text
>= 20%
```

## 7. Safety metrics

```text
collisions
near collisions
emergency stops
deadlocks
```

## 8. Efficiency metrics

```text
total completion time
average task time
tasks/hour
distance travelled
```

## 9. Traffic metrics

```text
average waiting
intersection utilization
congestion
reroutes
```

## 10. Network metrics

```text
message latency
packet loss
heartbeat recovery time
safe-mode duration
```

## 11. Experiment table

Keep a CSV:

```text
scenario
robot_count
baseline_time
fleetx_time
improvement
collisions
deadlocks
reroutes
avg_wait
distance
```

This becomes evidence for your SIH presentation.

## 12. Testing philosophy

Unit test:

- A*
- cost functions
- reservation conflicts
- priority
- deadlock detection
- auction
- inventory scoring

Integration test:

- ROS topics
- robot coordination
- dynamic rerouting

End-to-end test:

```text
order → task → robot → pickup → delivery
```
