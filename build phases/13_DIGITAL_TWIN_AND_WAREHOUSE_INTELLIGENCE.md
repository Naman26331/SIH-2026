# 13 — Digital Twin & Warehouse Intelligence

## Purpose
Create a live software representation of the warehouse for monitoring, replay, analytics, and what-if simulation.

## Digital Twin State
Represent:
- Warehouse map, aisles, intersections, stations, chargers
- Robot pose, velocity, battery, task, destination, route
- Reservations and predicted conflicts
- Inventory/SKU locations
- Orders and task queues
- Blocked aisles and dynamic obstacles
- Congestion and traffic heatmaps

## Architecture
```text
ROS 2 Robot Nodes
      ↓
Fleet State Collector
      ↓
Backend / Event Stream
      ↓
Digital Twin State
      ↓
WebSocket
      ↓
React Dashboard
```

Safety-critical robot decisions remain local; the dashboard is not required for collision avoidance.

## Dashboard
Include:
1. Live warehouse map
2. Robot status
3. Active tasks
4. Conflict/reservation view
5. Congestion heatmap
6. Inventory heatmap
7. Battery/charging
8. Event timeline
9. KPI/benchmark view

## What-If Mode
Change robot count, order rate, blocked aisles, failures, battery levels, storage locations, and priorities, then compare simulation KPIs.

## Replay Mode
Store timestamped events and support pause/play, speed control, timeline scrubbing, filtering, and conflict highlighting.

## KPIs
Collision count, deadlocks, task completion time, throughput, waiting time, reroutes, utilization, congestion, and energy.
