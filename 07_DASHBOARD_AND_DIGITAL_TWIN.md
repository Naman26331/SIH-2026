# 07 — Dashboard and Digital Twin

## 1. Goal

Build a lightweight warehouse control-room UI.

It should observe the fleet in real time.

## 2. Main screen

```text
┌──────────────────────────────────────────────┐
│ FLEET-X CONTROL CENTER                      │
├──────────────────────────────────────────────┤
│ Robots 6 | Active 5 | Charging 1            │
│ Tasks 24 | Completed 18 | Collisions 0      │
├──────────────────────────────────────────────┤
│                                              │
│             WAREHOUSE MAP                    │
│                                              │
│     R1 →              R3 ↓                  │
│                                              │
│          R2 ↑            R5 →               │
│                                              │
├──────────────────────────────────────────────┤
│ ROBOT STATUS                                 │
│ R1  84%  Task 12  MOVING                    │
│ R2  62%  Task 08  WAITING                   │
│ R3  91%  Task 13  MOVING                    │
└──────────────────────────────────────────────┘
```

## 3. Required dashboard views

### Fleet

- robot position
- battery
- velocity
- task
- state

### Map

- shelves
- aisles
- intersections
- robots
- blocked regions
- reservations

### Tasks

- queued
- assigned
- active
- completed
- failed

### Inventory

- product
- quantity
- location
- demand score

### Traffic

- congestion
- conflicts
- deadlocks
- waiting time

### Performance

- completion time
- tasks/hour
- distance
- collisions
- reroutes

## 4. Digital twin

The UI is a live representation of:

```text
warehouse state
+
robot state
+
inventory state
+
task state
+
traffic state
```

## 5. Realtime architecture

```text
ROS 2
 ↓
Fleet Gateway
 ↓
WebSocket
 ↓
React
```

Avoid polling every second if you can stream events.

## 6. Recommended frontend

```text
React
TypeScript
Tailwind
```

For the warehouse map, begin with:

```text
SVG / Canvas
```

Add Three.js only if a 3D visualization genuinely improves the demo.

## 7. Important UX

Use clear states:

```text
MOVING
WAITING
NEGOTIATING
REROUTING
BLOCKED
CHARGING
FAILED
SAFE MODE
```

Highlight important events:

```text
⚠ Conflict predicted
↻ Rerouting
✓ Reservation granted
⚡ Battery low
📡 Network disconnected
```

## 8. KPI cards

Show:

```text
Collision count
Average task time
Time saved
Robot utilization
Average waiting
Deadlocks
Reroutes
Network latency
```

## 9. Comparison mode

Provide:

```text
Traditional stop-and-wait
VS
FLEET-X
```

with identical task sets.

This is critical for proving the 20% improvement.
