# 17 — Failure Recovery & Network Resilience

## Purpose
Keep the fleet safe and productive when a robot fails or communication becomes unreliable.

## Robot Failure Detection
Use heartbeat timeout, stale pose, missed acknowledgements, abnormal battery state, and node/process failure.

## Recovery Flow
```text
Heartbeat lost
     ↓
Mark robot unavailable
     ↓
Invalidate future reservations
     ↓
Find affected tasks
     ↓
Reassign tasks
     ↓
Other robots replan
```

## Network Failure
A disconnected robot retains its local map and safety constraints, uses conservative behavior, avoids uncertain cooperative commitments, and synchronizes state after reconnection.

## Operating Modes
```text
CONNECTED   → normal coordination
DEGRADED    → lower speed + larger safety margins
DISCONNECTED→ local safety + conservative routing
RECOVERING  → state synchronization + conflict reconciliation
```

## State Synchronization
Exchange identity/epoch, pose/status, task state, and reservations; reject stale data before resuming normal operation.

## Tests
Robot failure at intersection, failure during a task, 5/30-second network loss, delayed messages, duplicates, and stale intents.

## KPIs
Recovery time, recovered tasks, collisions, deadlocks, downtime, and stale-state duration.
