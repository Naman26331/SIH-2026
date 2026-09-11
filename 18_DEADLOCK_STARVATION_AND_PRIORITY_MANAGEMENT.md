# 18 — Deadlock, Starvation & Priority Management

## Purpose
Prevent robots from waiting forever for one another and prevent low-priority tasks from being starved.

## Priority
```text
P0 — emergency/safety
P1 — critical order
P2 — normal order
P3 — background/rebalancing
```

Safety always overrides task priority.

## Wait-For Graph
If `A → B`, robot A is waiting for B. A cycle such as `A → B → C → A` indicates a potential deadlock.

## Deadlock Recovery
Detect cycle → choose resolution robot → temporarily prioritize it → release/reverse one reservation → replan affected robots → verify the cycle is removed.

## Starvation Prevention
```text
effective_priority =
    base_priority + aging_factor * waiting_time
```

Waiting tasks gradually gain priority.

## Tie-Breaking
Use higher priority, lower predicted delay, earlier reservation timestamp, then robot ID as deterministic final tie-break.

## Fairness
Track per-task and per-robot waiting time so one robot cannot repeatedly win every conflict.

## KPIs
Deadlocks, conflict resolution time, maximum task wait, starvation events, and fairness.
