# 11 — SIH Demo and Presentation

## 1. Opening

Do not start with code.

Start with the problem:

> In a large warehouse, the hard problem is not making one robot move. The hard problem is coordinating hundreds of moving decisions when communication is delayed, aisles are blocked and demand changes.

Then introduce:

> FLEET-X is an edge-native decentralized warehouse intelligence platform.

## 2. Demo story

### Step 1 — Order

```text
ORDER #1042
Mouse ×2
Keyboard ×1
USB Hub ×1
```

### Step 2 — Inventory lookup

```text
Mouse → A14
Keyboard → C08
USB Hub → B21
```

### Step 3 — Task allocation

```text
R1 → Mouse
R2 → Keyboard
R3 → USB Hub
```

### Step 4 — Robots move

Show the digital twin.

### Step 5 — Conflict

Two robots approach an intersection.

Dashboard:

```text
CONFLICT PREDICTED
```

Then:

```text
R1 priority 78
R2 priority 61

R1 reservation granted
R2 rerouting
```

### Step 6 — Blocked aisle

Create an obstacle.

Show:

```text
BLOCKED A7
```

Other robots automatically replan.

### Step 7 — Failure

Disable R2.

Show:

```text
R2 heartbeat timeout
Tasks released
R1/R3 re-auction
```

### Step 8 — Network failure

Disconnect communication.

Show:

```text
NETWORK OFFLINE
SAFE EDGE MODE
```

Robots remain safe.

### Step 9 — Demand spike

Increase demand for Mouse.

Show:

```text
Demand: HIGH
```

System recommends/requires beneficial inventory redistribution.

### Step 10 — Benchmark

Show:

```text
Stop-and-wait: 8m 41s
FLEET-X:        6m 37s
Improvement:    23.8%
Collisions:     0
```

Use actual measured values, not fabricated numbers.

## 3. Slides

Recommended slide order:

1. Problem
2. Why centralized systems struggle
3. Proposed solution
4. System architecture
5. Decentralized robot communication
6. Conflict resolution
7. Dynamic rerouting
8. Warehouse intelligence
9. Digital twin
10. Edge/failure resilience
11. Benchmark results
12. Scalability
13. Technology stack
14. Future scope

## 4. Strong one-line pitch

> FLEET-X turns a fleet of independent AMRs into a self-coordinating warehouse system that can continue operating safely even when the central network cannot.

## 5. Innovation points

### 1. Intent-aware coordination

Robots share future intent, not only position.

### 2. Decentralized traffic negotiation

Robots negotiate shared resources locally.

### 3. Self-healing tasks

Tasks move automatically when a robot fails or becomes unavailable.

### 4. Demand-aware inventory

Inventory placement considers demand, travel cost and congestion.

### 5. Network-resilient operation

Safety does not depend on a cloud server.

## 6. Judge questions to prepare for

### Why not centralized?

Because the safety-critical loop should survive network latency/failure.

### Why ROS 2?

It provides a practical distributed robotics middleware and integrates well with simulation/navigation.

### Why A*?

Simple, explainable baseline; dynamic costs and reservations extend it.

### How do you guarantee zero collisions?

Do not claim mathematical guarantee unless formally proven. Say:

> We enforce collision prevention through local safety control plus time-aware fleet coordination, and we validate zero collisions across defined benchmark scenarios.

### How is this different from simple collision avoidance?

The system combines:

- communication
- intent
- reservations
- task allocation
- inventory intelligence
- failure recovery

### Why use AI?

AI is used where prediction helps; deterministic safety logic remains local and explainable.

## 7. Golden rule

Never show a feature without a measurable result.
