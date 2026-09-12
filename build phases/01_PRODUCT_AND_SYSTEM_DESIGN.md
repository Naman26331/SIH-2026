# 01 — Product and System Design

## 1. Problem

Large warehouses use many autonomous mobile robots (AMRs). Centralized path planning creates:

- network latency
- Wi-Fi dead-zone risk
- server dependency
- single-point-of-failure risk
- congestion around shared routes

FLEET-X moves safety-critical coordination to the edge.

## 2. Product vision

FLEET-X is a warehouse operating layer in which:

- robots know their own state
- robots share position and future intent
- robots negotiate conflicts locally
- tasks can move between robots
- blocked aisles trigger local rerouting
- inventory locations can adapt to demand
- the dashboard observes the fleet without controlling every safety decision

## 3. Main modules

### A. Robot Agent

Responsible for:

- localization
- local map
- navigation
- obstacle detection
- intent generation
- communication
- conflict handling
- battery state
- task execution

### B. Fleet Coordination

Responsible for:

- robot discovery
- shared state
- path reservations
- conflict detection
- negotiation
- deadlock detection
- task reassignment

### C. Warehouse Intelligence

Responsible for:

- inventory database
- demand score
- storage optimization
- order batching
- congestion prediction
- pre-positioning

### D. Dashboard

Displays:

- warehouse map
- robot locations
- battery
- task status
- conflicts
- blocked aisles
- congestion
- KPIs

## 4. Real-world warehouse inspiration

A useful concept from Amazon's public robotics material is that inventory can be stored in randomized locations while software maintains knowledge of where each item is. This avoids requiring every product category to occupy one dedicated area and can reduce contention.

FLEET-X should not claim to reproduce Amazon. Instead, implement the general principle:

> physical storage can be flexible; digital inventory tracking makes flexible storage useful.

Add demand-aware rebalancing:

```text
Demand increases
      ↓
Expected future picking cost increases
      ↓
Check alternative locations
      ↓
If expected saving > movement cost
      ↓
Move/pre-position inventory
```

## 5. Two optimization loops

### Long-term loop

```text
Orders
 ↓
Demand prediction
 ↓
Inventory optimization
 ↓
Storage/rebalancing decisions
```

### Real-time loop

```text
Robot state
 ↓
Intent sharing
 ↓
Conflict prediction
 ↓
Negotiation
 ↓
Reservation / reroute
 ↓
Movement
```

## 6. Design principle

Do not optimize only for shortest distance.

Optimize for:

```text
Travel time
+ congestion
+ battery cost
+ task urgency
+ collision risk
+ waiting time
```

The fastest path is often not the shortest path.

## 7. Failure philosophy

The fleet should degrade gracefully:

```text
Normal network
→ decentralized coordination

Network degraded
→ local coordination / reduced speed

Network unavailable
→ local safety mode

Network restored
→ state synchronization
```

## 8. Suggested product name

Primary:

**FLEET-X**

Subtitle:

**Edge-Native Decentralized Intelligence for Autonomous Warehouses**
