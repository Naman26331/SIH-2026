# 04 — Decentralized Fleet Protocol

## 1. Goal

Create a small custom protocol on top of ROS 2/DDS to demonstrate that robots exchange meaningful fleet information.

## 2. Message types

### HEARTBEAT

```json
{
  "type": "HEARTBEAT",
  "robot_id": "R01",
  "timestamp": 172345,
  "battery": 82,
  "status": "MOVING"
}
```

### POSE_UPDATE

```json
{
  "type": "POSE_UPDATE",
  "robot_id": "R01",
  "x": 4.2,
  "y": 7.1,
  "velocity": 0.5
}
```

### INTENT_UPDATE

```json
{
  "type": "INTENT_UPDATE",
  "robot_id": "R01",
  "destination": "PICK_12",
  "planned_path": ["N12","N13","N14","N21"],
  "eta": 14.2,
  "priority": 4
}
```

### PATH_RESERVATION

```json
{
  "type": "PATH_RESERVATION",
  "robot_id": "R01",
  "node": "N14",
  "start": 12.0,
  "end": 14.0
}
```

### CONFLICT_ALERT

```json
{
  "type": "CONFLICT_ALERT",
  "robot_a": "R01",
  "robot_b": "R02",
  "resource": "N14",
  "estimated_time": 12.8
}
```

### BLOCKED_AISLE

```json
{
  "type": "BLOCKED_AISLE",
  "aisle": "A07",
  "confidence": 0.96,
  "ttl": 15
}
```

## 3. Intent sharing

Position-only:

```text
R1 is here.
```

Intent-aware:

```text
R1 is here.
R1 wants to go there.
R1 expects to occupy these nodes.
R1 expects to arrive at the intersection at this time.
```

The second approach enables predictive conflict handling.

## 4. Local fleet state

Every robot maintains a temporary local view:

```text
robots[]
reservations[]
blocked_areas[]
tasks[]
```

It should tolerate stale information.

## 5. Conflict protocol

```text
Robot detects predicted conflict
        ↓
Compare priorities
        ↓
Can current robot reserve resource?
        ↓
YES → reserve
NO  → negotiate
        ↓
wait OR reroute
        ↓
broadcast decision
```

## 6. Priority

Example:

```text
priority_score =
task_priority
+ urgency
+ waiting_time
+ battery_factor
```

Avoid starvation by increasing waiting time over time.

## 7. Tie breaking

If scores are equal:

```text
lower robot_id wins
```

This makes the system deterministic.

## 8. Network failure

The protocol must never assume every message arrives.

Use:

- timestamps
- sequence numbers
- TTL
- heartbeat timeout
- stale-state detection

## 9. Security for future versions

Potential additions:

- message authentication
- robot identity
- replay protection
- encrypted transport

Security is secondary to the SIH MVP, but mention it as future work.
