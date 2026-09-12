# 14 — Predictive Congestion & AI

## Purpose
Predict future congestion and use the prediction to improve routing, task allocation, inventory placement, and dispatch timing.

## Congestion Score
```text
CongestionScore =
    w1 * robot_density
  + w2 * reservation_density
  + w3 * predicted_wait
  + w4 * queue_length
  + w5 * blocked_probability
```

Normalize features to `[0,1]`.

## Features
- Robots per aisle
- Average speed
- Reservation count
- Historical wait time
- Task arrival rate
- Order density
- Blockage frequency

## Models
Start with Logistic Regression, Random Forest, Gradient Boosting, or XGBoost. Deep learning is not required for the MVP.

## Pipeline
```text
ROS 2 telemetry
      ↓
Feature extraction
      ↓
Congestion model
      ↓
Prediction
      ↓
Fleet planner
      ↓
Routing + task allocation
```

## Safe AI
AI recommends or scores options. Deterministic safety constraints remain authoritative.

```text
AI recommendation
      ↓
Safety constraints
      ↓
Conflict manager
      ↓
Approved action
```

## Evaluation
Measure prediction accuracy, congestion reduction, waiting time, throughput, reroutes, and completion time using separate evaluation scenarios.
