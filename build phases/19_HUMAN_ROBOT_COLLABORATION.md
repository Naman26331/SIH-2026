# 19 — Human-Robot Collaboration

## Purpose
Model human workers and make robots react conservatively around them.

## Human State
Track position, velocity, direction, short-horizon predicted position, and safety zone.

## Safety Zones
```text
Emergency zone → stop
Caution zone   → slow/replan
Normal zone    → normal operation
```

Actual distances must be determined from the robot platform, sensors, speed, and applicable safety requirements.

## Worker-Aware Routing
Avoid crowded aisles, reduce speed near workers, temporarily reserve pedestrian-heavy areas, and reroute around human blockages.

## Prediction
For simulation use constant-velocity prediction or a simple Kalman filter. Keep predictions conservative.

## Scenarios
Worker crosses an aisle, stops at an intersection, approaches a robot, multiple workers create congestion, or a narrow aisle becomes blocked.

## KPIs
Safety-zone violations, emergency stops, waiting time, worker-aware reroutes, and completion time.

## Principle
Human safety always has higher priority than throughput.
