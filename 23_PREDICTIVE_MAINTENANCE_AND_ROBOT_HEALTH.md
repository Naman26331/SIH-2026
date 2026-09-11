# 23 — Predictive Maintenance & Robot Health

## Purpose
Detect early signs of robot degradation before they become failures.

## Health Signals
Track motor current, motor temperature, battery health, wheel velocity mismatch, localization quality, packet loss, CPU/RAM usage, navigation failures, and emergency stops.

## Health Score
Combine normalized signals into a weighted health score and classify:
```text
HEALTHY
WARNING
SERVICE_SOON
CRITICAL
```

## Anomaly Detection
Start with moving averages, threshold rules, z-score, or Isolation Forest.

## Maintenance Policy
If health degrades:
1. reduce new task assignments
2. prioritize inspection/charging
3. finish or safely hand off active work
4. remove robot from normal dispatch
5. create maintenance event

## Dashboard
Show health score, recent anomalies, battery condition, failure history, and recommended action.

## KPIs
Failures avoided, warning lead time, false alarm rate, mean time between failures, and fleet availability.
