# 16 — Battery, Charging & Energy Management

## Purpose
Prevent low battery from causing task failures and fleet degradation.

## Battery State
Track battery percentage, remaining runtime, charging state, current task, predicted energy requirement, charger, and health estimate.

## Battery-Aware Task Score
```text
TaskScore =
    task_priority
  - distance_cost
  - congestion_cost
  - battery_risk
```

Reject or defer tasks when predicted energy is insufficient.

## Charging States
```text
NORMAL
LOW_BATTERY
RETURN_TO_CHARGER
CHARGING
RECOVERY
```

## Predictive Charging
Estimate future demand, reserve chargers, avoid simultaneous unnecessary charging, and charge lower-priority robots during demand valleys.

## Low-Battery Recovery
Stop accepting new tasks → safely finish/hand off current work → reserve charging route → release unnecessary reservations → navigate to charger.

## KPIs
Energy per task, charger utilization, low-battery failures, idle charging time, and tasks per battery cycle.
