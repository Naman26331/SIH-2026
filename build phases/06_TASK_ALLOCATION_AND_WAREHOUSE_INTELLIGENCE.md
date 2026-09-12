# 06 — Task Allocation and Warehouse Intelligence

## 1. Task allocation

Suppose:

```text
T1 → shelf A
T2 → shelf F
T3 → shelf H
```

Robots:

```text
R1 battery 90%
R2 battery 52%
R3 battery 25%
```

Choose the robot with minimum expected cost, not simply the nearest robot.

## 2. Robot-task cost

```text
cost =
distance
+ congestion
+ battery risk
+ task urgency
+ predicted waiting
```

## 3. Auction model

For each task:

```text
Task broadcast
      ↓
R1 calculates bid
R2 calculates bid
R3 calculates bid
      ↓
Lowest valid cost wins
```

Example:

```text
R1 = 7.2
R2 = 4.1  ← winner
R3 = 9.4
```

## 4. Dynamic reassignment

If R2 fails:

```text
R2 unavailable
 ↓
release tasks
 ↓
rebroadcast
 ↓
R1/R3 calculate bids
 ↓
new robot takes task
```

## 5. Inventory database

Track:

```text
Product
Quantity
Location
Demand score
Weight
Size
```

Example:

```text
P001
Wireless Mouse
48 units
A17
Demand: HIGH
```

## 6. Flexible/randomized storage principle

Do not require every product type to have one fixed zone.

Instead:

```text
Product ID → exact digital location
```

This allows flexible storage.

The system can deliberately distribute frequently requested inventory across multiple suitable areas to reduce contention.

## 7. Demand score

A simple first version:

```text
demand_score =
recent_orders * recency_weight
+ weekly_orders * weekly_weight
+ trend * trend_weight
```

Later use a forecasting model.

## 8. Storage utility

For candidate location L:

```text
utility =
demand_benefit
- travel_cost
- congestion_cost
- movement_cost
```

Choose the location with the best utility.

## 9. Rebalancing rule

Never move inventory merely because another location is closer.

Move only when:

```text
expected_future_saving > movement_cost
```

This prevents endless inventory movement.

## 10. Demand prediction

Inputs:

```text
timestamp
product_id
historical orders
day of week
recent demand
```

Outputs:

```text
forecast demand
```

Start with simple statistical features and scikit-learn/XGBoost.

Do not start with deep learning.

## 11. Pre-positioning

If the system predicts:

```text
Mouse demand ↑
```

it can:

```text
identify suitable nearby inventory
 ↓
check congestion
 ↓
estimate movement cost
 ↓
pre-position if beneficial
```

## 12. Order batching

Group compatible tasks by:

- same storage zone
- same pod/shelf
- same direction
- same packing station

Goal:

```text
fewer trips
less congestion
higher robot utilization
```

## 13. Battery-aware scheduling

When battery is low:

```text
finish safe/current task if possible
 ↓
reserve charger
 ↓
navigate to charger
 ↓
release/reassign remaining tasks
```

## 14. Charging scheduler

For one charging station:

```text
R1 15%
R2 17%
R3 43%

R1 → charger
R2/R3 → continue
```

Avoid sending all low-battery robots to the same charger simultaneously.
