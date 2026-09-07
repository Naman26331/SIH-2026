# 05 — Path Planning and Collision Avoidance

## 1. Warehouse as a graph

Represent the warehouse as:

```text
N1 ── N2 ── N3 ── N4
      │     │
      N5    N6
      │     │
      N7 ── N8
```

Nodes represent positions/intersections.

Edges represent traversable aisle segments.

## 2. A* baseline

A* uses:

```text
f(n) = g(n) + h(n)
```

Where:

- g(n) = cost from start to n
- h(n) = estimated cost from n to goal

Start with distance as the cost.

## 3. Make the cost dynamic

Later:

```text
edge_cost =
distance
+ congestion_penalty
+ blocked_penalty
+ reservation_penalty
+ energy_penalty
```

This allows the robot to choose a slightly longer but faster route.

## 4. Reservation table

Example:

```text
NODE   TIME       ROBOT
N7     10-12      R1
N7     12-14      R2
N8     14-16      R3
```

A robot cannot reserve an occupied time interval.

## 5. Edge conflicts

Do not check only nodes.

Two robots can collide while traversing the same edge in opposite directions:

```text
R1 →──────← R2
```

Therefore reserve:

```text
edge + time interval
```

## 6. Intersection reservation

Robot asks:

```text
I7
ETA = 12.4
Exit = 14.8
```

If free:

```text
GRANTED
```

Otherwise:

```text
REROUTE
```

or:

```text
WAIT
```

## 7. Conflict prediction

For each pair of robots:

```text
Predict future positions
Compare shared nodes/edges
Compare time windows
```

If:

```text
same resource
AND
overlapping time
```

then create a conflict.

## 8. Deadlock detection

Maintain a wait-for graph:

```text
R1 → R2
R2 → R3
R3 → R1
```

A cycle means deadlock.

Resolution:

```text
detect cycle
 ↓
select robot to yield
 ↓
release reservation
 ↓
reroute/wait
 ↓
verify cycle gone
```

## 9. Starvation prevention

A robot waiting too long should gain priority.

Example:

```text
effective_priority =
base_priority + waiting_seconds * aging_factor
```

## 10. Dynamic blocked aisle

```text
sensor detects obstacle
        ↓
local map update
        ↓
BLOCKED_AISLE broadcast
        ↓
robots invalidate affected paths
        ↓
A* replanning
        ↓
reservation update
```

Use TTL so temporary obstacles eventually expire.

## 11. Recommended algorithm progression

### MVP

A* + reservations

### Improved

A* + reservations + negotiation

### Advanced

Cooperative A* / CBS comparison

Do not implement every multi-agent planning algorithm at once.

## 12. Emergency avoidance

Planning is not enough.

If a person or object suddenly appears:

```text
local sensor
 ↓
emergency stop/slow
```

The safety controller takes precedence over fleet optimization.
