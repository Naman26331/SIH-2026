# 15 — Intelligent Inventory Placement & Demand Optimization

## Purpose
Treat storage placement as an optimization problem instead of permanently assigning every SKU to a fixed location.

## SKU Profile
Track SKU ID, demand rate, order frequency, current location, compatible zones, replenishment rate, and priority.

## Objective
```text
PlacementCost =
    travel_cost
  + congestion_cost
  + replenishment_cost
  + handling_cost
```

Minimize expected cost while respecting warehouse constraints.

## Demand-Aware Placement
Place high-demand inventory closer to relevant picking/packing zones, while avoiding excessive concentration that could create congestion.

## Placement Score
```text
PlacementScore =
    demand_benefit
  - travel_cost
  - congestion_penalty
  - relocation_cost
```

Move inventory only when predicted future benefit exceeds relocation cost.

## Flexible Storage
The system can assign compatible open locations and track exact SKU locations digitally, allowing demand-aware rebalancing.

## Order Batching
Group compatible orders to reduce robot travel, repeated pod movement, intersection traffic, and unnecessary trips.

## KPIs
Pick travel distance, order completion time, relocation count, storage-zone congestion, utilization, and throughput.

## SIH Demo
Simulate a demand spike → detect it → predict impact → propose better locations → rebalance selected inventory → measure improvement.
