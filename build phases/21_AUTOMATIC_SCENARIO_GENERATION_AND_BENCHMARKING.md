# 21 — Automatic Scenario Generation & Benchmarking

## Purpose
Generate repeatable scenarios and objectively compare FLEET-X with a traditional stop-and-wait baseline.

## Scenario Parameters
Vary robot count, warehouse size, order arrival rate, task density, obstacle probability, blocked-aisle probability, battery distribution, communication latency, robot failures, and demand distribution.

## Reproducibility
Store scenario ID, random seed, software version, map version, algorithm configuration, and simulation duration.

## Baseline
Implement a simple stop-and-wait policy with overlapping paths and no predictive congestion or inventory optimization. Keep the comparison fair.

## Metrics
- Total task completion time
- Average task time
- Throughput
- Collision count
- Deadlocks
- Waiting time
- Distance travelled
- Reroutes
- Robot utilization
- Energy
- Communication latency

## Target Success Metric
```text
Improvement (%) =
    (BaselineTime - FLEETXTime)
    / BaselineTime * 100
```

Target: zero inter-robot collisions and at least 20% reduction in total task completion time. Report only values actually measured in repeated simulations.

## Statistical Validation
Run multiple random seeds and report mean, median, standard deviation, and confidence intervals where practical.

## Automated Runner
```text
Scenario Generator
      ↓
Launch simulation
      ↓
Run baseline
      ↓
Run FLEET-X
      ↓
Collect metrics
      ↓
CSV/JSON
      ↓
Charts/report
```
