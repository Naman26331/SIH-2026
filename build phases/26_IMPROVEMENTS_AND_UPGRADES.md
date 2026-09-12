# 26 — Improvements & Upgrades (Gap Analysis vs SIH PS)

## Purpose
FLEET-X (docs 00-25) already covers the SIH problem statement in full and goes well beyond it. This document lists what is still missing or under-argued, and the concrete upgrades that would make the project stand out to judges. Ordered roughly by priority.

---

## 1. Missing: Proof That Decentralization Solves the Stated Problem

The benchmark in doc 21 only compares FLEET-X vs. **stop-and-wait**. That proves the coordination algorithm is faster — it does not prove decentralization itself was necessary, since a centralized planner could also beat stop-and-wait.

The PS's actual justification for decentralization is: *centralized systems suffer under network latency, Wi-Fi dead zones, and single-point failure.* That claim is never tested directly anywhere in the docs.

### Upgrade
Add a **second baseline**: a centralized planner, run under injected network latency / packet loss / simulated dead-zones. Show:

```text
Centralized planner + latency injection → stalls / degrades / collides
FLEET-X (decentralized) + same latency  → graceful degradation, still safe
```

This is the one experiment that directly answers "why not centralized?" with data instead of an argument.

**Priority: HIGH — do this first.**

---

## 2. Missing: Continuous-Space / Geometric Collision Layer

All conflict handling described (reservations, intersection negotiation, A*) is **discrete/graph-based**. It prevents conflicts at nodes/edges but says nothing about continuous-space safety between graph points — e.g. a robot drifting off its planned edge, or two robots on adjacent edges getting geometrically too close.

### Upgrade
Add a lightweight **velocity-obstacle / ORCA-style local avoidance layer** underneath the graph reservation system.

```text
Graph reservations  → traffic-level coordination (who goes first)
ORCA / VO layer      → geometric collision guarantee in continuous space
```

Two-layer safety argument is much stronger than reservations alone for judge Q&A.

**Priority: HIGH.**

---

## 3. Risk: DDS Discovery Reliability on Real Wi-Fi

Doc 02/03 use ROS 2 DDS as the "decentralized" transport. Default Fast-DDS relies on multicast discovery, which is often unreliable on consumer/enterprise Wi-Fi — exactly the network conditions the PS is concerned about. If the live demo runs on real APs instead of localhost, discovery can silently fail.

### Upgrade
- Test discovery reliability on the actual demo Wi-Fi **early**, not during setup week.
- Consider **Zenoh** as the RMW implementation (ROS 2 supports it as a drop-in). It's designed for edge/Wi-Fi/lossy scenarios and strengthens the "edge-native" pitch with a real technical choice instead of an assertion.

**Priority: HIGH — demo-breaking risk if skipped.**

---

## 4. Missing: Scalability / Complexity Story for Conflict Detection

Pairwise conflict checking is O(n²) in robot count. Nothing in the docs addresses this, even though scalability is one of the stated innovation claims (doc 22, doc 02 §8).

### Upgrade
Add spatial partitioning (grid buckets or a quadtree) so each robot only checks conflicts with nearby robots, not the whole fleet.

```text
Before: every robot checks every other robot → O(n²)
After:  every robot checks only its spatial neighbors → sub-quadratic
```

One line on a slide — "conflict checking is spatially localized, so it scales sub-quadratically" — is a low-effort, high-value technical differentiator.

**Priority: MEDIUM.**

---

## 5. Missing: Decision Explainability in the Dashboard

Priority, cost, and congestion scores are tracked internally, but the dashboard (doc 07) only shows resulting states (MOVING/WAITING/etc.), not *why* a decision was made.

### Upgrade
Add a click-to-inspect panel: click a robot mid-conflict → show its live cost breakdown:

```text
cost = distance + congestion + battery_risk + urgency_penalty + predicted_wait
     → final priority score → tie-break reasoning
```

Turns the coordination logic from a black box into something defensible live, and gives a strong demo beat between "conflict predicted" and "reservation granted."

**Priority: MEDIUM-HIGH — cheap, high demo impact.**

---

## 6. Missing: Live-Demo Failure Backup Plan

Nothing in doc 11 addresses what happens if Gazebo, Wi-Fi, or a robot crashes mid-pitch.

### Upgrade
- Pre-record a video of the exact demo sequence as a fallback.
- Use the existing **Replay Mode** (doc 13) as a live substitute if the real simulation breaks — the infrastructure for this already exists, it just needs to be planned as a fallback path, not only an analytics feature.

**Priority: MEDIUM — cheap insurance.**

---

## 7. Missing: Grounding in a Real Safety Standard

Safety zones (doc 19) are designed but never anchored to any real-world standard.

### Upgrade
Reference **ISO 3691-4** (industrial AMR safety) when describing safety-zone thresholds — even a line like "safety zone thresholds are informed by ISO 3691-4 guidance" signals engineering maturity to judges with industry backgrounds.

**Priority: LOW-MEDIUM — cheap credibility boost.**

---

## 8. Underused: Sustainability / Energy Angle

Battery (doc 16) is treated purely as an operational constraint, not an optimization target. SIH scoring often rewards a sustainability narrative.

### Upgrade
Add **energy-aware routing** as an explicit optimization objective alongside time and congestion:

```text
cost = distance + congestion + battery_risk + urgency_penalty + energy_cost
```

Report a headline metric: "X% lower total fleet energy consumption" alongside the 20% time-improvement number.

**Priority: LOW-MEDIUM.**

---

## 9. Missing: CI / Automated Regression Testing

Doc 08 mentions pytest but no pipeline.

### Upgrade
Add a GitHub Actions workflow that runs unit tests (A*, reservation conflicts, deadlock detection, auction logic) on every push. Trivial to set up; signals "engineered system" rather than "student project" when judges ask about process.

**Priority: LOW.**

---

## 10. Missing: Drift Correction for Physical Localization

AMCL/SLAM is deferred as "later" (doc 03), but on real hardware with cheap sensors, drift shows up fast.

### Upgrade
Use **fiducial markers (ArUco tags)** at fixed warehouse points for ground-truth re-localization. Low effort, large reliability payoff specifically for the physical prototype demo.

**Priority: LOW — only matters if a physical robot demo is planned.**

---

## Priority Summary

| # | Item | Priority | Effort |
|---|---|---|---|
| 1 | Centralized-planner-under-latency baseline | HIGH | Medium |
| 2 | ORCA / velocity-obstacle local avoidance layer | HIGH | Medium |
| 3 | Zenoh / Wi-Fi discovery testing | HIGH | Low-Medium |
| 5 | Decision explainability panel | MEDIUM-HIGH | Low |
| 4 | Spatial partitioning for conflict detection | MEDIUM | Low |
| 6 | Live-demo failure backup plan | MEDIUM | Low |
| 7 | ISO 3691-4 grounding | LOW-MEDIUM | Very low |
| 8 | Energy-aware routing objective | LOW-MEDIUM | Low |
| 9 | CI / regression testing | LOW | Low |
| 10 | ArUco drift correction | LOW | Low (hardware-dependent) |

## If Time Is Short
Do #1, #5, and #3 first — in that order. #1 directly answers the PS's own justification for decentralization (no other team is likely to have this result). #5 is cheap and has high demo impact. #3 is not glamorous but is a demo-breaking risk if skipped.

Everything else is genuinely valuable but optional relative to the PS's core requirements, which the existing docs (00-25) already cover more thoroughly than most teams will.
