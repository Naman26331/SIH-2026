#!/usr/bin/env python3
"""The one table. Everything measured on the same code, in one run.

    python3 -u tools/final_table.py

Every number a judge might be shown comes from here, so that nothing on a slide
can have been measured on a different version of the code.

The metric
----------
Both fleets are given the SAME orders for the SAME length of time, and we count
what actually got delivered. "% faster" is then the drop in average seconds per
delivered order:

    seconds per order = window / orders delivered
    % faster = (stop_and_wait_spo - fleetx_spo) / stop_and_wait_spo * 100

Why this one and not "time to finish the batch": under load stop-and-wait never
finishes, so there would be no number at all. Why not "average time per order"
as the fleets themselves report it: a jammed fleet delivers the easy early
orders while the floor is empty, then seizes up, so its own average covers only
what it survived and it can look FASTER than a fleet doing six times the work.
Counting output over a fixed clock has neither hole in it, and no run has to be
thrown away.

Fairness: identical map, start positions, speeds, orders, A*, job auction and
local safety reflex. The ONLY difference is the coordination logic.
"""

import argparse
import csv
import os
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "shared"))
sys.path.insert(0, os.path.join(ROOT, "simulator"))

from fleetx_core import fleet_world                    # noqa: E402
from scenarios import Scenarios                        # noqa: E402

DT = 0.05


def run(mode, robots, seed, every, window):
    world = fleet_world(robots)
    world.coordination = mode
    if mode == "STOP_AND_WAIT":
        world.reservations_enabled = False
        world.negotiation_enabled = False
        world.deadlock_enabled = False
    sc = Scenarios(world)
    sc.apply("orders", seed=seed, every=every, limit=None)

    worst_gap, biggest_jump = 99.0, 0.0
    while world.sim_time < window:
        before = {r.robot_id: (r.x, r.y) for r in world.robots.values()}
        sc.keep_busy()
        world.tick(DT)
        rs = list(world.robots.values())
        for r in rs:
            px, py = before[r.robot_id]
            biggest_jump = max(biggest_jump, ((r.x - px) ** 2 + (r.y - py) ** 2) ** 0.5)
        for i in range(len(rs)):
            for j in range(i + 1, len(rs)):
                a, b = rs[i], rs[j]
                worst_gap = min(worst_gap, ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5)

    k = world.kpis()
    return {
        "mode": mode, "robots": robots, "seed": seed,
        "delivered": k["task_done"], "collisions": k["collisions"],
        "deadlocked": k["deadlocked"], "distance": k["total_distance"],
        "closest_gap": round(worst_gap, 3), "biggest_jump": round(biggest_jump, 4),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--robots", type=int, nargs="+", default=[3, 5, 10, 15, 20])
    ap.add_argument("--every", type=float, default=4.0)
    ap.add_argument("--window", type=float, default=400.0)
    ap.add_argument("--csv", default=os.path.join(ROOT, "final_table.csv"))
    args = ap.parse_args()
    seeds = list(range(1, args.seeds + 1))

    print()
    print("  FLEET-X FINAL NUMBERS")
    print("  " + "=" * 78)
    print(f"  {args.window:.0f}s of orders (one every {args.every:g}s), "
          f"{args.seeds} seeds per fleet size, identical work for both fleets")
    print(f"  started {time.strftime('%H:%M:%S')}")
    print()

    rows, summary = [], []
    for robots in args.robots:
        t0 = time.time()
        print(f"  {robots} robots ...", end=" ", flush=True)
        base = [run("STOP_AND_WAIT", robots, s, args.every, args.window) for s in seeds]
        print("baseline done ...", end=" ", flush=True)
        fx = [run("FLEETX", robots, s, args.every, args.window) for s in seeds]
        rows += base + fx

        mb = statistics.mean(r["delivered"] for r in base)
        mf = statistics.mean(r["delivered"] for r in fx)
        spo_b = args.window / mb if mb else float("inf")
        spo_f = args.window / mf if mf else float("inf")
        pct = (spo_b - spo_f) / spo_b * 100.0 if mb else 0.0
        summary.append({
            "robots": robots,
            "base_delivered": mb, "fx_delivered": mf,
            "base_collisions": sum(r["collisions"] for r in base),
            "fx_collisions": sum(r["collisions"] for r in fx),
            "spo_b": spo_b, "spo_f": spo_f, "pct": pct,
            "fx_deadlocked": statistics.mean(r["deadlocked"] for r in fx),
            "gap": min(r["closest_gap"] for r in fx),
            "jump": max(r["biggest_jump"] for r in fx),
        })
        print(f"FLEET-X done  ({time.time() - t0:.0f}s)   "
              f"-> {mb:.1f} vs {mf:.1f} orders, {pct:.1f}% faster, "
              f"{sum(r['collisions'] for r in fx)} collisions")

    print()
    print("  " + "=" * 78)
    print("  THE TABLE")
    print("  " + "-" * 78)
    print(f"  {'fleet':>6} | {'collisions':^12} | {'orders delivered':^21} | {'% faster':>9}")
    print(f"  {'size':>6} | {'S&W':>5} {'FLEET-X':>6} | {'S&W':>8} {'FLEET-X':>10} | {'':>9}")
    print("  " + "-" * 78)
    for s in summary:
        print(f"  {s['robots']:>6} | {s['base_collisions']:>5} {s['fx_collisions']:>6} | "
              f"{s['base_delivered']:>8.1f} {s['fx_delivered']:>10.1f} | {s['pct']:>8.1f}%")
    print("  " + "-" * 78)

    total_fx_coll = sum(s["fx_collisions"] for s in summary)
    total_b_coll = sum(s["base_collisions"] for s in summary)
    best = max(summary, key=lambda s: s["fx_delivered"])
    print()
    print(f"  Collisions across every run:   FLEET-X {total_fx_coll},   "
          f"stop-and-wait {total_b_coll}")
    print(f"  Closest two robots ever came:  "
          f"{min(s['gap'] for s in summary):.3f} squares  (touching is under 0.7)")
    print(f"  Biggest single-tick move:      "
          f"{max(s['jump'] for s in summary):.4f} squares  (wheels allow 0.125)")
    print(f"  Best throughput:               {best['fx_delivered']:.1f} orders "
          f"at {best['robots']} robots")
    print()
    print("  Supporting detail (FLEET-X):")
    for s in summary:
        print(f"    {s['robots']:>2} robots: {s['spo_f']:5.1f}s per order "
              f"(stop-and-wait {s['spo_b']:6.1f}s), "
              f"{s['fx_deadlocked']:.1f} robots jammed at the end on average")

    with open(args.csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print()
    print(f"  Every individual run: {os.path.relpath(args.csv, ROOT)}")
    print(f"  finished {time.strftime('%H:%M:%S')}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
