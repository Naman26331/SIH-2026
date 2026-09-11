#!/usr/bin/env python3
"""Guard the rule: shared/fleetx_core/ must stay pure logic.

No web code. No ROS 2 code. Ever.

That rule is the whole reason the grid simulator and the ROS 2 robot can share
one brain. It is easy to break by accident, so this checks it for you.

    python3 tools/check_purity.py

Exits 0 if the brain is clean, 1 if something has crept in.
"""

import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BRAIN = os.path.join(ROOT, "shared", "fleetx_core")

# Anything the brain is allowed to import from outside itself.
# Standard library only, and only the boring parts.
ALLOWED = {
    "dataclasses", "enum", "typing", "heapq", "math", "time",
    "collections", "itertools", "abc", "json", "random", "statistics",
    # Phase 23: message signing (shared/fleetx_core/security.py). Both are
    # pure-Python standard library, run identically on the laptop simulator
    # and on the ROS 2 side, and touch neither a network nor a framework --
    # exactly what "harmless" means in this list.
    "hashlib", "hmac",
}

# Things that must never appear. Importing any of these means the brain has
# been tied to one machine or one framework.
BANNED_HINTS = (
    "rclpy", "rospy", "ros2", "geometry_msgs", "nav_msgs", "std_msgs",
    "sensor_msgs", "tf2", "gazebo", "nav2",
    "http", "socket", "flask", "fastapi", "django", "uvicorn",
    "websocket", "websockets", "requests", "urllib", "asyncio", "aiohttp",
)


def top_level(name: str) -> str:
    return name.split(".")[0]


def check_file(path: str):
    """Return a list of plain-English problems found in one file."""
    problems = []
    with open(path, "r", encoding="utf-8") as fh:
        source = fh.read()

    tree = ast.parse(source, filename=path)
    rel = os.path.relpath(path, ROOT)

    for node in ast.walk(tree):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level > 0:
                continue          # "from .grid import ..." -- its own package, fine
            names = [node.module or ""]
        else:
            continue

        for name in names:
            root_name = top_level(name)
            low = name.lower()
            if any(bad in low for bad in BANNED_HINTS):
                problems.append(
                    f"{rel}:{node.lineno}  imports '{name}'. "
                    f"That is web or ROS 2 code and must live in a wrapper, not in the brain."
                )
            elif root_name not in ALLOWED:
                problems.append(
                    f"{rel}:{node.lineno}  imports '{name}', which is not on the allowed list. "
                    f"If it is a harmless standard-library module, add it to ALLOWED in this file."
                )
    return problems


def main() -> int:
    if not os.path.isdir(BRAIN):
        print(f"Cannot find the brain folder: {BRAIN}")
        return 1

    files = sorted(
        os.path.join(BRAIN, f) for f in os.listdir(BRAIN) if f.endswith(".py")
    )
    all_problems = []
    for path in files:
        all_problems.extend(check_file(path))

    print(f"Checked {len(files)} files in shared/fleetx_core/")
    if all_problems:
        print("\nPROBLEM - the shared brain is no longer pure:\n")
        for p in all_problems:
            print("  x " + p)
        print("\nFix these before going further, or the ROS 2 version will not be able")
        print("to reuse the brain and it will have to be written twice.")
        return 1

    print("Clean. No web code and no ROS 2 code in the shared brain.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
