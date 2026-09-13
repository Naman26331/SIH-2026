"""Find the shared brain and import it.

The brain lives in `shared/fleetx_core/` at the top of the repository, NOT
inside this ROS 2 package. That is deliberate: it is the same code the laptop
simulator runs, and copying it in here would be the moment the two versions
started drifting apart.

Set FLEETX_BRAIN to point somewhere else if you have moved it.
"""

import os
import sys

_ENV = "FLEETX_BRAIN"


def _candidates():
    if os.environ.get(_ENV):
        yield os.environ[_ENV]
    here = os.path.dirname(os.path.abspath(__file__))
    # installed:  <ws>/install/fleet_agent/lib/python3.x/site-packages/fleet_agent
    # source:     <repo>/ros2_ws/src/fleet_agent/fleet_agent
    probe = here
    for _ in range(10):
        probe = os.path.dirname(probe)
        if not probe or probe == "/":
            break
        yield os.path.join(probe, "shared")


def add_brain_to_path() -> str:
    """Put the brain on sys.path and return where it was found."""
    for path in _candidates():
        if os.path.isdir(os.path.join(path, "fleetx_core")):
            if path not in sys.path:
                sys.path.insert(0, path)
            return path
    raise ImportError(
        "Cannot find the FLEET-X brain (shared/fleetx_core).\n"
        "It should be at the top of the repository, two levels above ros2_ws.\n"
        f"Set {_ENV}=/path/to/repo/shared if you keep it somewhere else."
    )


BRAIN_PATH = add_brain_to_path()

from fleetx_core import (  # noqa: E402  (must follow the sys.path fix)
    BlockedAisle, Cell, ConflictAlert, FleetBus, Grid, Heartbeat, IntentUpdate,
    PathReservation, PoseUpdate, Robot, RobotStatus, TaskAnnounce, TaskBid,
    TaskClaim, WaitReport, YieldRequest, default_grid,
)

__all__ = [
    "BRAIN_PATH", "add_brain_to_path",
    "BlockedAisle", "Cell", "ConflictAlert", "FleetBus", "Grid", "Heartbeat",
    "IntentUpdate", "PathReservation", "PoseUpdate", "Robot", "RobotStatus",
    "TaskAnnounce", "TaskBid", "TaskClaim", "WaitReport", "YieldRequest",
    "default_grid",
]
