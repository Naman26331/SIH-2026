"""FLEET-X shared brain.

PURE LOGIC ONLY -- this package must never import a web library or a ROS 2
library. It is shared, unchanged, by:

  Part 1  grid_sim/   the browser simulator that runs on a laptop
  Part 2  ros2_ws/    the ROS 2 agent that runs on Ubuntu / a real robot

Keeping it clean is what stops the brain from being written twice.
"""

from .astar import find_path, manhattan, uniform_cost
from .bus import FleetBus, InMemoryBus
from .conflicts import (DEFAULT_CLEARANCE, DEFAULT_HORIZON, Conflict, ConflictKind,
                        Plan, Window, build_plan, find_conflicts)
from .deadlock import (STUCK_SECONDS, WaitForGraph, Waiting, choose_victim)
from .fleet_view import DEFAULT_STALE_AFTER, FleetView, Neighbour
from .messages import (BlockedAisle, ConflictAlert, Heartbeat, IntentUpdate,
                       MessageType, PathReservation, PoseUpdate, TaskAnnounce,
                       TaskBid, TaskClaim, WaitReport, YieldRequest, from_dict)
from .grid import Cell, CellKind, Grid, default_grid
from .obstacles import (DEFAULT_TTL, SENSOR_RANGE, Block, BlockedMap,
                        within_range)
from .priority import (BASE_PRIORITY, DOMINANCE_MARGIN, as_points,
                       effective_priority, next_wait_credit, quantise,
                       yields_to)
from .reservations import (DEFAULT_LOOKAHEAD, OCCUPANCY_PRIORITY, Reservation,
                           ReservationTable, avoidance_cost, edge_key, node_key)
from .robot import Robot, RobotStatus
from .tasks import (BID_WINDOW, Task, TaskBoard, TaskStatus, auction_winner,
                    bid_cost, bid_rank)
from .world import (COLLISION_DISTANCE, World, fleet_world, phase1_world,
                    phase2_world)

__all__ = [
    "Cell", "CellKind", "Grid", "default_grid",
    "find_path", "manhattan", "uniform_cost",
    "Robot", "RobotStatus",
    "FleetBus", "InMemoryBus",
    "FleetView", "Neighbour", "DEFAULT_STALE_AFTER",
    "Heartbeat", "PoseUpdate", "IntentUpdate", "ConflictAlert", "PathReservation",
    "MessageType", "from_dict",
    "Reservation", "ReservationTable", "node_key", "edge_key", "DEFAULT_LOOKAHEAD",
    "OCCUPANCY_PRIORITY", "avoidance_cost",
    "effective_priority", "next_wait_credit", "as_points", "BASE_PRIORITY",
    "yields_to", "quantise", "DOMINANCE_MARGIN",
    "Conflict", "ConflictKind", "Plan", "Window", "build_plan", "find_conflicts",
    "WaitForGraph", "Waiting", "choose_victim", "STUCK_SECONDS",
    "WaitReport", "YieldRequest", "BlockedAisle",
    "TaskAnnounce", "TaskBid", "TaskClaim",
    "Task", "TaskBoard", "TaskStatus", "auction_winner", "bid_cost",
    "bid_rank", "BID_WINDOW",
    "BlockedMap", "Block", "within_range", "SENSOR_RANGE", "DEFAULT_TTL",
    "DEFAULT_HORIZON", "DEFAULT_CLEARANCE",
    "World", "phase1_world", "phase2_world", "fleet_world", "COLLISION_DISTANCE",
]
