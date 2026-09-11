"""Turning brain messages into ROS 2 messages, and back.

Every function here is a straight field-for-field copy. There is no logic in
this file on purpose: if a decision were made here it would exist in the ROS 2
build and not in the simulator, and the two would start to disagree.

Grid squares and metres
-----------------------
The brain thinks in whole squares: (0,0) is the top-left of the warehouse and y
increases DOWNWARD, like reading a page. Gazebo and Nav2 think in metres with y
increasing NORTH. So:

    world_x =  grid_x * RESOLUTION
    world_y = -grid_y * RESOLUTION

RESOLUTION is metres per square (1.0 by default -- a one-metre grid). Change it
in one place, here, and everything follows.
"""

from typing import List, Optional, Sequence, Tuple

from builtin_interfaces.msg import Time as RosTime

from fleet_msgs.msg import (BatteryStatus, BlockedAisle as BlockedAisleMsg,
                            ConflictAlert as ConflictAlertMsg,
                            PathReservation as PathReservationMsg, RobotIntent,
                            RobotState, Task as TaskMsg, TaskBid as TaskBidMsg,
                            TaskClaim as TaskClaimMsg,
                            WaitReport as WaitReportMsg,
                            YieldRequest as YieldRequestMsg)

from .brain import (BlockedAisle, Cell, ConflictAlert, Heartbeat, IntentUpdate,
                    PathReservation, PoseUpdate, TaskAnnounce, TaskBid,
                    TaskClaim, WaitReport, YieldRequest)

RESOLUTION = 1.0        # metres per grid square


# --------------------------------------------------------------- geometry

def cell_to_world(cell) -> Tuple[float, float]:
    """Grid square -> metres in Gazebo/Nav2."""
    x = cell[0] if isinstance(cell, (tuple, list)) else cell.x
    y = cell[1] if isinstance(cell, (tuple, list)) else cell.y
    return (x * RESOLUTION, -y * RESOLUTION)


def world_to_cell(x: float, y: float) -> Cell:
    """Metres -> the grid square a robot is standing in."""
    return Cell(int(round(x / RESOLUTION)), int(round(-y / RESOLUTION)))


def world_to_grid_xy(x: float, y: float) -> Tuple[float, float]:
    """Metres -> fractional grid position, for smooth tracking."""
    return (x / RESOLUTION, -y / RESOLUTION)


# ------------------------------------------------------------------ time

def to_ros_time(seconds: float) -> RosTime:
    sec = int(seconds)
    return RosTime(sec=sec, nanosec=int((seconds - sec) * 1e9))


def from_ros_time(stamp: RosTime) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ----------------------------------------------------------------- pairs

def flatten(cells: Sequence) -> List[int]:
    """[(1,2), (3,4)] -> [1, 2, 3, 4]. ROS 2 has no array-of-pairs type."""
    out: List[int] = []
    for c in cells:
        x = c[0] if isinstance(c, (tuple, list)) else c.x
        y = c[1] if isinstance(c, (tuple, list)) else c.y
        out.extend((int(x), int(y)))
    return out


def unflatten(values: Sequence[int]) -> List[Tuple[int, int]]:
    return [(int(values[i]), int(values[i + 1])) for i in range(0, len(values) - 1, 2)]


# ------------------------------------------------------- brain -> ROS 2

def pose_to_ros(m: PoseUpdate) -> RobotState:
    out = RobotState()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.x, out.y = float(m.x), float(m.y)
    out.cell = [int(m.cell[0]), int(m.cell[1])]
    out.velocity = float(m.velocity)
    out.heading = m.heading
    return out


def intent_to_ros(m: IntentUpdate) -> RobotIntent:
    out = RobotIntent()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.x, out.y = float(m.x), float(m.y)
    out.velocity = float(m.velocity)
    out.has_destination = m.destination is not None
    out.destination = [int(m.destination[0]), int(m.destination[1])] if m.destination else [0, 0]
    out.planned_nodes = flatten(m.planned_nodes)
    out.node_etas = [float(e) for e in m.node_etas]
    out.has_eta_destination = m.eta_destination is not None
    out.eta_destination = float(m.eta_destination or 0.0)
    out.priority = int(m.priority)
    out.status = m.status
    return out


def heartbeat_to_ros(m: Heartbeat) -> BatteryStatus:
    out = BatteryStatus()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.battery = float(m.battery)
    out.status = m.status
    out.health = m.health
    return out


def conflict_to_ros(m: ConflictAlert) -> ConflictAlertMsg:
    out = ConflictAlertMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.robot_a, out.robot_b = m.robot_a, m.robot_b
    out.resource = [int(m.resource[0]), int(m.resource[1])]
    out.kind = m.kind
    out.estimated_time = float(m.estimated_time)
    return out


def reservation_to_ros(m: PathReservation) -> PathReservationMsg:
    out = PathReservationMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.action, out.kind = m.action, m.kind
    out.cells = flatten(m.cells)
    out.start, out.end = float(m.start), float(m.end)
    out.priority = int(m.priority)
    return out


def blocked_to_ros(m: BlockedAisle) -> BlockedAisleMsg:
    out = BlockedAisleMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.cell = [int(m.cell[0]), int(m.cell[1])]
    out.confidence = float(m.confidence)
    out.ttl = float(m.ttl)
    out.cleared = bool(m.cleared)
    return out


def task_to_ros(m: TaskAnnounce) -> TaskMsg:
    out = TaskMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.task_id = m.task_id
    out.pickup = [int(m.pickup[0]), int(m.pickup[1])]
    out.dropoff = [int(m.dropoff[0]), int(m.dropoff[1])]
    out.product = m.product
    out.shelf = m.shelf
    out.quantity = int(m.quantity)
    out.priority = int(m.priority)
    out.flexible = bool(m.flexible)
    return out


def bid_to_ros(m: TaskBid) -> TaskBidMsg:
    out = TaskBidMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.task_id = m.task_id
    out.cost = float(m.cost)
    return out


def claim_to_ros(m: TaskClaim) -> TaskClaimMsg:
    out = TaskClaimMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.task_id = m.task_id
    out.action = m.action
    out.cost = float(m.cost)
    return out


def wait_to_ros(m: WaitReport) -> WaitReportMsg:
    out = WaitReportMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.is_blocked = m.blocked_by is not None
    out.blocked_by = m.blocked_by or ""
    out.waiting = float(m.waiting)
    out.priority = int(m.priority)
    out.is_moving = bool(m.is_moving)
    return out


def yield_to_ros(m: YieldRequest) -> YieldRequestMsg:
    out = YieldRequestMsg()
    out.robot_id = m.robot_id
    out.stamp = to_ros_time(m.timestamp)
    out.seq = m.seq
    out.target = m.target
    out.resource = [int(m.resource[0]), int(m.resource[1])]
    out.reason = m.reason
    return out


# ------------------------------------------------------- ROS 2 -> brain

def ros_to_pose(m: RobotState) -> PoseUpdate:
    return PoseUpdate(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        x=m.x, y=m.y, cell=(int(m.cell[0]), int(m.cell[1])),
        velocity=m.velocity, heading=m.heading)


def ros_to_intent(m: RobotIntent) -> IntentUpdate:
    return IntentUpdate(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        x=m.x, y=m.y, velocity=m.velocity,
        destination=(int(m.destination[0]), int(m.destination[1]))
        if m.has_destination else None,
        planned_nodes=unflatten(list(m.planned_nodes)),
        node_etas=list(m.node_etas),
        eta_destination=m.eta_destination if m.has_eta_destination else None,
        priority=m.priority, status=m.status)


def ros_to_heartbeat(m: BatteryStatus) -> Heartbeat:
    return Heartbeat(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        battery=m.battery, status=m.status, health=m.health)


def ros_to_conflict(m: ConflictAlertMsg) -> ConflictAlert:
    return ConflictAlert(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        robot_a=m.robot_a, robot_b=m.robot_b,
        resource=(int(m.resource[0]), int(m.resource[1])),
        kind=m.kind, estimated_time=m.estimated_time)


def ros_to_reservation(m: PathReservationMsg) -> PathReservation:
    return PathReservation(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        action=m.action, kind=m.kind, cells=unflatten(list(m.cells)),
        start=m.start, end=m.end, priority=m.priority)


def ros_to_blocked(m: BlockedAisleMsg) -> BlockedAisle:
    return BlockedAisle(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        cell=(int(m.cell[0]), int(m.cell[1])), confidence=m.confidence,
        ttl=m.ttl, cleared=m.cleared)


def ros_to_task(m: TaskMsg) -> TaskAnnounce:
    return TaskAnnounce(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        task_id=m.task_id,
        pickup=(int(m.pickup[0]), int(m.pickup[1])),
        dropoff=(int(m.dropoff[0]), int(m.dropoff[1])),
        product=m.product, shelf=m.shelf, quantity=int(m.quantity),
        priority=m.priority, flexible=m.flexible)


def ros_to_bid(m: TaskBidMsg) -> TaskBid:
    return TaskBid(robot_id=m.robot_id, timestamp=from_ros_time(m.stamp),
                   seq=m.seq, task_id=m.task_id, cost=m.cost)


def ros_to_claim(m: TaskClaimMsg) -> TaskClaim:
    return TaskClaim(robot_id=m.robot_id, timestamp=from_ros_time(m.stamp),
                     seq=m.seq, task_id=m.task_id, action=m.action, cost=m.cost)


def ros_to_wait(m: WaitReportMsg) -> WaitReport:
    return WaitReport(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        blocked_by=m.blocked_by if m.is_blocked else None,
        waiting=m.waiting, priority=m.priority, is_moving=m.is_moving)


def ros_to_yield(m: YieldRequestMsg) -> YieldRequest:
    return YieldRequest(
        robot_id=m.robot_id, timestamp=from_ros_time(m.stamp), seq=m.seq,
        target=m.target, resource=(int(m.resource[0]), int(m.resource[1])),
        reason=m.reason)
