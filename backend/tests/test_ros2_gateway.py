"""ROS-free contract tests for live DDS telemetry projection."""

import os
import sys

BACKEND = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BACKEND, "shared"))
sys.path.insert(0, os.path.join(
    BACKEND, "ros2_ws", "src", "fleet_agent"))

from fleet_agent.telemetry import TelemetryProjector
from fleetx_core import (PathReservation, PoseUpdate, TaskAnnounce, TaskClaim,
                         TaskStatus, security)


def signed(message):
    return security.seal(message)


def test_signed_pose_reaches_dashboard_and_replay_is_rejected():
    projector = TelemetryProjector()
    stamp = projector.timestamp()
    pose = signed(PoseUpdate(
        robot_id="R1", timestamp=stamp, seq=1,
        x=2.25, y=8.0, cell=(2, 8), velocity=0.4, heading="E"))

    assert projector.ingest(pose)
    assert not projector.ingest(pose)

    state = projector.snapshot("R1")
    assert state["mode"] == "ros2"
    assert state["robots"][0]["cell"] == [2, 8]
    assert state["robots"][0]["velocity"] == 0.4
    assert state["bus"]["transport"] == "ROS 2 DDS"
    assert state["bus"]["rejected"] == 1


def test_reservation_and_task_events_use_current_core_models():
    projector = TelemetryProjector()
    stamp = projector.timestamp()
    projector.ingest(signed(PoseUpdate(
        robot_id="R1", timestamp=stamp, seq=1,
        x=2.0, y=8.0, cell=(2, 8), velocity=0.0)))
    projector.ingest(signed(PathReservation(
        robot_id="R1", timestamp=stamp + 0.1, seq=2, action="CLAIM", kind="NODE",
        cells=[(3, 8)], start=stamp, end=stamp + 30.0, priority=8)))
    projector.ingest(signed(TaskAnnounce(
        robot_id="ORDERS", timestamp=stamp, seq=1, task_id="T-1",
        pickup=(3, 8), dropoff=(5, 8), product="Mouse", shelf="A1",
        quantity=2, priority=7)))
    projector.ingest(signed(TaskClaim(
        robot_id="R1", timestamp=stamp + 0.2, seq=3, task_id="T-1",
        action="CLAIM", cost=4.0)))

    state = projector.snapshot("R1")
    assert state["views"]["R1"]["table"][0]["kind"] == "NODE"
    assert state["tasks"][0]["status"] == TaskStatus.ASSIGNED.value
    assert state["tasks"][0]["assigned_robot"] == "R1"
    assert state["tasks"][0]["line"] == "Mouse x2 from shelf A1"


def test_unsigned_message_never_changes_live_state():
    projector = TelemetryProjector()
    message = PoseUpdate(
        robot_id="R9", timestamp=projector.timestamp(), seq=1,
        x=0.0, y=0.0, cell=(0, 0), velocity=0.0)

    assert not projector.ingest(message)
    assert projector.snapshot()["robots"] == []
