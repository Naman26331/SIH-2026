"""Write every message that crosses the fleet bus to a CSV file.

This is simulator scaffolding, not robot brain -- it lives here, not in
shared/fleetx_core/, on purpose: fleetx_core stays pure logic with no file
I/O, and the ROS 2 side has its own, completely different, logging story
(real DDS topics, rosbag) that must not be tied to a CSV writer.

One row per message, in the order the bus saw it:

    seq, sim_time, wall_time, type, sender, target, summary, details

Common columns first so the file reads as a single sheet -- open it in
Excel, or `column -s, -t | less` in a terminal -- rather than the wide,
mostly-empty grid you get from unioning eleven different message schemas
into one row shape. `summary` is a plain-English line in the same voice as
the rest of the event feed, so the file is legible on its own; `details` is
the message's full `to_dict()` as JSON, for anything that needs the exact
fields back.
"""

import csv
import json
import os
import threading
import time
from typing import Any, List, Optional, TextIO

COLUMNS = ["seq", "sim_time", "wall_time", "type", "sender", "target", "summary", "details"]


def _target_of(message: Any) -> str:
    """Whoever this message is ABOUT, when that is someone other than the
    sender -- a YieldRequest's target, a DistressSignal's failed robot, a
    TaskClaim spoken on a quiet peer's behalf. Blank when it is just the
    sender talking about itself."""
    target = getattr(message, "target", None)
    if target:
        return str(target)
    failed = getattr(message, "failed_robot_id", None)
    if failed:
        return str(failed)
    about, speaker = getattr(message, "robot_id", None), getattr(message, "sender", None)
    if speaker and about and speaker != about:
        return str(about)
    return ""


def _summarise(message: Any) -> str:
    """One plain-English line per message, so the CSV reads like a
    transcript of the fleet talking, not a data dump."""
    kind = getattr(message, "type", type(message).__name__)
    who = getattr(message, "sender", None) or getattr(message, "robot_id", "?")

    if kind == "HEARTBEAT":
        return f"{who} is alive - battery {message.battery:.0f}%, {message.status}"
    if kind == "POSE_UPDATE":
        return f"{who} at ({message.cell[0]}, {message.cell[1]}), heading {message.heading}"
    if kind == "INTENT_UPDATE":
        dest = tuple(message.destination) if message.destination else None
        return (f"{who} heading to {dest}, "
                f"{len(message.planned_nodes)} square(s) planned")
    if kind == "CONFLICT_ALERT":
        return (f"{who} flags {message.kind} between {message.robot_a} and "
                f"{message.robot_b} at {tuple(message.resource)}")
    if kind == "PATH_RESERVATION":
        return f"{who} {message.action.lower()}s {message.kind} {message.cells}"
    if kind == "WAIT_REPORT":
        blocker = message.blocked_by or "nobody"
        return f"{who} stuck behind {blocker} for {message.waiting:.1f}s"
    if kind == "YIELD_REQUEST":
        return (f"{who} -> {message.target}: {message.action} "
                f"({message.reason}) for {tuple(message.resource)}")
    if kind == "BLOCKED_AISLE":
        state = "cleared" if message.cleared else "blocked"
        return f"{who} reports {tuple(message.cell)} {state}"
    if kind == "TASK_ANNOUNCE":
        return f"{who} announces {message.task_id} ({message.product or 'job'})"
    if kind == "TASK_BID":
        return f"{who} bids {message.cost:.1f} on {message.task_id}"
    if kind == "TASK_CLAIM":
        return f"{who} {message.action.lower()}s {message.task_id}"
    if kind == "DISTRESS_SIGNAL":
        return f"{who} reports {message.failed_robot_id} down at {tuple(message.cell)} - send the medic"
    return f"{who} sent {kind}"


class CommsLogger:
    """Buffered CSV writer for every message published on the fleet bus.

    Register with `InMemoryBus(on_publish=logger.log)`, or set
    `bus.on_publish = logger.log` after the fact. Rows are batched in memory
    and flushed together every FLUSH_EVERY messages (and on close()/flush()),
    so a busy fleet -- easily 100+ messages a second with a full floor of
    robots -- does not turn into a filesystem write per message.
    """

    FLUSH_EVERY = 50

    def __init__(self, path: str, world: Optional[Any] = None):
        self._world = world
        self._seq = 0
        self._buffer: List[List[Any]] = []
        self._lock = threading.Lock()

        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        # Fresh file per run: a run's message log describing a different
        # sim_time timeline than the last run it would silently continue
        # from is more confusing merged into one file than kept separate.
        self._file: TextIO = open(path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(COLUMNS)
        self._file.flush()

    def log(self, message: Any) -> None:
        with self._lock:
            self._seq += 1
            sim_time = round(self._world.sim_time, 2) if self._world is not None else ""
            details = message.to_dict() if hasattr(message, "to_dict") else {}
            self._buffer.append([
                self._seq,
                sim_time,
                round(time.time(), 3),
                getattr(message, "type", type(message).__name__),
                getattr(message, "robot_id", ""),
                _target_of(message),
                _summarise(message),
                json.dumps(details, separators=(",", ":")),
            ])
            if len(self._buffer) >= self.FLUSH_EVERY:
                self._flush_locked()

    def flush(self) -> None:
        with self._lock:
            self._flush_locked()

    def _flush_locked(self) -> None:
        if not self._buffer:
            return
        self._writer.writerows(self._buffer)
        self._buffer.clear()
        self._file.flush()

    def close(self) -> None:
        self.flush()
        self._file.close()
