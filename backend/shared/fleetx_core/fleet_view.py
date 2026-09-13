"""One robot's private notebook about the other robots.

PURE LOGIC ONLY. No web code, no ROS 2 code.

Every robot keeps its own temporary view of the fleet, and that view must
tolerate stale information.

That last part matters. This notebook is NOT the truth. It is a collection of
things other robots said a moment ago, some of which never arrived. Each robot
has a slightly different, slightly out-of-date picture. That is what makes the
system decentralised rather than one shared brain pretending to be many.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

from .messages import Heartbeat, IntentUpdate, MessageType, PoseUpdate

# If nothing has been heard from a robot for this long, stop trusting it.
# Heartbeats go out twice a second, so this is four missed heartbeats.
DEFAULT_STALE_AFTER = 2.0

# Which way each heading points, for working out where a robot has got to
# since it last spoke.
_HEADING_VECTORS = {"N": (0.0, -1.0), "E": (1.0, 0.0), "S": (0.0, 1.0), "W": (-1.0, 0.0)}


@dataclass
class Neighbour:
    """Everything one robot currently believes about another robot."""

    robot_id: str

    # From the last pose message.
    x: float = 0.0
    y: float = 0.0
    cell: Tuple[int, int] = (0, 0)
    velocity: float = 0.0
    heading: str = "E"

    # From the last heartbeat.
    battery: float = 100.0
    status: str = "UNKNOWN"
    position_known: bool = False

    # From the last intent message -- the useful part.
    destination: Optional[Tuple[int, int]] = None
    planned_nodes: List[Tuple[int, int]] = field(default_factory=list)
    node_etas: List[float] = field(default_factory=list)
    eta_destination: Optional[float] = None
    priority: int = 5

    # Bookkeeping.
    last_heard: float = 0.0        # sim time any message arrived
    last_pose: float = 0.0
    last_intent: float = 0.0
    intent_sent_at: float = 0.0    # when the sender said it, not when it arrived
    highest_seq: int = -1
    missed: int = 0                # messages we can prove went missing
    received: int = 0

    def age(self, now: float) -> float:
        """How many seconds since we last heard anything from this robot."""
        return max(0.0, now - self.last_heard)

    def is_stale(self, now: float, stale_after: float = DEFAULT_STALE_AFTER) -> bool:
        return self.age(now) > stale_after

    def position_at(self, now: float) -> Tuple[float, float]:
        """Where this robot has probably got to by now.

        Positions arrive by radio, so the last one we heard is always a little
        out of date -- typically 0.15s, which at 2.5 squares a second is nearly
        four tenths of a square. That was enough to drive straight through a
        safety check. So carry the last known position forward along its last
        known heading. Sailors call this dead reckoning.
        """
        age = max(0.0, now - self.last_pose)
        dx, dy = _HEADING_VECTORS.get(self.heading, (0.0, 0.0))
        return (self.x + dx * self.velocity * age,
                self.y + dy * self.velocity * age)

    @property
    def next_cell(self) -> Optional[Tuple[int, int]]:
        """The square this robot says it is about to move onto."""
        if not self.planned_nodes:
            return None
        first = self.planned_nodes[0]
        return (first[0], first[1])

    def eta_at(self, cell: Tuple[int, int], now: float) -> Optional[float]:
        """When do we think this robot reaches that square?

        Returned in seconds from NOW, adjusted for how long the message took to
        get here. Returns None if the square is not on its stated plan.

        Phase 4 uses this: if two robots give overlapping times for the same
        square, that is a conflict, and it can be seen before it happens.
        """
        target = (cell[0], cell[1])
        travel = max(0.0, now - self.intent_sent_at)
        for node, eta in zip(self.planned_nodes, self.node_etas):
            if (node[0], node[1]) == target:
                return max(0.0, eta - travel)
        return None

    def to_dict(self, now: float, stale_after: float = DEFAULT_STALE_AFTER) -> Dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "position_known": self.position_known,
            "x": round(self.x, 3), "y": round(self.y, 3),
            "cell": [self.cell[0], self.cell[1]],
            "velocity": round(self.velocity, 2), "heading": self.heading,
            "battery": round(self.battery, 1), "status": self.status,
            "destination": list(self.destination) if self.destination else None,
            "planned_nodes": [[n[0], n[1]] for n in self.planned_nodes],
            "node_etas": [round(e, 2) for e in self.node_etas],
            "priority": self.priority,
            "age": round(self.age(now), 2),
            "stale": self.is_stale(now, stale_after),
            "missed": self.missed,
            "received": self.received,
        }


class FleetView:
    """The notebook. One of these lives inside each robot."""

    def __init__(self, owner_id: str, stale_after: float = DEFAULT_STALE_AFTER):
        self.owner_id = owner_id
        self.stale_after = stale_after
        self.neighbours: Dict[str, Neighbour] = {}

    # ------------------------------------------------------------ listening

    def ingest(self, message: Any, now: float) -> None:
        """File one incoming message into the notebook."""
        sender = getattr(message, "robot_id", None)
        if sender is None or sender == self.owner_id:
            return

        note = self.neighbours.get(sender)
        if note is None:
            note = Neighbour(robot_id=sender)
            self.neighbours[sender] = note

        # Count messages we can prove never arrived. Sequence numbers only ever
        # go up, so a jump means the gap was lost on the way.
        seq = getattr(message, "seq", None)
        if seq is not None:
            if note.highest_seq >= 0 and seq > note.highest_seq + 1:
                note.missed += seq - note.highest_seq - 1
            note.highest_seq = max(note.highest_seq, seq)

        note.received += 1
        note.last_heard = now

        if isinstance(message, Heartbeat):
            note.battery = message.battery
            note.status = message.status

        elif isinstance(message, PoseUpdate):
            # Ignore a message that overtook a newer one on the way here.
            if message.timestamp < note.last_pose:
                return
            note.last_pose = message.timestamp
            note.x, note.y = message.x, message.y
            note.cell = (message.cell[0], message.cell[1])
            note.velocity = message.velocity
            note.heading = message.heading
            note.position_known = True

        elif isinstance(message, IntentUpdate):
            if message.timestamp < note.last_intent:
                return
            note.last_intent = message.timestamp
            note.intent_sent_at = message.timestamp
            note.destination = message.destination
            note.planned_nodes = list(message.planned_nodes)
            note.node_etas = list(message.node_etas)
            note.eta_destination = message.eta_destination
            note.priority = message.priority
            note.status = message.status
            note.x, note.y = message.x, message.y
            note.velocity = message.velocity
            note.position_known = True

    # ------------------------------------------------------------- reading

    def known(self) -> List[Neighbour]:
        return list(self.neighbours.values())

    def fresh(self, now: float) -> List[Neighbour]:
        """Only the robots we still trust."""
        return [n for n in self.neighbours.values() if not n.is_stale(now, self.stale_after)]

    def stale(self, now: float) -> List[Neighbour]:
        """Robots that have gone quiet. The fleet marks these, releases their
        reservations, and avoids their last known path."""
        return [n for n in self.neighbours.values() if n.is_stale(now, self.stale_after)]

    def get(self, robot_id: str) -> Optional[Neighbour]:
        return self.neighbours.get(robot_id)

    def who_is_heading_for(self, cell: Tuple[int, int], now: float) -> List[Tuple[str, float]]:
        """Which robots say they will be on that square, and in how long?

        Sorted soonest first. Only counts robots we still trust.
        This is the raw material for Phase 4's conflict detection.
        """
        out: List[Tuple[str, float]] = []
        for note in self.fresh(now):
            eta = note.eta_at(cell, now)
            if eta is not None:
                out.append((note.robot_id, eta))
        out.sort(key=lambda pair: pair[1])
        return out

    # ---------------------------------------------------------------- output

    def to_dict(self, now: float) -> Dict[str, Any]:
        return {
            "owner": self.owner_id,
            "neighbours": [n.to_dict(now, self.stale_after) for n in self.neighbours.values()],
        }
