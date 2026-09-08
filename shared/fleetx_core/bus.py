"""How messages travel between robots.

PURE LOGIC ONLY. No web code, no ROS 2 code.

Think of FleetBus as a socket in the wall. The robot brain says
"publish this" and "what is in my inbox?" and never asks how the message
actually got there. That is the whole trick that lets one brain run in two
places:

        THE BRAIN (written once)
     publish(msg)  /  poll(robot_id)
                 |
      -----------+-----------
      |                     |
  InMemoryBus          Ros2Bus  (ros2_ws/, written later)
  laptop, today        DDS topics on Ubuntu

InMemoryBus deliberately drops and delays messages, because
04_DECENTRALIZED_FLEET_PROTOCOL section 8 says:

    "The protocol must never assume every message arrives."

Warehouse Wi-Fi bouncing around metal shelving loses packets constantly. If we
only ever test on a perfect connection we will build something that falls over
on demo day.
"""

import random
from abc import ABC, abstractmethod
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Tuple


class FleetBus(ABC):
    """The socket in the wall. Both the simulator and ROS 2 provide one."""

    @abstractmethod
    def register(self, robot_id: str) -> None:
        """Tell the bus a robot exists, so it can be given an inbox."""

    @abstractmethod
    def publish(self, message: Any) -> None:
        """Shout a message to every other robot."""

    @abstractmethod
    def poll(self, robot_id: str) -> List[Any]:
        """Empty one robot's inbox and hand it the messages."""


class InMemoryBus(FleetBus):
    """A bus that runs inside one program. Used by the grid simulator.

    Messages can be dropped (packet_loss) and held up (latency), so the fleet
    can be tested on a bad connection rather than a perfect one.
    """

    def __init__(
        self,
        packet_loss: float = 0.0,      # 0.0 = perfect, 0.3 = 30% of messages vanish
        latency: float = 0.0,          # seconds of delay before delivery
        jitter: float = 0.0,           # random extra delay, 0..jitter seconds
        seed: Optional[int] = None,
    ):
        self.packet_loss = packet_loss
        self.latency = latency
        self.jitter = jitter
        self._rng = random.Random(seed)

        self._inboxes: Dict[str, Deque[Tuple[float, Any]]] = {}
        self._now: float = 0.0

        # Robots whose messages are thrown away before they leave the building.
        # This is the "Silence R2" button -- used to test heartbeat timeouts.
        self.silenced: set = set()

        # Counters for the dashboard.
        self.sent = 0
        self.delivered = 0
        self.dropped = 0
        self.recent: Deque[Any] = deque(maxlen=40)

    # ------------------------------------------------------------ the clock

    def set_time(self, now: float) -> None:
        """The simulator tells the bus what time it is.

        Only the in-memory bus needs this. ROS 2 has a real clock.
        """
        self._now = now

    # -------------------------------------------------------------- the bus

    def register(self, robot_id: str) -> None:
        self._inboxes.setdefault(robot_id, deque())

    def publish(self, message: Any) -> None:
        sender = getattr(message, "robot_id", None)
        self.sent += 1
        self.recent.append(message)

        if sender in self.silenced:
            self.dropped += 1
            return

        for robot_id, inbox in self._inboxes.items():
            if robot_id == sender:
                continue                       # a robot does not talk to itself
            if self.packet_loss and self._rng.random() < self.packet_loss:
                self.dropped += 1
                continue
            delay = self.latency + (self._rng.random() * self.jitter if self.jitter else 0.0)
            inbox.append((self._now + delay, message))

    def poll(self, robot_id: str) -> List[Any]:
        """Hand over every message that has finished travelling."""
        inbox = self._inboxes.get(robot_id)
        if not inbox:
            return []

        ready: List[Any] = []
        keep: Deque[Tuple[float, Any]] = deque()
        for deliver_at, message in inbox:
            if deliver_at <= self._now:
                ready.append(message)
            else:
                keep.append((deliver_at, message))
        self._inboxes[robot_id] = keep
        self.delivered += len(ready)
        return ready

    # ------------------------------------------------------------ dashboard

    def silence(self, robot_id: str, on: bool = True) -> None:
        """Cut a robot's transmitter. It keeps driving, it just stops talking."""
        if on:
            self.silenced.add(robot_id)
        else:
            self.silenced.discard(robot_id)

    def reset_stats(self) -> None:
        self.sent = self.delivered = self.dropped = 0
        self.recent.clear()

    def stats(self) -> Dict[str, Any]:
        return {
            "sent": self.sent,
            "delivered": self.delivered,
            "dropped": self.dropped,
            "packet_loss": round(self.packet_loss, 3),
            "latency_ms": round(self.latency * 1000),
            "silenced": sorted(self.silenced),
        }
