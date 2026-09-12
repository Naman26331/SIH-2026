"""PART 1 and PART 2 -- proving a message really came from inside this fleet.

20_CYBERSECURITY_AND_TRUSTED_DECENTRALIZED_COMMUNICATION asks for two things:
a message cannot be forged, and an old one cannot be replayed to fool a robot
later. Kept deliberately small, so the whole mechanism can be read in one
sitting rather than taken on faith.

How it works
------------
1. Every real fleet member is issued the same key when it joins (FLEET_KEY).
   A real deployment would give each robot its OWN certificate or rotating
   credential -- mutual TLS, a device key, anything of that shape -- and it
   would slot in here without changing anything else, because this file only
   ever asks one question: "does this tag prove the sender holds a fleet
   key?" A shared secret is the smallest thing that demonstrates that
   property honestly, which is what an MVP needs to be able to show.

2. Every message that goes out is SIGNED: a short tag computed from who sent
   it, its sequence number, its timestamp and its type, hashed together with
   the key using HMAC-SHA256. Reading this file tells you exactly how the tag
   is made -- the algorithm is public, on purpose. What is NOT public is the
   key, and without it the tag cannot be reproduced. That is the actual
   security property, not obscurity.

3. Every message that arrives is CHECKED before the robot brain ever reads
   it. No tag, or the wrong tag: dropped, and logged. A sequence number
   already seen, or lower than one already accepted from the same sender: a
   replay, dropped too -- 04_DECENTRALIZED_FLEET_PROTOCOL and doc 20 both
   call this out directly: reject previously processed message IDs and old
   timestamps.

Nothing here touches steering, safety or the reservation table. Authentication
only decides whether a message is read at all; everything after that is
exactly the brain that existed before this file.

Pure Python. No web, no ROS 2.
"""

import hashlib
import hmac
import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

# Issued to every real fleet member. Swap for per-robot certificates or a
# rotating credential store on real hardware; nothing else in this file
# changes, because it only ever asks "does this tag match this key?"
FLEET_KEY = b"fleetx-2026-demo-key"

TAG_LEN = 16     # hex characters. Short on purpose: this is a tamper check,
                 # not a confidentiality mechanism, and every message on the
                 # bus carries one, twenty times a second per robot.

# A message older than this by the time it is checked is stale, not late --
# rejecting it is replay protection, not intolerance of a slow network. Chosen
# well above any latency or jitter this project ever tests with (at most a
# second or so), so a message that is genuinely just delayed is never mistaken
# for an attack.
MAX_MESSAGE_AGE = 20.0


def _identity(message: Any) -> tuple:
    """Who, when and what kind -- used by replay protection, which only ever
    asks about ORDERING and FRESHNESS, never about content."""
    sender = getattr(message, "robot_id", "") or ""
    seq = getattr(message, "seq", 0) or 0
    timestamp = getattr(message, "timestamp", 0.0) or 0.0
    kind = getattr(message, "type", None) or type(message).__name__
    return sender, seq, timestamp, kind


def _canonical(message: Any) -> bytes:
    """Every field of the message, in a fixed order, as bytes.

    The tag has to cover the WHOLE message, not just who sent it and when --
    otherwise the sender's identity is proven but the content is not, and a
    tampered battery reading or a rewritten status would sail straight
    through with a perfectly valid signature attached. Every message class
    already has a to_dict(); reusing it means there is exactly one place that
    defines "everything in this message", and signing and displaying it can
    never quietly drift apart.
    """
    if hasattr(message, "to_dict"):
        data = message.to_dict()
    else:
        sender, seq, timestamp, kind = _identity(message)
        data = {"robot_id": sender, "seq": seq, "timestamp": timestamp, "type": kind}
    return json.dumps(data, sort_keys=True, default=str).encode("utf-8")


def _digest(message: Any) -> str:
    """The tag. Same message, same key, always the same output -- which is
    exactly what lets a receiver recompute it and compare. Change ANY field,
    even by a fraction, and every bit of the output changes with it."""
    return hmac.new(FLEET_KEY, _canonical(message), hashlib.sha256).hexdigest()[:TAG_LEN]


def seal(message: Any) -> Any:
    """Sign a message at the moment it is sent. Returns the same object,
    carrying a `.tag` -- the proof that whoever sent it held the fleet key,
    covering everything the message says, not just who is saying it.
    """
    message.tag = _digest(message)
    return message


@dataclass
class Verdict:
    """Why a message was accepted or rejected. Printable, on purpose -- the
    event feed says exactly this, not just "rejected"."""

    ok: bool
    reason: str = ""

    def __bool__(self) -> bool:
        return self.ok


def check_signature(message: Any) -> Verdict:
    """Does this message carry a tag proving both who sent it AND that
    nothing in it has changed since?"""
    tag = getattr(message, "tag", None)
    if not tag:
        return Verdict(False, "no signature")
    expected = _digest(message)
    if not hmac.compare_digest(expected, tag):
        return Verdict(False, "bad signature")
    return Verdict(True)


class ReplayGuard:
    """One robot's newest accepted sequence per sender and message stream.

    Kept on the ROBOT, not the bus -- a real robot has no shared memory with
    anyone else. Streams matter because DDS only guarantees ordering within a
    topic; pose and reservation callbacks may legitimately interleave.
    """

    def __init__(self, max_age: float = MAX_MESSAGE_AGE) -> None:
        self.max_age = max_age
        # DDS preserves writer order within one topic, not across independent
        # topics. Track each signed message stream separately so a newer pose
        # cannot make an earlier reservation look like a replay.
        self._last_seq: Dict[Tuple[str, str], int] = {}

    def check(self, sender: str, seq: int, timestamp: float, now: float,
              stream: str = "") -> Verdict:
        if now - timestamp > self.max_age:
            return Verdict(False, f"stale ({now - timestamp:.1f}s old)")
        key = (sender, stream)
        last = self._last_seq.get(key)
        if last is not None and seq <= last:
            return Verdict(False, f"replay (seq {seq}, already saw {last})")
        self._last_seq[key] = seq
        return Verdict(True)

    def forget(self, sender: str) -> None:
        """A robot left or failed -- if it (or its name) comes back, its
        sequence numbers start over, and that must not look like a replay."""
        for key in [key for key in self._last_seq if key[0] == sender]:
            del self._last_seq[key]
