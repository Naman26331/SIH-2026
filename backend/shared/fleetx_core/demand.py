"""PART 1 and PART 2 -- learning which parts of the warehouse get busy.

The fleet predicts demand. The useful version of that is not a neural
network: it is noticing that aisle C has had most of the recent orders and that the far corner has had none, and standing somewhere
sensible before the next order arrives.

So this counts. It keeps, for each aisle and each slot of the hour, how many
orders have come from there, with older counts fading out. Predicting is then
reading the table. Every number in it can be printed, and every decision it
leads to can be explained in one sentence -- which matters more than accuracy
here, because a judge can check an explanation and cannot check a weight.

Two things make it safe to be wrong:
  * only a robot with nothing else to do ever acts on it, and
  * it is a suggestion about where to WAIT, never about where to drive through
    or who gives way. A wrong guess costs a short empty drive and nothing else.

Each robot builds its own copy from the order announcements it hears, so there
is no server holding the model -- the same way every robot builds its own copy
of the job board.

Pure Python. No web, no ROS 2.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# How long a slot of the timetable is, in seconds, and how many of them make a
# cycle. Ten one-minute slots: long enough to gather a few orders, short enough
# that "busy soon" still means soon.
SLOT_SECONDS = 60.0
SLOTS = 10

# Every order counts for less as it ages. After HALF_LIFE seconds an order
# counts half as much as a fresh one, so the model follows the warehouse rather
# than remembering last week for ever.
#
# Shorter than the time a busy aisle STAYS busy, or the model spends its life
# describing where the work used to be. At 240s against a 120s pattern it named
# the previous hot aisle about a third of the time.
HALF_LIFE = 30.0

# Do not act on a guess made from almost nothing.
#
# Two gates, and they check different things. MIN_ORDERS is "have we ever seen
# enough orders to talk about". MIN_EVIDENCE is "is enough of that still RECENT
# enough to count" -- without it, a slow shift of orders leaves six orders on
# the books whose faded weight is nearly nothing, and the model would happily
# declare a winner from rounding error.
MIN_ORDERS = 6
MIN_EVIDENCE = 3.0

# A zone must be this much busier than average before it is worth moving for.
# Below this the honest answer is "no idea", and the robot stays where it is.
INTERESTING = 1.35


@dataclass
class Prediction:
    """What the model thinks, in a form that can be printed."""

    zone: str
    expected: float          # orders expected from this zone in the next slot
    share: float             # its share of all recent orders, 0-1
    lift: float              # how many times busier than an average zone
    sample: int              # how many orders this is based on
    reason: str = ""

    def sentence(self) -> str:
        return (f"aisle {self.zone} has had {self.share * 100:.0f}% of the last "
                f"{self.sample} orders ({self.lift:.1f}x the average)")


class DemandModel:
    """Counts orders per aisle per slot of the hour, with old ones fading."""

    def __init__(self, slot_seconds: float = SLOT_SECONDS,
                 slots: int = SLOTS, half_life: float = HALF_LIFE) -> None:
        self.slot_seconds = slot_seconds
        self.slots = slots
        self.half_life = half_life
        # zone -> slot -> faded count
        self._counts: Dict[str, List[float]] = {}
        self._last_decay_at = 0.0
        self.total = 0                      # orders ever seen, undecayed

    # ------------------------------------------------------------- learning

    def slot_of(self, when: float) -> int:
        return int(when // self.slot_seconds) % self.slots

    def record(self, zone: str, when: float) -> None:
        """One order came from this zone at this time."""
        if not zone:
            return
        self._decay_to(when)
        row = self._counts.setdefault(zone, [0.0] * self.slots)
        row[self.slot_of(when)] += 1.0
        self.total += 1

    def _decay_to(self, now: float) -> None:
        """Fade every count towards zero as time passes.

        Done here rather than on a timer so the model behaves identically no
        matter how often it is asked -- a model whose answer depends on how
        often you look at it is not a model.
        """
        gap = now - self._last_decay_at
        if gap <= 0:
            return
        self._last_decay_at = now
        factor = 0.5 ** (gap / self.half_life)
        if factor >= 0.999:
            return
        for row in self._counts.values():
            for i, v in enumerate(row):
                row[i] = v * factor

    # ----------------------------------------------------------- predicting

    def weight(self, zone: str, when: float) -> float:
        """Faded orders from this zone in the slot that `when` falls in."""
        row = self._counts.get(zone)
        return row[self.slot_of(when)] if row else 0.0

    def recent(self, zone: str) -> float:
        """Faded orders from this zone at any time of the hour."""
        row = self._counts.get(zone)
        return sum(row) if row else 0.0

    def score(self, zone: str, when: float) -> float:
        """How much we expect from this zone around `when`.

        Two signals, added:
          * how busy the aisle has been lately, at any hour, and
          * how busy it usually is at THIS slot of the hour.

        The first works from the very first order. The second is worth nothing
        until the clock has been round at least once, and then starts to pick up
        patterns like "the C aisle goes mad every ten minutes". Using only the
        timetable was the first thing I tried and it predicted nothing at all
        for the first ten minutes, which is most of a demo.
        """
        return self.recent(zone) + 2.0 * self.weight(zone, when)

    def zones(self) -> List[str]:
        return sorted(self._counts)

    def predict(self, now: float, horizon: float = 30.0) -> Optional[Prediction]:
        """The busiest aisle for the period just ahead, or None for "no idea".

        None is a real answer and the common one early on. A robot that gets
        None simply carries on as it always did.
        """
        self._decay_to(now)
        if self.total < MIN_ORDERS or not self._counts:
            return None

        ahead = now + horizon
        scores = {z: self.score(z, ahead) for z in self._counts}
        total = sum(scores.values())
        if total < MIN_EVIDENCE:
            return None               # too little recent evidence to be worth it

        zone = max(scores, key=lambda z: (scores[z], z))
        share = scores[zone] / total
        average = total / len(scores)
        lift = scores[zone] / average if average > 0 else 1.0
        if lift < INTERESTING:
            return None                     # no zone stands out; say so

        pred = Prediction(zone=zone, expected=scores[zone], share=share,
                          lift=lift, sample=min(self.total, 999))
        pred.reason = pred.sentence()
        return pred

    def ranked(self, now: float, horizon: float = 30.0) -> List[Tuple[str, float]]:
        """Every zone, busiest first. For the dashboard."""
        self._decay_to(now)
        ahead = now + horizon
        scores = {z: self.score(z, ahead) for z in self._counts}
        return sorted(scores.items(), key=lambda kv: (-kv[1], kv[0]))

    def to_dict(self, now: float) -> Dict[str, object]:
        """Plain data for the dashboard. Dicts and numbers only."""
        pred = self.predict(now)
        rows = self.ranked(now)
        total = sum(v for _, v in rows) or 1.0
        return {
            "orders_seen": self.total,
            "ready": self.total >= MIN_ORDERS,
            "slot": self.slot_of(now),
            "busiest": pred.zone if pred else None,
            "reason": pred.reason if pred else "",
            "zones": [{"zone": z, "score": round(v, 2),
                       "share": round(v / total, 3)} for z, v in rows],
        }
