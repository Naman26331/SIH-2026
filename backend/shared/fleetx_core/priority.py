"""How important a robot thinks it is right now.

PURE LOGIC ONLY. No web code, no ROS 2 code.

The priority score combines task priority, urgency, waiting time and a
battery factor:

    priority_score = task_priority + urgency + waiting_time + battery_factor

The fairness rule:

    effective_priority = base_priority + waiting_seconds * aging_factor

Why aging matters
-----------------
Without it, an unlucky robot in a busy aisle waits forever while others stream
past. That is called starvation. With it, the longer you have been stuck the
more you are owed, so everyone eventually gets their turn.

Whole numbers on the wire
-------------------------
The ROS 2 message declares priority as int32, so scores are kept in
TENTHS of a point: 5.0 becomes 50. That keeps the ROS 2 message an integer
while still letting priorities separate finely.
"""

SCALE = 10                   # priority is stored in tenths of a point

# Priorities are compared in WHOLE POINTS, not tenths.
#
# This is not rounding for tidiness -- it is what makes robots agree. Each
# robot has a slightly different, slightly older copy of the table, so its
# numbers are never exactly the same as anybody else's. If a tenth of a point
# could decide who owns a square, two robots reading the same situation could
# each conclude they had won, and drive into each other. Comparing whole points
# and falling back to the lower robot ID means small differences of opinion
# cannot change the answer.
PRIORITY_BUCKET = SCALE

# How many whole points one robot must lead by before priority decides a
# contest at all. Below this, the lower robot ID wins.
#
# Bucketing alone was not enough: whatever the step size, a robot whose score
# sits exactly ON a boundary is read as one band by itself and another band by
# a robot holding a slightly older copy -- and then both of them conclude they
# have won. A margin means a stale reading, which is only ever off by about
# 0.2 of a point, can never flip the answer.
DOMINANCE_MARGIN = 2

BASE_PRIORITY = 5 * SCALE    # 5.0 -- what an ordinary robot starts on

# Waiting: +1.0 point per second stuck, capped so a robot parked all day
# cannot accumulate an absurd score.
AGING_PER_SECOND = 1 * SCALE
MAX_AGING = 30 * SCALE

# How fast the waiting credit fades once a robot is moving again.
#
# This must be SLOW, and here is why. If the credit vanished the instant a
# robot started moving, then: R2 waits, gains priority, wins the square, stops
# waiting, instantly loses the priority, R1 wins again, R2 stops... and the two
# of them swap places every tick and creep into each other. Fading slowly means
# whoever wins keeps winning long enough to actually get clear.
CREDIT_DECAY_PER_SECOND = 0.25

# Battery: below this, a robot needs a charger more than anyone needs the aisle.
LOW_BATTERY = 25.0
BATTERY_WEIGHT = 4           # points (tenths) per percent below the threshold


def next_wait_credit(credit: float, held_up: bool, dt: float) -> float:
    """Grow the waiting credit while stuck, fade it slowly once moving.

    Kept separate so it is easy to test on its own.
    """
    if held_up:
        return min(credit + dt, MAX_AGING / AGING_PER_SECOND)
    return max(0.0, credit - dt * CREDIT_DECAY_PER_SECOND)


def effective_priority(
    base: int = BASE_PRIORITY,
    waiting: float = 0.0,
    battery: float = 100.0,
    task_priority: int = 0,
) -> int:
    """Work out a robot's current standing, in tenths of a point.

    Everything here is deterministic, so every robot computes the same answer
    for every other robot. That is what lets them agree without another round
    of messages. Ties are still broken by the lower robot ID (04 section 7).
    """
    score = base + task_priority

    # starvation prevention
    score += min(int(waiting * AGING_PER_SECOND), MAX_AGING)

    # a nearly flat robot should not be giving way to anybody
    if battery < LOW_BATTERY:
        score += int((LOW_BATTERY - battery) * BATTERY_WEIGHT)

    return int(score)


def bucket(scaled: int) -> int:
    """Coarse band used for comparison. See PRIORITY_BUCKET above."""
    return int(scaled) // PRIORITY_BUCKET


def yields_to(my_priority: int, my_id: str, their_priority: int, their_id: str) -> bool:
    """Should I give way to them?

    Both robots run this and get opposite answers -- always, whoever asks. It
    never says "both go" and never says "both stop", because the fallback is
    a comparison of names, which cannot be stale or disagreed about.
    """
    mine, theirs = bucket(my_priority), bucket(their_priority)
    if mine - theirs >= DOMINANCE_MARGIN:
        return False                       # I clearly outrank them
    if theirs - mine >= DOMINANCE_MARGIN:
        return True                        # they clearly outrank me
    return my_id > their_id                # too close to call: names decide


def quantise(scaled: int) -> int:
    """Round a score down to a whole point.

    Broadcast scores change in whole steps, at most once a second, instead of
    drifting every tick. Robots then converge on the same number quickly
    instead of forever holding readings that differ by a fraction.
    """
    return (int(scaled) // PRIORITY_BUCKET) * PRIORITY_BUCKET


def as_points(scaled: int) -> float:
    """Turn the stored tenths back into readable points, for display."""
    return round(scaled / SCALE, 1)
