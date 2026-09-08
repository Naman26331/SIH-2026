"""Real jobs, and how robots decide between themselves who does each one.

PURE LOGIC ONLY. No web code, no ROS 2 code.

A job is a TWO-LEG journey: go and collect something from a shelf, then take it
to a packing station. That is what a warehouse robot actually does, and it is
what the benchmark in Phase 15 will time.

Nobody hands the jobs out
-------------------------
06_TASK_ALLOCATION section 3:

    Task broadcast
          -> R1 calculates bid = 7.2
             R2 calculates bid = 4.1   <- winner
             R3 calculates bid = 9.4

The order itself comes from outside the fleet -- a customer bought something.
But WHO DOES IT is settled by the robots between themselves, so it keeps
working with the server switched off.

The cheapest robot, not the nearest
-----------------------------------
06_TASK_ALLOCATION section 1: "Choose the robot with minimum expected cost, not
simply the nearest robot." A robot on 15% battery four squares away is the
closest and the worst choice -- it will run flat halfway and the job has to be
done again.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from .grid import Cell

# How long an auction stays open. Long enough for everyone's bid to arrive,
# short enough not to leave a robot standing about.
BID_WINDOW = 0.6

# A task nobody claimed within this long goes back up for auction.
CLAIM_TIMEOUT = 2.5


class TaskStatus(str, Enum):
    QUEUED = "QUEUED"          # created, not yet offered
    ANNOUNCED = "ANNOUNCED"    # auction open, taking bids
    ASSIGNED = "ASSIGNED"      # somebody won it, on the way to collect
    CARRYING = "CARRYING"      # collected, on the way to drop off
    DONE = "DONE"
    FAILED = "FAILED"


@dataclass
class Task:
    """One job. Fields follow the data model in 02_TECHNICAL_ARCHITECTURE §5."""

    task_id: str
    pickup: Cell
    dropoff: Cell
    product: str = ""
    priority: int = 5
    created_at: float = 0.0
    deadline: Optional[float] = None

    # True = "take it to a packing station", any one of them. The robot picks
    # which when it collects the parcel. False = this exact square, no choice.
    #
    # Packing stations are interchangeable in a real warehouse. Naming one
    # specific square in the order is what made every robot queue for the same
    # bay while the others stood empty.
    flexible: bool = False

    assigned_robot: Optional[str] = None
    status: TaskStatus = TaskStatus.QUEUED

    announced_at: float = 0.0
    claimed_at: Optional[float] = None
    picked_at: Optional[float] = None
    done_at: Optional[float] = None
    winning_bid: Optional[float] = None
    reassignments: int = 0

    bids: Dict[str, float] = field(default_factory=dict)

    @property
    def finished(self) -> bool:
        return self.status is TaskStatus.DONE

    @property
    def open_for_bids(self) -> bool:
        return self.status in (TaskStatus.QUEUED, TaskStatus.ANNOUNCED)

    def duration(self) -> Optional[float]:
        """How long from being announced to being delivered."""
        if self.done_at is None:
            return None
        return self.done_at - self.created_at

    def release(self) -> None:
        """Take it off whoever had it and put it back up for auction."""
        self.assigned_robot = None
        self.status = TaskStatus.ANNOUNCED
        self.claimed_at = None
        self.picked_at = None
        self.winning_bid = None
        self.bids.clear()
        self.reassignments += 1

    def to_dict(self) -> Dict[str, object]:
        return {
            "task_id": self.task_id,
            "pickup": [self.pickup.x, self.pickup.y],
            "dropoff": [self.dropoff.x, self.dropoff.y],
            "product": self.product,
            "priority": self.priority,
            "flexible": self.flexible,
            "status": self.status.value,
            "assigned_robot": self.assigned_robot,
            "winning_bid": (round(self.winning_bid, 1)
                            if self.winning_bid is not None else None),
            "bids": {r: round(c, 1) for r, c in self.bids.items()},
            "reassignments": self.reassignments,
            "duration": (round(self.duration(), 1)
                         if self.duration() is not None else None),
        }


# ------------------------------------------------------------- the bid


# Weights for the cost of a job, from 02_TECHNICAL_ARCHITECTURE section 6:
#   C = w1*distance + w2*congestion + w3*battery + w4*urgency + w5*waiting
W_DISTANCE = 1.0
W_CONGESTION = 2.0
W_BATTERY = 25.0
W_URGENCY = 1.5
W_WAITING = 1.0

# Below this a robot is a bad bet for a long job -- it may not finish.
BATTERY_COMFORTABLE = 40.0


def bid_cost(
    distance_to_pickup: float,
    leg_distance: float,
    battery: float,
    waiting: float = 0.0,
    congestion: float = 0.0,
    task_priority: int = 5,
) -> float:
    """What this job would cost THIS robot. Lower is a better bid.

    Distance is only the first term. A robot that is nearest but nearly flat,
    or nearest but already stuck in traffic, should not win.
    """
    cost = W_DISTANCE * (distance_to_pickup + leg_distance)
    cost += W_CONGESTION * congestion
    cost += W_WAITING * waiting

    # Battery risk: the emptier it is, the worse a bet it is for a long job.
    if battery < BATTERY_COMFORTABLE:
        shortfall = (BATTERY_COMFORTABLE - battery) / BATTERY_COMFORTABLE
        cost += W_BATTERY * shortfall * (1.0 + leg_distance / 40.0)

    # An urgent job is worth more effort, so it looks cheaper to take on.
    cost -= W_URGENCY * max(0, task_priority - 5)
    return max(0.0, cost)


def bid_rank(cost: float, robot_id: str) -> Tuple[int, str]:
    """How bids are compared. Lowest wins.

    Rounded to whole units on purpose. Robots hear each other's bids at
    slightly different moments, so a comparison that turned on a tenth of a
    square could let two robots each decide they had won. Same lesson as the
    booking table: coarse comparison, then the lower name settles it.
    """
    return (int(round(cost)), robot_id)


def auction_winner(bids: Dict[str, float]) -> Optional[str]:
    """Who wins, given the bids we have heard. Everyone computes this the same."""
    if not bids:
        return None
    return min(bids.items(), key=lambda kv: bid_rank(kv[1], kv[0]))[0]


# ------------------------------------------------------------ the board


class TaskBoard:
    """The jobs, as far as one robot (or the dashboard) knows about them."""

    def __init__(self) -> None:
        self.tasks: Dict[str, Task] = {}

    def add(self, task: Task) -> Task:
        existing = self.tasks.get(task.task_id)
        if existing is not None:
            return existing
        self.tasks[task.task_id] = task
        return task

    def get(self, task_id: str) -> Optional[Task]:
        return self.tasks.get(task_id)

    def open_tasks(self) -> List[Task]:
        return [t for t in self.tasks.values() if t.open_for_bids]

    def for_robot(self, robot_id: str) -> Optional[Task]:
        for task in self.tasks.values():
            if task.assigned_robot == robot_id and not task.finished:
                return task
        return None

    def held_by(self, robot_id: str) -> List[Task]:
        return [t for t in self.tasks.values()
                if t.assigned_robot == robot_id and not t.finished]

    def release_all(self, robot_id: str) -> List[Task]:
        """Take every job off a robot that has stopped responding."""
        released = self.held_by(robot_id)
        for task in released:
            task.release()
        return released

    # ---------------------------------------------------------------- stats

    def stats(self, now: float) -> Dict[str, object]:
        tasks = list(self.tasks.values())
        done = [t for t in tasks if t.finished]
        durations = [t.duration() for t in done if t.duration() is not None]
        avg = sum(durations) / len(durations) if durations else 0.0
        per_hour = (len(done) / now * 3600.0) if now > 0 else 0.0
        return {
            "created": len(tasks),
            "done": len(done),
            "queued": sum(1 for t in tasks if t.open_for_bids),
            "active": sum(1 for t in tasks
                          if t.status in (TaskStatus.ASSIGNED, TaskStatus.CARRYING)),
            "reassigned": sum(t.reassignments for t in tasks),
            "avg_task_time": round(avg, 1),
            "tasks_per_hour": round(per_hour, 1),
            "total_task_time": round(sum(durations), 1),
        }

    def rows(self, limit: int = 12) -> List[Dict[str, object]]:
        order = {TaskStatus.CARRYING: 0, TaskStatus.ASSIGNED: 1,
                 TaskStatus.ANNOUNCED: 2, TaskStatus.QUEUED: 3,
                 TaskStatus.DONE: 4, TaskStatus.FAILED: 5}
        ranked = sorted(self.tasks.values(),
                        key=lambda t: (order.get(t.status, 9), t.task_id))
        return [t.to_dict() for t in ranked[:limit]]

    def all_done(self) -> bool:
        return bool(self.tasks) and all(t.finished for t in self.tasks.values())

    def __len__(self) -> int:
        return len(self.tasks)
