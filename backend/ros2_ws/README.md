# FLEET-X — Part 2, the ROS 2 build

This is the ROS 2 / Gazebo half of FLEET-X. It runs on **Ubuntu**, not on a Mac.

**Read this first, because it explains why the code looks the way it does:**

> **None of the coordination logic lives here.** Route finding, booking squares,
> deciding who gives way, breaking deadlocks, bidding for jobs — all of it is in
> `backend/shared/fleetx_core/`, and it is the *same code* the laptop
> simulator runs. This package is the wiring that feeds it real sensors and
> drives real wheels.

That is not tidiness for its own sake. The brain has been run for hours in
simulation, has 251 tests against it, and produced the benchmark numbers
(zero collisions, 77–87% faster than stop-and-wait, from 3 to 20 robots). If we
had rewritten it for ROS 2, none of that evidence would carry over — and the two
versions would quietly disagree within a week.

---

## 1. What you need

Ubuntu 24.04 with ROS 2 Jazzy (or 22.04 with Humble — both should work).

```bash
sudo apt update
sudo apt install -y \
  ros-$ROS_DISTRO-desktop \
  ros-$ROS_DISTRO-gazebo-ros-pkgs \
  ros-$ROS_DISTRO-gazebo-ros2-control \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  python3-colcon-common-extensions
```

The brain itself needs **nothing**. It is pure Python standard library — no pip
installs, no numpy, nothing.

---

## 2. Build it — one command

```bash
cd <repo>/backend/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means you can edit Python and re-run without rebuilding.
You only need to rebuild after changing a `.msg` file.

---

## 3. Run it — one command

```bash
ros2 launch fleet_bringup warehouse.launch.py robots:=3
```

That starts Gazebo with the warehouse, spawns three robots, gives each one a
brain, and begins feeding in orders. You should see robots collect parcels and
carry them to the packing stations, stopping and going round each other on the way.

Useful variations:

```bash
# more robots
ros2 launch fleet_bringup warehouse.launch.py robots:=5

# no Gazebo window (much faster on a laptop)
ros2 launch fleet_bringup warehouse.launch.py robots:=3 gui:=false

# no orders — drive them by hand instead
ros2 launch fleet_bringup warehouse.launch.py robots:=3 orders_every:=0

# once Nav2 is configured, hand navigation over to it
ros2 launch fleet_bringup warehouse.launch.py robots:=3 drive_mode:=nav2
```

### Just the brains, no Gazebo

Useful for checking the fleet protocol on its own:

```bash
ros2 launch fleet_agent fleet.launch.py robots:=3
```

They will not move (no odometry), but they will discover each other, bid for
jobs and book squares. Watch it with:

```bash
ros2 topic echo /fleet/robot_intents
ros2 topic echo /fleet/reservations
ros2 topic list | grep fleet
```

---

## 4. What is in here

```
ros2_ws/src/
├── fleet_msgs/          robot messages plus signed operator goal
│   └── msg/*.msg        one per message class in the brain, same field names
├── fleet_agent/         one robot = one node
│   ├── brain.py         finds shared/fleetx_core and imports it
│   ├── translate.py     brain message <-> ROS 2 message, and squares <-> metres
│   ├── ros2_bus.py      Ros2Bus(FleetBus) — the brain's socket, wired to DDS
│   ├── telemetry.py     ROS-free, tested dashboard state projector
│   ├── gateway_node.py  DDS ↔ backend adapter; no HTML
│   ├── agent_node.py    odometry -> update_pose(), laser -> sense(), brain -> wheels
│   ├── order_source.py  stands in for the warehouse order system
│   └── launch/          one_robot.launch.py, fleet.launch.py
└── fleet_bringup/       the warehouse
    ├── scripts/generate_world.py   builds the world and map FROM the brain's grid
    ├── worlds/warehouse.world      generated — do not hand-edit
    ├── maps/warehouse.{pgm,yaml}   generated — for Nav2
    ├── models/amr.urdf.xacro       a plain diff-drive robot with a laser
    └── launch/warehouse.launch.py  the one command above
```

### Topics

The names come from `03_ROBOT_AND_ROS2_IMPLEMENTATION.md` §3:

| Topic | Message | What it carries |
|---|---|---|
| `/fleet/robot_states` | `RobotState` | where each robot is, ~10 Hz |
| `/fleet/robot_intents` | `RobotIntent` | **where each robot is about to be** |
| `/fleet/battery` | `BatteryStatus` | heartbeat + battery, 2 Hz |
| `/fleet/conflicts` | `ConflictAlert` | "we are going to want the same square" |
| `/fleet/reservations` | `PathReservation` | booking a square for a time slot |
| `/fleet/blocked_aisles` | `BlockedAisle` | "something is in the way here" |
| `/fleet/tasks` | `Task` | a job, announced to everyone |
| `/fleet/task_bids` | `TaskBid` | "it would cost me this much" |
| `/fleet/task_claims` | `TaskClaim` | claimed / picked up / delivered / released |
| `/fleet/wait_reports` | `WaitReport` | "I am stuck behind X" — the wait-for graph |
| `/fleet/yield_requests` | `YieldRequest` | "please move, you are in my way" |
| `/fleet/operator_goals` | `OperatorGoal` | signed dashboard goal for one robot |

The first seven are the ones named in the design docs. The remaining messages are
extensions the working system turned out to need — auctions, deadlock detection
and asking a parked robot to shift are all things the docs describe in prose but
never gave a message for.

`/fleet/*` topics are **global**, deliberately. Each robot's own `odom`, `scan`
and `cmd_vel` are namespaced (`/R1/odom`). That split is what lets robots hear
each other while keeping their wheels separate.

---

## 5. Two ways to drive

**`drive_mode:=simple` (the default)** — a plain turn-then-go controller
publishing `cmd_vel`. It works the moment you launch, with no configuration.
Use it to see the fleet behaving on day one.

**`drive_mode:=nav2`** — publishes the next square as a `PoseStamped` goal and
lets Nav2 handle local obstacle avoidance and smooth motion. This is the right
answer for real hardware, but Nav2 needs configuring first (map server, AMCL,
costmaps, the usual). Start with `simple`, move to `nav2` when you have time.

Either way, **the brain still chooses the route.** Nav2 is only ever given the
*next square*, never the final destination — because the route has to respect
booked squares and other robots' stated intentions, and Nav2 knows nothing about
those. `03_ROBOT_AND_ROS2_IMPLEMENTATION.md` §7: *"Keep your custom logic
above/beside Nav2 rather than rewriting the entire navigation stack."*

---

## 6. Changing the warehouse

The map lives in **one** place: `backend/shared/fleetx_core/grid.py`. After editing it:

```bash
cd <repo>/backend/ros2_ws/src/fleet_bringup
python3 scripts/generate_world.py
```

That regenerates the Gazebo world and the Nav2 map from the same grid the robots
plan against. Never hand-edit `worlds/warehouse.world` — it will drift from what
the robots believe, and every robot will be confidently wrong about where the
shelves are.

### Coordinates

The brain thinks in whole squares, `(0,0)` top-left, y increasing **downward**
like reading a page. Gazebo and Nav2 use metres with y increasing **north**. The
conversion lives in exactly one place, `translate.py`:

```
world_x =  grid_x * RESOLUTION
world_y = -grid_y * RESOLUTION       RESOLUTION = 1.0 m per square
```

---

## 7. Things that will probably need fixing on the first run

Being honest: **this has never been executed.** It was written on a Mac, where
ROS 2 will not install. The logic it calls is heavily tested; the ROS 2 plumbing
around it is not. Expect an hour of small fixes. The likely candidates:

1. **Gazebo version.** Jazzy ships Gazebo Harmonic (`gz sim`), not Gazebo
   Classic. If `gzserver.launch.py` is missing, you are on the new one — either
   install `ros-jazzy-ros-gz` and use `ros_gz_sim` launch files, or
   `sudo apt install ros-jazzy-gazebo-ros-pkgs` for the classic bridge. The
   plugin filenames in `amr.urdf.xacro` (`libgazebo_ros_diff_drive.so`) are
   Classic names; Harmonic uses `gz-sim-diff-drive-system`.
2. **`spawn_entity.py`** is Gazebo Classic. Harmonic uses `ros_gz_sim create`.
3. **TF frames.** Each robot publishes `R1/odom` → `R1/base_footprint`. With no
   `map` frame published, `drive_mode:=nav2` will not work until you start a map
   server and AMCL. `simple` mode does not care.
4. **Robots may drift.** The simple controller has no acceleration limits, so a
   robot can overshoot a square on a slippery floor. Turn down `max_linear` or
   raise `arrive_within` in `agent_node.py`.
5. **The brain path.** `brain.py` walks up from the installed package looking for
   `shared/fleetx_core`. With `--symlink-install` this works. If it does not:
   ```bash
   export FLEETX_BRAIN=/full/path/to/repo/shared
   ```
6. **Message field names.** `.msg` files use flattened `int32[]` arrays for lists
   of squares, because ROS 2 has no array-of-pairs type. `translate.py` packs and
   unpacks them. If you add a field to a brain message, add it in both places.
7. **Clock.** Messages use shared ROS clock seconds. On separate machines, run
   NTP or `chrony` so signature freshness and reservation windows agree.

---

## Live dashboard bridge

On Raspberry Pi, after building and sourcing workspace:

```bash
bash backend/run_ros2.sh
```

This starts API only on port 8000 and listens to DDS. It does not serve HTML
and never silently falls back to simulator mode.

On laptop:

```powershell
uv run frontend/run.py http://<PI-IP>:8000
```

Open `http://localhost:3000`. Browser talks only to laptop frontend proxy;
frontend proxy talks to Pi API.

---

## 8. How to tell it is working

```bash
# every robot should appear
ros2 topic hz /fleet/robot_states

# the interesting one: robots announcing where they are ABOUT to be
ros2 topic echo /fleet/robot_intents

# robots booking squares
ros2 topic echo /fleet/reservations

# the moment two robots notice they want the same square
ros2 topic echo /fleet/conflicts
```

If you see intents flowing and reservations being claimed, the decentralised
protocol is alive — the fleet is coordinating, not being told what to do.

---

## 9. If you want to check the brain on its own

It runs anywhere Python 3.9+ does, including a Mac, with no ROS and no installs:

```bash
cd <repo>
python3 -m unittest discover -s tests      # 251 tests
python3 backend/tools/check_purity.py       # proves the brain has no ROS/web code
uv run backend/run.py                       # simulation backend API
uv run frontend/run.py                      # dashboard, separate terminal
python3 -u backend/tools/final_table.py     # benchmark
```
