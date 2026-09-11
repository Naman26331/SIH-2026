# PROGRESS_NOTES.md — What we built, when, and why

This is the running diary of the FLEET-X build.

**Rule:** every time anything is built, changed, fixed or deleted, a new entry
goes at the top of the log below. Written in plain English, no jargon.

Each entry answers 4 things:

- **What** — what actually changed (files, folders, features)
- **Why** — the reason we did it
- **Milestone** — which phase from `10_DEVELOPMENT_ROADMAP.md` it belongs to
- **How to see it** — the command to run it, or what to look at

---

## Milestone tracker

Phases come from `10_DEVELOPMENT_ROADMAP.md`.

| Phase | Name | Status |
|---|---|---|
| 0 | Team setup (repo, board, conventions) | Done |
| 1 | Warehouse simulation — one robot moves A → B | Done (grid version) |
| 2 | Three robots navigating at once | Done |
| 3 | Communication (state, intent, heartbeat) | Done |
| 4 | Conflict detection | Done |
| 5 | Reservation (nodes + edges + time) | Done |
| 6 | Negotiation (priority, wait or reroute) | Done |
| 7 | Deadlock detection and recovery | Done |
| 8 | Dynamic obstacles / blocked aisles | Done |
| 9 | Task allocation (auction, reassignment) | Done |
| 10 | Dashboard | Done |
| 11 | Inventory intelligence | Done (inventory-lite) |
| 12 | Demand prediction | Done |
| 13 | Battery and charging | Done |
| 14 | Edge mode (network failure + recovery) | Done |
| 15 | Benchmarking (stop-and-wait vs FLEET-X) | Done |
| 16 | Final demo | Not started |

Statuses used: `Not started` → `In progress` → `Done`.

---

## Cleanup list — tidy up before the demo

Small things that are not worth stopping for now, but should be fixed before
anyone shows this to a judge.

| # | What is wrong | Where | Cause |
|---|---|---|---|
| 1 | Radio ticker shows `PATH_RESERVR1IONundefined% undefined` | [grid_sim/web/index.html](grid_sim/web/index.html) line ~949 | The ticker only knows how to describe three message types. We have since added four more (PATH_RESERVATION, CONFLICT_ALERT, WAIT_REPORT, YIELD_REQUEST), and they fall through to the battery line, printing `undefined% undefined`. The name is also longer than the 64px column, so it overlaps the robot id and reads as `PATH_RESERVR1ION`. Fix: give each message type its own one-line description and let the column grow. |

| 2 | Throughput stops improving past ~10 robots, and robots pile up in the bottom-right corner | the warehouse map in [shared/fleetx_core/grid.py](shared/fleetx_core/grid.py) | The four delivery bays are `DDDD` in a row on the BOTTOM EDGE (row 15), so they can only be entered from above — a dead-end pocket. Every order in the whole simulation targets one of those four squares. At 10+ robots they all converge there, and robots holding parcels cannot get in. **This is a warehouse LAYOUT limit, not a coordination bug.** Possible fixes, in order of honesty: (a) let a robot deliver to ANY free bay instead of one fixed bay, (b) queue outside the pocket instead of crowding it, (c) more bays, or bays with drive-through access from both sides. **FIXED 2026-09-08** — see the layout entry in the log. Two stations of two bays, 15 squares apart, each reachable from four sides. Throughput went from collapsing at scale to flat. |

| 3 | 26 `__pycache__/*.pyc` files are committed to git | throughout | Python's compiled scratch files. They are rewritten every time the code runs, so `git status` shows fake changes constantly, they cause meaningless merge conflicts between teammates, and they are tied to Python 3.14 so they are useless to anyone on another version. Fix (Anushka's call, it is a git change): `git rm -r --cached '**/__pycache__'` then add `__pycache__/` and `*.pyc` to `.gitignore` — this one SHOULD go in `.gitignore` rather than `.git/info/exclude`, because every teammate needs it too, and there is nothing private about it. |

Spotted by Anushka on 2026-09-08. Issues 2 and 3 found on 2026-09-08 during scaling tests.

---

## The two numbers we must hit

- **0 robot-to-robot collisions** in the demo scenarios
- **20% or more faster** than the plain stop-and-wait version

Any measured result goes into the log entry that produced it. Real numbers only,
never made-up ones.

---

## Log

_(Newest entry first.)_

### 2026-09-11 (later still) — Phase 12: guessing where the work will be

- **What:** robots learn which aisles get busy and, when they have nothing to
  do, go and wait there instead of standing where they happened to finish.

  **The model counts. That is all it does.** `shared/fleetx_core/demand.py`
  keeps, per aisle and per slot of the hour, how many orders have come from
  there, with older ones fading. Predicting is reading the table. Two signals
  added together: how busy the aisle has been lately at any hour, and how busy
  it usually is at *this* slot of the hour. The first works from the very first
  order; the second is worth nothing until the clock has been round once and
  then starts catching patterns like "C goes mad every ten minutes".

  I deliberately did not reach for anything cleverer. Every number in it can be
  printed and every decision it leads to is one sentence — *"aisle B has had
  78% of the last 59 orders (3.1x the average)"* — and a judge can check an
  explanation but cannot check a weight.

  **Nobody holds the model.** Each robot builds its own from the order
  announcements every robot already hears, exactly like the job board. No
  server, nothing to fail.

  **Acting on it is designed to be harmless.** Only a robot with no job, no
  charging trip and nowhere to be will move. It waits on ordinary floor, never
  a picking face, a packing bay or a charger. A real job beats a hunch
  instantly. If the model has no opinion, or the move would not gain at least 4
  squares, nothing happens. If two robots are already closer, it leaves it to
  them. **A wrong guess costs one short empty drive, and that is the whole
  risk.**

  **On the dashboard:** a "Where the work is" panel showing each aisle's share
  as a bar, the sentence explaining the call, and a line per robot that moved —
  *"R1 pre-positioned near B8, busy zone (aisle B has had 53% of the last 9
  orders)"*. On the map, a robot waiting on a hunch gets a **dashed** purple
  ring, not a solid marker: it is a guess, and it should not look like a job.

- **The order stream had to change first, and this is the honest bit.** Orders
  used to pick a shelf uniformly at random. Against that, the model correctly
  reported "no idea" for ever and pre-positioning could never be anything but
  noise — **there was genuinely nothing to learn.** Real warehouses are not
  uniform: a few fast-moving lines take most of the picks and which lines drift
  through the day. So most orders now come from whichever aisle is currently
  hot, and the hot aisle changes every two minutes. It is seeded like
  everything else, so both sides of the benchmark still get the identical
  stream. I am flagging this because "I changed the test data and then my
  predictor started working" is exactly the sort of thing that deserves to be
  said out loud rather than buried.

- **How good is it, really?** Measured, not asserted. Blind guessing is 25%.

  | how fast the busy aisle moves | model names it right |
  |---|---|
  | every 60s | 65% |
  | every 120s | 78% |
  | every 300s | 95% |
  | every 600s | 96% |

  Memory half-life was chosen by measuring, not taste: 240s made it describe
  where the work *used to be* (47%), and shorter was better at every pattern
  speed, so it is 30s. At a very slow order rate it holds too few orders to be
  sure and simply declines to answer 74% of the time — the safe direction.

- **And does it actually help? A small amount, and only when it can.**
  Re-measured after the herding fix below, on the corrected code:

  | fleet | orders, off | orders, on | change |
  |---|---|---|---|
  | 3 | 86.0 | 86.0 | +0.0% |
  | 5 | 96.3 | 97.7 | +1.4% |
  | 10 | 98.3 | 98.0 | −0.3% |
  | 20 | 97.7 | 96.7 | −1.0% |

  At 10 and 20 robots the swing is inside noise, and the reason is not that the
  idea is bad: those fleets already deliver ~98 of the 100 orders the benchmark
  issues. There is no headroom to show a gain, or a loss. **Give the fleet more
  work than it can do and the effect shows up properly:**

  | fleet | rate | off | on | change |
  |---|---|---|---|---|
  | 5 | every 1.5s | 140.7 | 148.7 | **+5.7%** |
  | 10 | every 1.0s | 186.7 | 190.0 | **+1.8%** |
  | 20 | every 1.0s | 241.0 | 241.0 | +0.0% |

  So: worth having when the fleet is the bottleneck, negligible or a wash when
  it is not, and costs nothing when it is wrong. Collisions 0 in every one of
  these runs, both settings, all fleet sizes.

- **Four bugs, all mine, and one of them was the same mistake as Phase 14.**

  1. **My anti-herding check read the radio.** I wrote "if two robots are
     already closer, leave it to them" — using the fleet view, which is built
     from messages. Cut the network and every robot believes it is alone, so
     all of them answered "nobody is closer", **four robots drove to the
     identical square and sat holding for each other for 91 seconds.** This is
     precisely the error Phase 14 existed to fix, made again in new code.
     Caught by the Phase 14 blackout test, not by anything I wrote.

     Fixed three ways, each independently sufficient: out of touch means no
     hunches at all (there is no new work arriving to get ahead of, and you
     cannot know where the others are going); robots scatter by their own
     **name**, which needs no messages and cannot go stale; and a hunch is
     abandoned after 8 seconds of getting nowhere.

  2. **Scattering by name still collided**, because each robot built its own
     list of candidate squares filtered by what *it* knew was occupied — so
     "slot 3" meant a different square to each robot. The candidate list is now
     built from the map alone, identical for everybody, and occupancy is only
     consulted at the end to decline. Declining is always safe; queueing is not.

  3. **A waiting robot could sit on a picking face.** I excluded the target
     aisle's faces but not the others', and the middle of aisle B is easily the
     picking square for a rack in A or C.

  4. **A robot that booked a charger kept its old staging marker**, so the
     dashboard showed it "pre-positioned near B8" while it was really driving
     to a charger on 10% battery. Fixed by dropping the hunch wherever the
     robot stops being free.

  The lesson from the first one is worth keeping: **anything that only works
  when the radio works must be tested with the radio off.** I had that written
  down from Phase 14 and still walked into it.

- **Why:** an idle robot standing in the wrong half of the warehouse is the
  cheapest waste there is. And it is a feature a judge can interrogate: every
  move has a printed reason with the numbers behind it.

- **Gate after the fix:** 332 tests pass (22 in `test_phase12.py`, three of
  them written specifically to pin the herding bug down so it cannot come back
  quietly). Purity guard clean. All 28 buttons at 20 robots under load, 0
  failures. Full 3/5/10/15/20 table re-run on the corrected code — see below.
  A live blackout on the running dashboard: 90 seconds with the network cut,
  10 robots, orders every 2.5s — nobody pre-positioned, nobody stalled,
  collisions 0.

- **Milestone:** Phase 12 — Demand prediction.

- **How to see it:** `python3 grid_sim/run.py 8000`, Start orders, and watch
  "Where the work is". It says "learning" for the first few orders, then names
  an aisle and explains why. Robots that move on it appear underneath with
  their reason, and on the map as a dashed purple ring. `world.prepositioning
  = False` turns the whole thing off and the fleet behaves exactly as it did
  before this phase — there is a test that says so.


### 2026-09-11 (later) — Phase 11: orders read like orders

- **What:** shelves have names and stock, so an order says
  **"Mouse x2 from shelf A14"** instead of "collect from (7, 5)".

  - **`shared/fleetx_core/inventory.py`** — a new, deliberately thin layer. It
    names every rack, says what is on it, and can turn a shelf name back into
    the floor square a robot must stand on to reach it. It makes no decisions,
    so it changes nothing about how robots drive or who gives way. It lives in
    the shared brain because Part 2 needs the same names.
  - **Names are warehouse signs.** A letter for each block of racks running top
    to bottom, then a number counting along it: A1 to A36, then B, C, D. 120
    racks get names. The other 24 are the middle of a four-wide block with
    shelves on all four sides — no picker can reach them, so they are never
    offered.
  - **Orders pick a shelf, not a square.** The shelf knows its own face square,
    so the robot drives to exactly where it always drove. Stock goes down as
    orders are made and is restocked when a shelf empties, because an order for
    an empty shelf would be a lie and a warehouse that empties itself makes a
    worse demo than one that does not.
  - **On the dashboard:** the order list reads "Mouse ×2 · A14" with the raw
    squares still there on hover. The collect diamond on the map carries the
    rack name. Each block of racks has its aisle letter above it, big and faint
    — the sign you look up at. Hovering a rack tells you its name, what is on
    it and how many; hovering the floor square in front of it tells you which
    racks it serves. That last one is how you check for yourself that "shelf
    A14" really is the square the robot drove to.

- **One bug worth writing down, because it was an architecture mistake, not a
  typo.** After wiring it all up, the dashboard read "Mouse x2 from shelf A14"
  while the robot actually doing the job had no idea what shelf it was on.

  Cause: every robot builds its **own** copy of the job board from radio
  messages — that is the whole decentralised design — and I had added the shelf
  name to the `Task` but not to the `TaskAnnounce` message that carries it.
  So the world's board and the robot's board were two different versions of the
  same order. Fixed by putting `shelf` and `quantity` on the message, and the
  matching fields on Part 2's `Task.msg` and `translate.py` so the ROS 2 side
  stays in step. **Anything not on the message does not exist as far as a robot
  is concerned** — worth remembering for every future field.

- **Why:** a judge should be able to read one line of the order list and point
  at the exact rack on the map. Coordinates are for the robots.

- **Milestone:** Phase 11 — Inventory intelligence (the "lite" half).

- **How to see it:** `python3 grid_sim/run.py 8000`, press Start orders. The
  Jobs panel now reads in words. Hover any rack on the map for its name and
  stock; hover the square in front of it to see which racks it serves.

- **Gate:** 310 tests pass (17 new in `test_phase11.py`). Purity guard clean.
  All 28 dashboard buttons respond at 20 robots under full load, 0 failures.
  Collisions 0. Full table below.


### 2026-09-11 — Phase 13: robots run out of charge, and do something about it

- **What:** robots now have a battery that matters, and manage it themselves.

  **The decision is a question, not a threshold.** Not "am I under 20%" — a
  fixed line is wrong in both directions, too twitchy for a short hop and too
  brave for a long haul. Every robot asks: *if I drive this whole job and then
  drive on to the nearest charging bay, do I still arrive with something in
  hand?* Same question, asked in two places:
    - **At the auction.** A robot that cannot finish a job and still reach a
      charger does not bid for it. The job goes to somebody who can, rather
      than being abandoned half-done.
    - **While working.** If the sums stop adding up it hands the job back — but
      only if it has not picked the parcel up yet. A robot already carrying
      something delivers it if it possibly can; dropping a box in an aisle to
      go and charge is worse than arriving low.

  **Booking a bay uses the existing reservation table, unchanged.** A charging
  bay is a square, and squares are already booked for a window of time by
  whoever ranks highest. Nothing new was invented: the bay goes into the same
  `_wanted_slots`, gets the same broadcast, obeys the same ownership rule.

  **Lowest battery goes first**, and this is where the old lesson bit again.
  The booking priority is the robot's own urgency — the emptier, the higher —
  rounded into whole 5% bands, because every robot reads everyone else's charge
  a fraction of a second late and a comparison that turns on a hair lets two
  robots each decide they won. Ties break on robot ID, which cannot go stale.

  **Charging:** 4% per second, up to 95%, then back to work. Draining is 0.35%
  per second driving, 0.05% sitting.

  **"Drain batteries" button** on the dashboard runs the whole fleet down to
  8-22% so the demo happens now instead of in twenty minutes. Twenty robots
  wanting four bays is also the interesting case. There is a battery panel
  listing every robot, sorted emptiest first, with a bar that goes red under
  15%, and it says who is charging and who is on their way.

- **Four bugs found building this. Three were mine, one was old and serious.**

  1. **All ten robots booked the same four bays.** Every robot decides in the
     same tick, before anybody has heard anybody, so ten of them each booked
     and all ten drove at four squares and wedged. Booking is only half of it:
     a robot now keeps listening and **stands down** when it hears somebody
     emptier wanted the bay. That is the whole point of sharing a table instead
     of asking a server.

  2. **The chargers were a 2x2 block in the corner** — the exact dead-end
     pocket shape we removed from the delivery bays, and it failed the same
     way. Drain ten batteries and all ten converge on one corner, wedge, and
     sit there until they are flat. They are now spread along row 14 at
     x = 3, 10, 17 and 24, each reachable from four sides.

  3. **Charging lasted one tick.** Deciding runs after the battery pass, so it
     saw "no destination", called the robot idle, and undid the charging status
     every single tick. Ten robots sat on four chargers and drained to nothing
     while apparently plugged in.

  4. **A robot TELEPORTED 0.375 of a square** — and this one was an old trap,
     not new code. `_advance` snapped any robot whose status was not a driving
     one back onto its committed square. That is fine on a square and a
     teleport in an aisle: it jumps backwards through every safety check, which
     is how all three collisions in this project's history happened. Standing
     down from a bay mid-aisle marked the robot idle on the spot and it snapped
     back 0.375 squares. **Fixed at the root:** a part-finished step now holds
     the position its wheels are actually at, whatever the status says. My own
     new teleport test caught this; nothing else did, and the same trap could
     have been reached from any future caller.

  5. **The patrol loop hijacked a robot that was going to charge.** Free roam
     hands a new waypoint to anything briefly without a destination, and it
     sent a robot on 2.7% off across the warehouse instead of to the bay it had
     just booked. Fixed in two places on purpose: the patrol loop leaves
     charging robots alone, *and* the robot re-states its own destination if
     anything overwrites it. Scaffolding should not be able to strand a robot.

- **Why:** a warehouse robot that cannot manage its own charge is a demo, not a
  system. And doing it decentrally is the point — no server decides who charges.

- **Milestone:** Phase 13 — Battery and charging.

- **How to see it:** `python3 grid_sim/run.py 8000`. Load 20 robots, Start
  orders, then press **Drain batteries**. Watch the battery panel: never more
  than four on bays at once, the emptiest going first, the rest carrying on
  working and taking their turn. Fleet average climbs 16% to 57% in under two
  minutes, and the collision counter stays on 0.

- **Evidence:**

  | check | result |
  |---|---|
  | 30 minutes, 20 robots, natural drain | **442 of 446 orders delivered, 72 charge cycles, 0 collisions, nobody ever flat** |
  | Drain to 8-22% with 20 robots, live | never more than 4 on bays, avg 16% → 57% in 110s, 0 collisions |
  | Tests | **293 pass** (16 new in `test_phase13.py`) |
  | Purity guard | clean |
  | All 28 dashboard buttons at 20 robots under load | 0 failures |

- **The cost, stated plainly.** At 3 robots throughput falls 94.8 → 85.4 orders,
  a 9.9% drop. That is not a bug: each robot spends about 35 seconds of a
  400-second window on a charger, which is 8.8% of the fleet's available time,
  and the drop matches. With 5 robots or more it costs nothing measurable —
  98.0 and 98.6 orders, unchanged — because there is always somebody free to
  cover for the one that is plugged in. Small fleets feel it; real ones do not.


### 2026-09-10 (later) — Stop All really stops now, and the map stopped lying

- **What:**

  **1. Stop All did not stop anything.** The button said "Stopped. All robots
  parked where they are" while half the fleet drove on. Measured: 10 robots,
  press Stop All, and 4 of them travelled another 15.9 squares.

  Cause: Stop All only cleared each robot's *destination*. That stops nothing.
  A robot carrying a parcel hands itself the same destination straight back on
  the very next tick — deliberately, because finishing what you are carrying is
  meant to be hard to interrupt (that is the edge-mode design). So the button
  stopped new orders being *created* and nothing else.

  Fix: a real emergency stop, in the brain, because that is a real robot
  feature — `Robot.halt()` / `resume()` and `World.halt_all()` / `resume_all()`.
  A halted robot stops where it stands even mid-aisle, takes no new work, does
  not re-target itself, and reports zero speed so nobody else dead-reckons it
  creeping forward. Any other button releases it. A robot joining a stopped
  fleet joins stopped. Clicking a square to send a robot releases that one
  robot, and says so. There is a red STOPPED banner at the top of the sidebar.
  **Measured after: 0 robots moved, 0.000 squares in 30 seconds.**

  **2. The map had two kinds of pickup marker and one of them was fake.** The
  four green "P" squares were never used by any order — I checked every
  reference. Orders collect from the floor square beside a shelf, because that
  is where goods are. A judge saw a green "Pick station" and then watched every
  diamond appear somewhere else. The P squares are gone. Phase 1's one-robot
  demo now names its start square directly instead of asking for the first P.

  **3. Two orders in five said "fetch a box from the far wall."** The collect
  points were "any floor square with a non-walkable neighbour" — and off the
  edge of the map counts as non-walkable, so the whole outer border looked like
  a shelf. 82 of 202 collect points had no rack anywhere near them. Now it asks
  for an actual shelf: 136 points, and the 16 still on the border are the ones
  genuinely up against a rack.

  **4. Four delivery bays became ten.** All four used to sit on row 13, so every
  delivery in the warehouse ended in the same band however far up it started.
  Six more on the side walls (x=0 and x=27, beside each shelf band). They sit in
  the outermost lane of a three-wide aisle, so a robot parked on one leaves two
  lanes free — checked by parking a robot on all ten at once and confirming the
  floor stays in one piece, which is the dead-end lesson kept as a test.
  Deliveries now land in **all 10 bays, 43% in the bottom band instead of 100%**,
  and the average drive to the nearest bay fell from 9.3 to 7.4 squares.

  **5. The map key now explains the relationship instead of listing shapes.**
  Coloured squares are the building and never move; shapes are the work, one
  pair per live order, drawn in the colour of the robot that won the job. A star
  always sits on a blue packing bay — there is a test that they can never
  disagree, because if they can, the key is lying to a judge.

- **Why:** a demo that shows a button doing nothing, or a label that promises
  something that never happens, costs more than a missing feature.

- **Milestone:** Phase 10 — Dashboard, and Phase 9 — job realism.

- **How to see it:** `python3 grid_sim/run.py 8000`. Load 20 robots, start
  orders, press **Stop all** — everything freezes instantly and a red STOPPED
  banner appears. Press Start orders to release. Watch where the stars land:
  all over the map now, not just along the bottom.

- **THE HEADLINE NUMBER MOVED, AND NOT IN OUR FAVOUR. Read this bit.**

  | fleet | collisions S&W | collisions FX | orders S&W | orders FX | % faster | was |
  |---|---|---|---|---|---|---|
  | 3 | 0 | **0** | 41.2 | 94.8 | 56.5% | *76.6%* |
  | 5 | 0 | **0** | 30.2 | 98.0 | 69.2% | *81.9%* |
  | 10 | 0 | **0** | 35.6 | 98.6 | 63.9% | *85.9%* |
  | 15 | 0 | **0** | 25.4 | 98.6 | 74.2% | *86.8%* |
  | 20 | 0 | **0** | 25.0 | 98.6 | 74.6% | *86.6%* |

  We did not get worse. **FLEET-X went up** (95.8 to 98.6 orders at 20 robots)
  and robots jammed at the end fell to 0.0 at every fleet size. What happened is
  that spreading the delivery bays helped **stop-and-wait far more than it
  helped us** — its throughput at 20 robots nearly doubled, 12.8 to 25.0 —
  because queueing into one delivery band was most of what was killing it. We
  fixed its biggest problem for it.

  And FLEET-X cannot show the gain, because **it is against the ceiling**: the
  benchmark issues 400s / one every 4s = 100 orders, and FLEET-X delivers 100 of
  them. Measured directly:

  | order rate | issued | FLEET-X delivered |
  |---|---|---|
  | every 4s (benchmark) | 100 | **100 — everything there was** |
  | every 2s | 199 | 192 |
  | every 1s | 396 | 290 |

  So the benchmark is currently measuring how much the baseline improved, with
  our side pinned at 100%. The honest read: **the win condition (20% faster,
  zero collisions) is still met at every fleet size, worst case 56.5%**, and the
  number would be higher at an order rate that does not cap us. I have NOT
  changed the benchmark rate — moving the goalposts after seeing a result you
  do not like is exactly the wrong move, and it is the user's call.

- **The full gate:** 277 tests pass (9 added). Purity guard clean. All 27
  dashboard buttons respond at 20 robots under full order load, 0 failures.
  **Collisions 0 in every run on both sides.** Closest approach 1.000 squares;
  biggest single-tick move 0.1250, exactly the wheel limit, so nothing
  teleported.

- **One test I had to change, and why it is not me weakening a test.**
  `test_stop_parks_everyone` asserted that every robot's goal was `None` after
  Stop All. That sounds right and guaranteed nothing — clearing the destination
  is precisely the thing that did not work. It now asserts that nothing moved
  and every robot is halted, which is strictly stronger and catches the bug the
  old wording sailed past. A stopped robot now keeps its goal on purpose, so
  you can see what it was in the middle of.


### 2026-09-10 — Dashboard lag fixed, and two real bugs found while sweeping the buttons

- **What:**
  - **The lag.** Measured first, then fixed. At 20 robots the dashboard was
    being sent **194,916 bytes twenty times a second — 3.8 MB every second**.
    Nearly four fifths of that was `views`: a complete world-picture for every
    single robot, when the screen only ever shows one at a time. Three changes:
    - `world.snapshot(focus=...)` now sends the full picture for the one robot
      you have selected and a stub for the other nineteen. Every robot still
      sends the squares it has booked, because the map tints those.
    - The dashboard tells the server which robot it is showing, through a new
      `/api/focus`. Selecting a different robot switches which one gets detail.
    - Each robot was carrying its whole list of predicted conflicts — 680 of
      its 900 bytes — which the dashboard never read. It now sends the count.
      Routes are capped at 12 squares ahead, the conflict list at 14 rows.

    **Result: 3,807 KB/s → 524 KB/s at 20 robots. An 86% cut.** Building the
    snapshot costs 1.1 ms, about 2% of one core at 20 frames a second.

  - **The browser side.** The map still redraws 20 times a second so motion
    stays smooth, but the twelve text panels down the side now refresh 5 times
    a second — nobody reads faster than that — and a panel is only rewritten
    when its text has actually changed. Anything that must catch every single
    update (collisions, new conflicts, the event feed) still runs every frame.

  - **Buttons that would not take a click.** Root cause: a click only counts if
    the press and the release land on the *same* element. The sidebar was being
    rebuilt twenty times a second, so a button could be destroyed in the
    fraction of a second between pressing and letting go — and the browser then
    fires no click at all. Nothing was wrong with the Fail button itself.
    Fix: while a mouse button is held down anywhere on the page, panel rewrites
    are queued instead of applied, and the queue is flushed shortly after the
    release. No control can now be pulled out from under your finger.

  - **The sidebar jumping.** Every list that can grow — robots, jobs, decisions,
    reservations, conflicts, messages, the event feed — now has a fixed height
    and scrolls inside itself. Adding robots or letting the job list build up no
    longer makes the sidebar taller and pushes every button below it down the
    page. The controls people actually press (Add robot, Remove robot, Start
    orders, Stop all, the order-rate slider) are now in one block pinned to the
    top that never changes size. Fail/Revive got their own fixed section instead
    of sitting inside the Jobs panel, where the growing job list kept shoving
    them around.

- **Two real bugs the sweep turned up** (found by clicking all 27 controls with
  20 robots on the floor and orders coming once a second):

  1. **The scenario buttons stacked robots on top of each other, and it
     registered as collisions.** Staging a demo starts by parking everything out
     of the way, and that step cycled a hard-coded list of *five* corner squares
     with a modulo. Fine for the three-robot demos it was written for. With
     twenty robots it dropped four robots onto every square — twenty robots
     became five stacks, and the collision counter went off, correctly. Pressing
     Head-on at 20 robots took the count from 9 to 21.
     **This was never a coordination failure and never affected the benchmark**,
     which builds its fleets a different way. It was the demo setup putting
     robots in the same place before the brain ever got a say. Fixed: every
     robot now gets a parking square of its own, chosen as far as possible from
     the corridor the demos run down, and never one the demo needs.

  2. **Free roam only moved three robots.** The patrol routes were a hard-coded
     table for R1, R2 and R3; every other robot found nothing and stood still.
     With 20 robots on the floor, Free roam moved 3 of them. Fixed: R1, R2 and
     R3 keep their hand-picked routes — them meeting at (13, 8) is the point of
     the demo — and everyone else gets their own long route, picked the same way
     every time so the demo is repeatable.

- **Three things that looked like bugs and were not.** My first sweep script
  sent `cut`, `silent` and `latency` where the endpoints expect `down`, `on` and
  `latency_ms`, so Restore network reported "NETWORK DOWN", Unmute reported
  "silenced", and the latency slider looked ignored. The browser sends the right
  names. Worth writing down because for twenty minutes I believed I had found
  three bugs in code that was correct — the test was wrong, not the dashboard.

- **A jam that was honest behaviour.** After 700 random button presses the fleet
  had 13 of 20 robots stuck. That turned out to be the leftovers of the soak
  itself: it had silenced robots at random and never unmuted them. On a fresh
  world the same fleet ran clean. Silencing 8 of 20 radios deliberately gives
  2–4 robots briefly stuck at a time, recovering on their own, throughput barely
  down, collisions 0 — which is the graceful degradation we designed for.

- **Why:** the dashboard is the demo. If it stutters and eats clicks in front of
  a judge, none of the coordination work behind it gets seen.

- **Milestone:** Phase 10 — Dashboard (polish and hardening).

- **How to see it:** `python3 grid_sim/run.py 8000`, open
  `http://localhost:8000`, press **+ Add robot** until you have 20 and drag the
  order-rate slider to the fastest setting. The map stays smooth, the sidebar
  stops moving, and every button takes a click first time.

- **The full gate, after every change above:**

  | | result |
  |---|---|
  | Tests | 268 pass, 0 fail (266 before, 2 added) |
  | Collision suite | 16 of 16 pass |
  | Purity guard | clean — no web or ROS 2 code in the shared brain |
  | 701 random button presses at 20 robots, full order load | 0 failures, slowest reply 18 ms, **0 collisions** |

  | fleet | collisions S&W | collisions FLEET-X | orders S&W | orders FLEET-X | % faster |
  |---|---|---|---|---|---|
  | 3 | 0 | **0** | 22.4 | 95.6 | 76.6% |
  | 5 | 0 | **0** | 17.6 | 97.2 | 81.9% |
  | 10 | 0 | **0** | 13.8 | 97.8 | 85.9% |
  | 15 | 0 | **0** | 12.8 | 96.8 | 86.8% |
  | 20 | 0 | **0** | 12.8 | 95.8 | 86.6% |

  Closest two robots ever came: 1.000 squares (touching is under 0.7).
  Biggest single-tick move: 0.1250 squares — exactly what the wheels allow, so
  nothing teleported. **This table is identical to the one from before these
  changes, digit for digit**, which is the point: the dashboard got faster and
  the brain was not touched.

- **Gate detail:** 268 tests pass (2 new ones added below). 701 random button presses at 20
  robots under full order load: 0 failures, slowest reply 18 ms, 0 collisions.
  Full 3/5/10/15/20 table re-run — see the table entry below it.

- **Two new tests, so neither bug can come back:**
  - `test_staging_never_stacks_a_FULL_fleet_either` — stages every demo at 5,
    10 and 20 robots and insists no two ever share a square. The three-robot
    version of this test already existed and passed straight through the bug.
  - `test_free_roam_actually_moves_the_whole_fleet` — 20 robots, Free roam, 20
    seconds, and every single robot must have moved.

  Both were checked against the old code first: the old staging put 20 robots
  on 8 squares, and old Free roam left 15 of 20 standing still. A test that
  cannot fail is not a test.


### 2026-09-07 (late) — Two diagrams added so the last pages argue a point

- **What:**
  - **Research and references** — the "gap we fill" paragraph is now a picture.
    It is a map with two questions on the axes: who decides (one central
    computer vs. every robot) and what robots share (just position vs. intent +
    time). Every paper we cite is plotted on it. The top-right corner is empty;
    that is where FLEET-X sits. One line of text under it carries the argument.
  - **Impact and benefits** — the target-outcome bar chart is replaced by a flow
    chart of what actually happens when the network dies: Normal → Degraded
    link → Safe local mode → Link back, re-sync, with the note that obstacle
    detection, e-stop and speed limiting never need the network at all.
  - Anushka was shown four drafts first and picked these two.
- **Why:** Both pages were lists. A list tells the judge what we read; these
  say what we concluded. The network flow chart is also the answer to the
  question a judge is most likely to ask.
- **Milestone:** Phase 0 — Team setup.
- **How to see it:** Open `Fuego_SIH26123_FLEET-X_Idea.pdf`, pages 5 and 6.
- **Note:** the "≥ 20% faster" claim still appears, as a KPI tile at the top of
  page 5. It is a target, not a measured result, until the benchmark is run.
- **Still to fill in:** slide 1 says `<Team ID from SIH portal>`.

### 2026-09-07 (evening) — Trimmed the deck back on request

- **What:**
  - **Slide 1** is now the official template page exactly as supplied — same
    layout, same "TITLE PAGE" heading, same bullet list. Only the six field
    values are filled in. Font dropped from 24pt to 18pt purely so the long
    problem-statement title still fits inside the template's own text box.
  - **Slide 3:** removed the Raspberry Pi and LiDAR-robot photos. The tech
    stack now has 8 cards instead of 6 (Backend and Dashboard split again, plus
    a new "Edge hardware" card) and fills the whole column.
  - **Slide 4:** removed the 10-week build plan. The three feasibility cards
    are taller with an extra point each, and the risk/strategy rows are bigger.
  - **Slide 5:** removed the three photos and the three audience cards. In
    their place there is now a "what changes on the floor" table — five real
    situations, what a warehouse does today, and what happens on FLEET-X.
- **Why:** Anushka asked for it. The table also says more than the photos did —
  it shows the difference rather than describing the audience.
- **Milestone:** Phase 0 — Team setup.
- **How to see it:** Open `Fuego_SIH26123_FLEET-X_Idea.pdf`.
- **Still to fill in:** slide 1 says `<Team ID from SIH portal>`.

### 2026-09-07 (later) — Redesigned the deck: visuals, charts and real photos

- **What:** Rebuilt `Fuego_SIH26123_FLEET-X_Idea.pptx` / `.pdf` to be far less
  text-heavy.
  - Removed the blue bar and the "@SIH" footer from the bottom of every slide.
    The page number is still there, small and grey in the corner.
  - Slide 1 is now just the SIH title and the six required fields, at a much
    bigger size. FLEET-X, the tagline and the team member names are gone from it.
  - Drew five graphics instead of describing things in words: a colour-coded
    system architecture, a six-step chevron flow, a "who goes first at the
    junction" mini-diagram, a target-outcome bar chart, and a 10-week build
    timeline.
  - Added four real photos (warehouse AGVs, a port AGV, a Raspberry Pi 5, a
    LiDAR robot) taken from Wikimedia Commons under open licences. They are
    credited in small print at the bottom of slide 6.
  - Every card now has a colour of its own instead of everything being blue.
- **Why:** A wall of text is boring to look at and hard to judge quickly.
  Judges skim; pictures and one clear chart land faster than paragraphs.
- **Milestone:** Phase 0 — Team setup.
- **How to see it:** Open `Fuego_SIH26123_FLEET-X_Idea.pdf`.
- **Note on the chart:** the bar chart is labelled a *target*, not a result. We
  have not run the benchmark yet, so it must not be presented as measured.
- **Still to fill in:** slide 1 says `<Team ID from SIH portal>`.

### 2026-09-07 — Made the SIH idea submission deck (6 slides)

- **What:** Built `Fuego_SIH26123_FLEET-X_Idea.pptx` and exported
  `Fuego_SIH26123_FLEET-X_Idea.pdf`. Both sit in the project root. Made by
  filling in the official `SIH2026-IDEA-Presentation-Format (1).pptx` template —
  same template design, same six headings, we only added our content. The
  template's 7th "important instructions" slide was deleted, which the template
  itself says to do. Content came from reading all 13 project markdown files;
  the layout style was modelled on the Hyper Grey reference deck.
  The build script lives outside the repo (temporary), so nothing extra was
  added to the project.
- **Why:** The SIH portal needs a 6-slide idea PDF, and this is the first thing
  the jury sees.
- **Milestone:** Phase 0 — Team setup.
- **How to see it:** Open `Fuego_SIH26123_FLEET-X_Idea.pdf` in the project
  folder. Open the `.pptx` if you want to edit anything.
- **Still to fill in:** Slide 1 says `<Team ID from SIH portal>` — replace it
  with the real Team ID before uploading.

### 2026-09-09 — Phase 14: it really does keep working without a network

- **Anushka spotted the hole in the pitch.** "Kill the network" used to make the
  warehouse stop, which made the line "the central server is not
  safety-critical" false. Investigating it turned up something worse.

- **THE BUG AT THE HEART OF IT.** I had been calling this a "local safety
  reflex" for six phases:

      for note in self.fleet.fresh(now):    # <- robots HEARD ON THE RADIO

  It was not local at all. It was built from radio messages. Cut the radio and,
  two seconds later, every robot believed it was alone in the warehouse. The
  only reason 100% packet loss showed zero collisions was that nothing was
  moving. **If they had been moving they would have driven straight through
  each other.** A claim repeatedly made in this file was simply not true.

- **The fix:** robots now SEE each other. A laser reflects off a robot exactly
  as it does off a dropped pallet, so the world hands each robot the positions
  of other robots within sensor range, as sensor readings rather than messages.
  The safety check merges what was heard with what can be seen; with the radio
  dead the second half alone keeps them apart. On real hardware this is the
  LiDAR, which needs no network at all.

- **THREE MORE BUGS the fix uncovered:**
  1. **A sensor cannot tell parked from moving.** A shape is a shape, so every
     robot yielded to IDLE robots for ever. Fixed by watching whether a shape
     stays on the same square -- a moving robot crosses one in well under a
     second, a parked one does not.
  2. **Robots declared each other dead.** In the three seconds before safe mode
     engaged, every robot saw every other as "gone quiet", released their jobs
     and grabbed them. THREE ROBOTS ENDED UP CARRYING THE SAME PARCEL TO THE
     SAME SQUARE and wedged each other there. The missing rule is a classic
     one: **if you cannot reach anyone, the one that has gone quiet is you.**
  3. **Rerouting was still radio-only.** A stuck robot replanned straight
     through the robot blocking it, concluded the route was unchanged, and
     waited 109 seconds. Rerouting, stepping aside and parking now all use
     `known_occupied()`, which merges both sources.

- **Safe mode** (03_ROBOT_AND_ROS2 §9, 01_PRODUCT §7): heard nothing for 3
  seconds and we know we are not alone -> half speed, finish the job in hand,
  take NO new work (no radio means no auction: nobody would hear the bid or the
  claim), keep avoiding everyone on sensors. On reconnect: resync state, intent,
  bookings and job claims, and the order system repeats unclaimed orders so a
  blackout does not lose them for ever.

- **LIVE TEST on the real server:**

      --- normal ---        t= 40s  delivered=11  collisions=0  on sensors=0
      >>> NETWORK CUT <<<
                            t= 48s  delivered=13  collisions=0  on sensors=3  moving=3
                            t= 64s  delivered=15  collisions=0  on sensors=3  moving=0
      >>> RESTORED <<<
                            t= 98s  delivered=15  collisions=0  on sensors=0  moving=3
                            t=118s  delivered=18  collisions=0  on sensors=0  moving=3

  Four orders delivered DURING the blackout. Robots never came closer than
  1.00 squares. On restore the full protocol is back: auctions, giving way, the
  lot.

- **The line to use with judges, now true rather than aspirational:**
  **"The warehouse degrades, it doesn't stop, and it never becomes unsafe."**
  Do not claim throughput holds up -- it does not, and saying so invites a
  question you would lose. No new orders can reach the robots, they run at half
  speed, and coordination is cruder. That IS graceful degradation (01 §7).

- **Full gate re-run, because the brain changed:**

      fleet |  collisions   |  orders delivered  | % faster
       size |  S&W  FLEET-X |   S&W     FLEET-X  |
          3 |    0        0 |  22.4        95.0  |   76.4%
          5 |    0        0 |  17.6        97.0  |   81.9%
         10 |    0        0 |  13.8        97.6  |   85.9%
         15 |    0        0 |  12.8        97.6  |   86.9%
         20 |    0        0 |  12.8        94.2  |   86.4%

  Identical to the pre-Phase-14 numbers within noise, so edge mode cost nothing.
  266 of 266 tests pass (15 new). Collision suite clean. NEW test: 100% packet
  loss from a cold start, not one message ever -- 0 collisions. Purity clean.

- **Dashboard:** a "Cut the network" button, a flashing NETWORK DOWN banner, an
  "On sensors" counter and a SAFE_MODE badge.

---

### 2026-09-09 — Phase 10: the dashboard, and the side-by-side screen

- **What:** Built the comparison screen from 07_DASHBOARD §9 — both fleets
  running the SAME orders at the same time, live, so a judge watches
  stop-and-wait jam up while ours keeps flowing.

  - `grid_sim/comparison.py` — NEW. Runs both fleets in lockstep.
  - `grid_sim/web/compare.html` — NEW. The screen, at `/compare`.
  - `grid_sim/server.py` — three small routes added. The existing dashboard
    was NOT touched.
  - `tests/test_phase10.py` — NEW, 16 tests, almost all about fairness rather
    than about pictures.

- **How the fairness claim is made honest.** Each order is created ONCE and
  handed to both fleets on the same tick. The obvious alternative — two order
  generators started from the same seed — would also match today, but it relies
  on two random sequences staying in step for ever, and that is a claim a judge
  would be right to poke at. Here there is nothing to drift from. The screen
  also shows a live "clocks in step" tick, and the header states the whole
  claim: same warehouse, same orders same instant, same start positions, same
  A*, same safety reflex — the only difference is the coordination.

- **Measured live, 10 robots a side, one order every 2s:**

      t=  35s   S&W  4  |  FLEET-X 14   gap 3.5x
      t=  71s   S&W  8  |  FLEET-X 31   gap 3.9x
      t= 145s   S&W  8  |  FLEET-X 63   gap 7.9x
      collisions 0/0 throughout, clocks in step throughout

  Stop-and-wait flatlines at 8 delivered and never recovers. That is the
  moment the project explains itself without anybody saying anything.

- **How to see it:** `python3 grid_sim/run.py`, then
  http://localhost:8000/compare  (the ordinary dashboard is still at `/`).
  Controls: Start/Pause, Reset, robots 3/5/10, speed 1x/2x/4x. The speed
  control only does more steps per real second — the clock on screen is
  simulated time either way, so nothing is being fudged.

- **A BUG I WROTE, AND THE LESSON: a test that can hang is worse than a test
  that fails.**

  I wrote a test helper that loops until the simulated clock reaches a target:

      while c.left.sim_time < target:
          c.tick(dt)

  But `tick()` deliberately does nothing while the comparison is PAUSED, so the
  clock never moves and the loop never ends. And one of my own tests —
  `test_nothing_happens_until_you_press_start` — is specifically about the
  paused state. It burned **87 minutes at 100% CPU** before anyone noticed, and
  it was only caught because Anushka asked me to stop the web server and I
  looked at what else was running.

  A failing test tells you something. A hanging test just quietly eats the
  laptop, and I had reported it as "running in the background" for over an hour
  without checking on it.

  Two fixes: that test now steps a FIXED number of ticks instead of waiting on
  a clock that cannot move, and the helper has a hard iteration cap so nothing
  in that file can hang for ever again.

- 251 of 251 tests pass (16 new). Purity guard clean. Nothing in
  `shared/fleetx_core/` was touched, so every number already proven is
  unaffected.

---

### 2026-09-08 — Layout fix: spread the packing stations (cleanup item 2)

- **Teammate's idea, and it worked.** The four delivery bays used to sit in a
  row ON the bottom wall — a dead-end pocket with one mouth. Now they are two
  stations of two bays, 15 squares apart, each reachable from four sides:

      row 12  ............................
      row 13  PP...DD.............DD..CC..
      row 14  PP......................CC..
      row 15  ............................   <- the old pocket, now clear

  **Still exactly four bays.** Capacity was deliberately held constant so any
  improvement is provably about PLACEMENT, not about adding more doors. Pick
  stations and chargers were left untouched at four each.

- **The deeper flaw was not the map.** Every order named ONE specific delivery
  square, so robots queued for it while the other bays stood empty. Now a robot
  chooses its bay when it COLLECTS the parcel: nearest by road, with a penalty
  for any bay another robot is on or heading to, and it will switch bay if it
  gets stuck on the way. Worked out locally from what it has heard, like
  everything else here — no dispatcher.

- **RESULT — same four bays, just not in a corner:**

      fleet | orders delivered        | % faster | collisions
       size | before -> after         |          |
      ------+-------------------------+----------+-----------
          3 |  83.8  ->  96.6         |  76.8%   |     0
          5 |  96.4  ->  97.8         |  82.0%   |     0
         10 |  93.6  ->  97.6         |  85.9%   |     0
         15 |  61.6  ->  97.2         |  87.0%   |     0
         20 |  40.4  ->  95.6         |  86.6%   |     0

  Throughput used to COLLAPSE 58% from 5 to 20 robots. It is now flat: 97.8
  down to 95.6, a 2% drop across a 7x larger fleet.
  Seconds per order: 4.1s at 3 robots, 4.2s at 20 — essentially constant.
  Robots jammed at the end: was 8.8 of 20, now 0.4 of 20.

- **What to claim now:** "Same four packing stations. We just stopped putting
  them all in one dead end. Throughput went from collapsing at scale to flat
  from 3 to 20 robots, with zero collisions throughout." The scaling caveat
  from the previous entry is GONE.

- **One number went DOWN and it is not a regression.** "% faster" at 3 robots
  fell from 84.7% to 76.8%, because the BASELINE improved too (12.8 -> 22.4
  orders). Spread-out bays help stop-and-wait as well. FLEET-X also improved
  (83.8 -> 96.6). We are beating a stronger opponent, which is the more honest
  comparison — say this if anyone notices the smaller percentage.

- **Gates, all passed before anything was reported:**
  - 235 of 235 tests pass, with ZERO modifications
  - full collision suite A-E: 0 collisions, closest approach 1.000
  - 50% packet loss: 0 collisions
  - final table 3/5/10/15/20: 0 collisions at every size
  - biggest single-tick move 0.1250 squares, exactly the wheel limit

- **Predicted 16 test coordinate updates, needed ZERO.** The old bays are now
  plain FLOOR and still walkable, so a test saying "deliver to (13,15)" tests
  exactly what it did before — collect here, drive there, drop off. Delivering
  to an exact square is still supported (`flexible=False`). The only test that
  mentions DROP at all just checks stations of each kind exist somewhere, which
  is still true. No assertion changed, no logic changed, nothing weakened.

- **A mistake I caught in myself:** my first map edit also wiped a row of pick
  stations and chargers as a side effect, which would have quietly changed two
  other variables and made the experiment worthless. Reverted before measuring.

- **Re-run it:** `python3 -u tools/final_table.py --seeds 5 --robots 3 5 10 15 20`
  Raw data for all 50 runs in `final_table.csv`.

---

### 2026-09-08 — Phase 15b: THE FINAL TABLE (this is the one to trust)

- **Every number here comes from ONE run of the current code**, so nothing on a
  slide can have been measured on a different version. Produced by
  `python3 -u tools/final_table.py`. Raw data for all 50 runs in
  `final_table.csv`.

      fleet |  collisions   |  orders delivered  | % faster
       size |  S&W  FLEET-X |   S&W     FLEET-X  |
      ------+---------------+--------------------+----------
          3 |    0        0 |  12.8        83.8  |   84.7%
          5 |    0        0 |  12.0        96.4  |   87.6%
         10 |    0        0 |   6.0        93.6  |   93.6%
         15 |    0        0 |   4.6        61.6  |   92.5%
         20 |    0        0 |   4.8        40.4  |   88.1%

      Collisions across every run:  FLEET-X 0, stop-and-wait 0
      Closest two robots ever came: 1.000 squares (touching is under 0.7)
      Biggest single-tick move:     0.1250 squares (wheels allow 0.125)

  Seconds per order: 31.2 -> 4.8 (3 robots), 33.3 -> 4.1 (5), 66.7 -> 4.3 (10),
  87.0 -> 6.5 (15), 83.3 -> 9.9 (20).

- **WHAT TO CLAIM:** zero collisions at 3-20 robots; 84-94% faster than
  stop-and-wait at every size; this warehouse runs best at 5-10 robots.

- **WHAT NOT TO CLAIM:** "it scales to 20 robots". It does not. Throughput
  FALLS from 96 orders at 5 robots to 40 at 20. The percentage stays high at 20
  only because the baseline degrades even faster, so **the percentage is not the
  scaling signal — orders delivered is.** Supporting evidence: 8.8 of 20 robots
  jammed at the end on average, against 0.0 at 3-10 robots. That is cleanup
  item 2, the four delivery bays in a dead-end pocket.

- **Why this metric.** Both fleets get the same orders for the same 400s and we
  count what was delivered; "% faster" is the drop in average seconds per
  delivered order. The two alternatives were both worse:
    * "time to finish the batch" — under load stop-and-wait never finishes, so
      there would be no number at all.
    * "average time per order as the fleets report it" — this one LIES. A
      jammed fleet delivers the easy early orders while the floor is empty then
      seizes up, so its own average covers only what it survived, and it can
      look FASTER while doing a sixth of the work.
  Counting output over a fixed clock has neither hole, and no run has to be
  excluded — which was the weakness of the earlier 69.6%/80% figures.

- **A CORRECTION I OWE THE RECORD.** Partway through I told Anushka FLEET-X was
  "clean to 5 robots only" and that 10 robots collapsed to 24 orders. That was
  WRONG — 10 robots delivers 93.6, as healthy as 5. I measured it on a build
  that had the teleport fixed but not yet the reverse-gear fix, and stated a
  conclusion from a half-finished state. She told me to hold off acting on it
  and she was right.
  **Rule taken from this: do not draw conclusions from a partially-fixed
  build. Finish the fix, then measure.**

- **Also fixed in this round (the 20-robot collision):**
  1. **`consider_reroute` teleported robots.** It swapped the path while the
     robot was part way along a segment, leaving progress untouched, so
     "62% of the way west" became "62% of the way east" and the robot jumped
     1.37 squares through another robot. Nothing predicts teleportation, which
     is why the crash was also unpredicted. Now it plans from the square it is
     committed to entering.
  2. **The obstacle-replan had the same hole** and snapped robots backwards.
  3. **Robots could not reverse.** Fixing 1 and 2 exposed a real deadlock
     underneath: two robots each a fraction of a square into the SAME square
     from opposite sides, both stopped, neither able to finish or get out of
     the way. Teleporting had been hiding it. Real robots have reverse gear;
     now these do too. This is what restored throughput at 10+ robots.
  4. **A proper invariant test:** no robot may move further in one tick than
     its wheels allow. All three collisions this project has ever had were
     teleports — this catches the whole class at once instead of finding them
     one crash at a time.

- 235 of 235 tests pass. Full collision suite clean. Purity guard clean.

---

### 2026-09-08 — Phase 15: THE BENCHMARK — 69.6% faster, 0 collisions

- **What:** Built the stop-and-wait baseline and `tools/benchmark.py`, and
  measured the second half of the win condition.

- **THE HEADLINE: 69.6% faster than stop-and-wait, with zero collisions on
  both sides.** Target was 20%.

      Time to deliver the first 8 orders (10 seeds each, order every 4s)
        3 robots:  stop-and-wait 219.8s  ->  FLEET-X 46.2s   = 79.0% faster
        5 robots:  stop-and-wait 111.9s  ->  FLEET-X 44.4s   = 60.3% faster

      Orders delivered in 400 seconds
        3 robots:  17.3  ->  81.1   (369% more work)
        5 robots:  12.7  ->  96.2   (657% more work)

      Cleared a 20-order batch
        stop-and-wait  1 of 20 runs
        FLEET-X       19 of 20 runs

      Collisions across every run:  0 for BOTH fleets

  Raw per-seed data in `benchmark_results.csv` (121 rows, every run).

- **Making it a fair fight.** The harness CHECKS fairness rather than claiming
  it, and refuses to run otherwise: same map, same start positions, same
  speeds, same orders in the same order, same A*, same job auction, same local
  safety reflex. The ONLY difference is the coordination logic — intent
  sharing, booking, negotiation, deadlock breaking.

  I was deliberately GENEROUS to the baseline, because every concession makes
  our own number smaller:
    * it gets the identical LiDAR-style safety reflex (without it the baseline
      simply crashes, and a fleet that crashes is not a comparison)
    * it gets a standoff tie-break so it does not freeze for ever
    * it gets a random back-off that retreats AWAY from the blocker and pauses,
      rather than immediately walking back into it

- **THE METRIC HAD TO CHANGE, and here is why.** The roadmap asks for "time to
  complete the workload". That does not work here, because the load has only
  two regimes and nothing in between:

      order every 12s+  ->  the two fleets are IDENTICAL (0% apart). No
                            traffic, so there is nothing to coordinate.
      order every 3-4s  ->  stop-and-wait NEVER finishes, at any fleet size
                            from 3 to 8 robots.

  So the headline is "time to deliver the first 8 orders" — same order stream,
  same count, both fleets actually reach it.

- **A METRIC THAT LIED, which I nearly reported.** "Average time per order"
  showed the baseline at 11.6s against FLEET-X's 30.4s on one seed — the
  baseline looks FASTER. It is not: it delivered 5 orders against 58. It
  completed the easy early ones while the floor was empty, then jammed and
  never delivered the hard ones, so its average covers only what it survived.
  Survivorship bias. Thrown out.

- **HONEST BAD NEWS — say these out loud before a judge finds them:**
  1. **We lost one seed.** 3 robots, seed 1: stop-and-wait 44.6s, FLEET-X
     48.0s. We were 7.6% SLOWER. Light contention that run, and our caution
     (booking, safety margins) costs a little when there is nothing to
     coordinate. It is in the table and in the CSV.
  2. **FLEET-X is not perfect either.** One batch of 20 did not finish
     (5 robots, seed 5, delivered 19/20, zero collisions). A real robustness
     gap worth chasing before the finals.
  3. **Three seeds are excluded** where stop-and-wait never reached 8
     deliveries. Those are its WORST runs, so excluding them flatters the
     baseline and makes our figure an UNDER-estimate. The harness prints this
     caveat itself.

- **The best slide is not the percentage.** The baseline ranged from 44.6s to
  398.2s on the same workload — a 9x spread. FLEET-X ranged 39.6s to 52.1s.
  For a warehouse, predictability matters as much as speed.

- **How to phrase it to a judge:** "Under realistic load the conventional
  stop-and-wait fleet cleared its workload once in twenty runs. FLEET-X cleared
  it nineteen times in twenty, delivered 4-7x more orders in the same period,
  and was 69.6% faster to a like-for-like delivery count — with zero collisions
  on both sides. In a quiet warehouse the two are identical, which is the proof
  the gain is genuine congestion handling and not a tuning trick."

- **A BAD BUG OF MINE, found while building this.** When I added the Phase 8/9
  resets, a search-and-replace matched in TWO places and spliced a full
  memory-wipe into the middle of `answer_requests`. Every time a robot was
  asked to step aside it erased its obstacle map, its whole job board AND its
  current job. That is why FLEET-X was dropping 1 job in 12. Lesson: when
  patching by search-and-replace, check how many places the pattern matches.

- **Also added:** `fleet_world(n)` for 3 to 20 robots (08 §3 asks for exactly
  this), so contention can be dialled up.

- **How to re-run it:**
  ```
  python3 tools/benchmark.py                       # the standard run
  python3 tools/benchmark.py --seeds 10 --robots 3 5
  ```
  232 of 232 tests pass. Purity guard clean.

---

### 2026-09-08 — Phase 9: robots bid for real jobs

- **What:** Replaced the patrol scaffolding with real work. An order becomes a
  two-leg job — collect from a shelf, deliver to packing — and the robots
  decide between themselves who does it.

  - `shared/fleetx_core/tasks.py` — NEW. The job, the board, the cost of a bid
    (02 §6 weights) and the auction.
  - `shared/fleetx_core/messages.py` — TASK_ANNOUNCE, TASK_BID, TASK_CLAIM.
  - `shared/fleetx_core/robot.py` — bids, claims, collects, delivers, and
    re-announces a job when a peer goes quiet.
  - `shared/fleetx_core/world.py` — the order book, `fail_robot`/`revive_robot`.
  - `grid_sim/scenarios.py` — `OrderGenerator`, SEEDED so the same seed always
    produces the same orders. Phase 15 depends on this.
  - `grid_sim/web/index.html` — a jobs panel, Fail/Revive buttons, diamonds for
    collection points and stars for delivery points.
  - `tests/test_phase9.py` — NEW, 29 tests.

- **Why:** Two reasons. It is what a warehouse robot actually does, and the
  benchmark measures TASK COMPLETION TIME — you cannot time a patrol, because
  a patrol never finishes. Phase 9 is what gives Phase 15 something to measure.

- **Milestone:** Phase 9 — Task allocation.

- **How to see it:** open http://localhost:8000, press **Start orders**. Each
  job is auctioned in the event feed with all three bids written out. Then
  press **Fail R1** while it is carrying something and watch the job get
  rescued by somebody else.

- **Results measured on the live server:**
  - orders flowing, 5 of 9 delivered inside 50 seconds, average 9.6s per job
  - R1 killed while carrying T-003 -> released -> re-auctioned -> R3 delivered it
  - collisions 0 throughout
  - 232 of 232 tests pass. Purity guard clean.
  - Full verification suite re-run: still 0 collisions everywhere.

- **The cheapest robot, not the nearest** (06 §1). Proven in a test: a robot
  4 squares away on 15% battery loses to one 12 squares away on 90%, because
  the near one would run flat halfway and the job would have to be done again.

- **TWO REAL BUGS FOUND:**
  1. **A switched-off robot came back to life and won an auction.** `fail_robot`
     set the status to FAILED, then the robot cleared its goal — and
     `set_goal(None)` set the status to IDLE unconditionally, wiping the
     failure. A "failed" robot then bid for a job and collected a parcel.
     Fixed: a broken robot stays broken.
  2. **A dead robot's job was never picked up.** The world released it on its
     own board but never told the robots, so all of them kept it listed as
     R1's job for ever. Fixed by broadcasting a RELEASE.

- **And one design hole I left myself:** a job whose auction winner had already
  taken a different job was never re-auctioned. Three jobs go out at once, one
  robot has the best bid on two of them, takes the first — and the second sits
  there for ever with a winning bid naming a robot that is busy. I had defined
  a claim timeout in the design and never used it. Now an unclaimed job goes
  back up for auction after 2.5 seconds.

- **Test bug found (mine):** a test used (3,6) as a collection point. That is
  inside a shelf block, so no robot could reach it and correctly nobody bid.
  The test now asserts the collection square is actually open floor.

- **Built for Phase 15:** orders come from a seed, so the same seed always
  produces the same jobs in the same order. 08_SIMULATION §5 — "Create
  identical workloads." A benchmark comparing FLEET-X against stop-and-wait is
  worthless unless both sides run exactly the same work.

---

### 2026-09-08 — Phase 8: robots find and route round obstacles

- **What:** You can drop something into an aisle with a click. The robots are
  NOT told — one has to drive close enough to see it, then it warns the rest.

  - `shared/fleetx_core/obstacles.py` — NEW. Each robot's own map of squares it
    believes are blocked, with a time-to-live so blocks fade.
  - `shared/fleetx_core/messages.py` — added BLOCKED_AISLE, which 04 §2 already
    specified (confidence and ttl included).
  - `shared/fleetx_core/astar.py` — `find_path(..., blocked=...)` routes round
    squares that are genuinely impassable.
  - `shared/fleetx_core/robot.py` — senses, marks, broadcasts, tears up any
    route that now runs through a block, and replans.
  - `shared/fleetx_core/world.py` — holds the real obstacles and plays the part
    of each robot's laser scanner.
  - `grid_sim/` — a "Drop obstacle" mode, a clear-all button, hazard-hatched
    squares, and a panel listing what the selected robot knows.
  - `tests/test_phase8.py` — NEW, 25 tests.

- **Why:** The map on file goes out of date the moment a box falls off a
  pallet. 05_PATH_PLANNING §10 spells out the whole flow, and 11_SIH_DEMO
  step 6 makes it a demo moment.

- **Milestone:** Phase 8 — Dynamic obstacles.

- **How to see it:** open http://localhost:8000, click **Drop obstacle**, then
  click a busy aisle. Watch a robot drive up, stop, and light it red — and the
  other two robots' route lines bend away in the same instant.

- **Results measured on the live server:**
  - t=2.1s box dropped, 0 of 3 robots know about it
  - t=3.3s R3 drove near, found it, told everyone — 3 of 3 know
  - distance kept climbing throughout, collisions stayed 0
  - block correctly faded once nobody was near it any more
  - 202 of 202 tests pass. Purity guard clean.
  - Full verification suite re-run: still 0 collisions everywhere, including
    a new test that scatters obstacles at random for 4 minutes.

- **The design decision that matters:** robots have to FIND obstacles. Sensor
  range is 3 squares. If every robot magically knew the instant a box landed,
  the broadcast would be pointless and none of this would be decentralised —
  it would just be a shared database. A judge is likely to ask about exactly
  this. The simulator plays the part of the LiDAR and hands each robot only
  what is physically in range; the ROS 2 version swaps in a real laser scanner
  and the brain does not change a line. Same trick as the message bus.

- **Other decisions worth remembering:**
  - Blocks EXPIRE after 20 seconds unless somebody sees them again. Boxes get
    picked up, and without this the map would slowly fill with phantom walls
    and throughput would quietly collapse.
  - A robot that can see a square is empty CLEARS its block and tells everyone,
    so recovery is faster than waiting for the timer.
  - An obstacle is impassable, NOT merely expensive. Different from a busy
    square in Phase 6: you can push through a busy aisle if you must, but you
    cannot drive through a pallet because you are in a hurry.
  - If an obstacle cuts off the only route, the robot honestly reports BLOCKED
    rather than pretending. That is what the BLOCKED state has been waiting for
    since Phase 1.
  - You cannot drop a box onto a robot or into a shelf.

- **Test bug found (mine, not the code's):** a test claimed no BLOCKED_AISLE was
  ever broadcast. It was — the test was looking in `bus.recent`, a 40-message
  ring buffer for the dashboard ticker, which position updates flush within a
  second. Fixed by checking as the messages go out.

---

### 2026-09-08 — Phase 7: robots break their own deadlocks

- **What:** A robot stuck too long now works out WHY it is stuck, and either
  asks the robot in front to move or gets out of the way itself.

  - `shared/fleetx_core/deadlock.py` — NEW. The wait-for graph (05 §8), loop
    finding, and the rule for choosing who gives way.
  - `shared/fleetx_core/messages.py` — added WAIT_REPORT ("I am stuck behind
    X") and YIELD_REQUEST ("please move"). Both are extensions to the protocol
    in 04, which stops at conflict alerts.
  - `shared/fleetx_core/robot.py` — reports being stuck, spots loops, asks for
    room, steps aside, remembers the job and resumes it.
  - `shared/fleetx_core/world.py` — recovery counters, `deadlock_enabled` switch.
  - `grid_sim/web/index.html` — a green YIELDING state, a "Jams broken" counter,
    and a live "who is waiting for whom" panel.
  - `tests/test_phase7.py` — NEW, 24 tests.

- **Why:** Anushka hammered the dashboard and hit a real deadlock: R2 and R3
  frozen over square (26,0), stuck for 365 seconds.

- **THE DEADLOCK SHE FOUND WAS NOT THE ONE THE ROADMAP DESCRIBES.**
  The roadmap's Phase 7 is about circular waits (R1 waits for R2 waits for R3
  waits for R1). Hers was simpler and more realistic:

      R3 finished its job, parked on (26,0) and went IDLE.
      R2's DESTINATION was (26,0).
      R3 was not waiting for anything, so it had no reason to ever move.

  No rerouting can help, because the blocked square IS the goal. R2's own note
  said exactly that: "R2 waits - every route goes the same way".

  So Phase 7 covers two cases, not one:
    1. a circular wait -> one robot is chosen to give up
    2. somebody parked in the way -> it gets ASKED to move
  Case 2 is not in the roadmap and is arguably the more realistic warehouse
  problem. A parked robot is a courteous obstacle, not a wall.

- **Milestone:** Phase 7 — Deadlock.

- **How to see it:** open http://localhost:8000, send R3 somewhere and let it
  park. Then send R2 to that same square. R2 waits about 4 seconds, then R3
  turns green (YIELDING) and shuffles aside.

- **Results measured (not guessed):**
  - Her exact deadlock, on the live server: frozen 365s before, now cleared in
    4 seconds (asked at 13.65s, moved at 13.70s, arrived at 14.8s)
  - Head-on x10 and Intersection x10: 0 collisions, 0 deadlocked
  - Free roam 5 min: 0 collisions, 0 deadlocked, distance 2055
  - Free roam + clicking, 6 x 5 min: 0 collisions, 0 deadlocked
  - Scenario switching, 6 seeds: 0 collisions, 0 deadlocked
  - 50% packet loss: 0 collisions (was 1 in Phase 6 — the extra caution helped)
  - 177 of 177 tests pass. Purity guard clean.

- **THREE MORE BUGS my own tests caught while building this:**
  1. **The loop detector never looked at the robot's own edge.** The wait-for
     map is built from what OTHER robots say, so a robot searching for a loop
     containing itself started from nothing. A two-robot standoff was invisible.
  2. **A boxed-in robot was asked to move 37 times and could not.** Both its
     exits were blocked. Now it PASSES THE REQUEST ALONG to whoever is blocking
     it, so the request cascades until somebody has room.
  3. **The "jams broken" counter lied.** It counted every request, reporting 37
     recoveries for one jam that never cleared. Now it only counts when a robot
     actually moves.

- **And one real inefficiency fixed:** both robots kept giving way to each
  other — detour, meet again, detour again, forever. Phase 6 was meant to have
  THE LOSER reroute and I never gated it on actually losing. Fixed with the
  same yields_to rule. That test went from "never arrives in 60 seconds" to
  "both arrive in 5 seconds".

- **Design notes:**
  - Robots remember the job they were doing and resume it after stepping
    aside. Breaking a jam by wandering off and forgetting the job would just
    swap one problem for another.
  - Choosing who gives way: whoever has waited LEAST, so the robot stuck
    longest goes first. Compared in whole points, settled by the higher name
    when close.
  - A disagreement here is harmless — two robots both moving aside is untidy,
    not dangerous — so this rule is deliberately simpler than the one that
    decides who enters a square.
  - `World(deadlock_enabled=False)` brings back the old stuck-forever
    behaviour, and the Phase 5/6 jam tests use it.

---

### 2026-09-08 — Collision hunt: found 8 bugs, back to a real ZERO

- **What:** Anushka spotted 2 collisions in Phase 6 where Phase 5 had a clean 0.
  Investigated properly, found **eight** separate bugs, fixed all of them, and
  built a stress harness that hammers the system the way a person does.
  No test threshold was loosened and no counter was hidden.

  New file `tests/test_collision_free.py` (13 tests). It measures the actual
  GAP between robots every single tick, not just the collision counter —
  because checking the counter alone would pass if the counter were broken.

- **Why it was only showing up for Anushka and not in my runs:** she was
  clicking robots to new destinations during the run. My tests only ran the
  fixed patrol. Changing a robot's destination mid-move is its own code path,
  and it was broken.

- **The eight bugs:**

  1. **Robots disagreed about who had won a square.** THE big one, and the same
     class of bug as in Phase 6. R2's score sat exactly on a band boundary; R2
     read itself as band 6 while R3's slightly older copy read it as band 5.
     Both concluded they had won and drove into each other. Rounding to whole
     points had only moved the knife-edge, not removed it.
     Fixed with a **dominance margin**: priority only decides a contest if one
     robot leads by 2 whole points. Below that, the lower robot ID wins — and
     names cannot be stale or disagreed about. There is a test that checks all
     pairs of scores from 4.0 to 12.0 and asserts the rule never says
     "both go" or "both stop".

  2. **Positions were always out of date.** Positions arrive by radio, so the
     last one heard is ~0.15s old — nearly four tenths of a square at full
     speed. Enough to walk straight through a safety check. Fixed by carrying
     each robot's last known position forward along its heading (dead
     reckoning, the way sailors do it).

  3. **Changing destination mid-move teleported the robot.** It snapped up to
     most of a square backwards, straight through the safety checks. Fixed: it
     now finishes the step it has begun, then plans afresh from there.

  4. **Robots set off before hearing about each other.** Two robots two squares
     apart, both aiming at the gap between them, start in the very same instant
     — before either has heard the other's plan. Waiting for proof means
     waiting for a message that has not arrived. Fixed by being **cautious when
     in doubt**: anything sitting next to the square I am entering is treated
     as a contest unless I have recent news that it is going elsewhere.

  5. **Robots had no brakes.** "Always finish the segment you started" meant a
     hazard spotted mid-move could not stop anyone. Fixed by adding an
     EMERGENCY STOP that halts the robot dead, even mid-aisle.
     05_PATH_PLANNING §12: "the safety controller takes precedence over fleet
     optimisation." An ordinary hold (just refused a booking, nothing
     dangerous) still stops tidily on a square.

  6. **A stopped robot reported full speed.** `velocity` meant "how fast could
     I go", not "how fast am I going". Everyone then dead-reckoned stopped
     robots creeping forward, and two stopped robots blocked each other
     for ever. This one was caused by fix 2 and caught by the test suite.

  7. **Restarting the clock left stale bookings behind.** Pressing a scenario
     button winds the clock back to zero, but bookings hold absolute times, so
     survivors claimed squares from the middle of next week and the warehouse
     gummed up solid (distance fell to 16 squares). Fixed: restarting the clock
     now clears every booking.

  8. **Staging a scenario could drop one robot on top of another.** Demo setup,
     not robot behaviour, but it made the closest-gap reading meaningless.
     Fixed: the floor is cleared first, and staging refuses to stack robots.

- **The lesson worth keeping:** safety must NOT depend on the robots agreeing.
  They hold different, slightly older copies of the same information and will
  sometimes disagree — that is what decentralised means. So the last line of
  defence is a local reflex (is the space ahead clear?) plus a rule decided by
  robot NAMES, which cannot go stale. The booking table is for efficiency;
  the reflex is for safety.

- **Results measured — collisions AND the closest two robots ever came:**

      A. head-on scenario,      10 runs   collisions 0   closest 1.000
      B. intersection scenario, 10 runs   collisions 0   closest 1.000
      C. free roam, 5 minutes             collisions 0   closest 1.000  distance 2021
      D. free roam + clicking, 6 x 5 min  collisions 0   closest 1.000  distance ~2100
      E. scenario switching, 6 seeds      collisions 0   closest 0.875
      F. live server, 3 rounds of all
         three scenarios                  collisions 0   closest 1.000
      G. LIVE SERVER, free roam 4.5 min   collisions 0   closest 1.000  distance 1836
         (the real server the browser talks to, not a headless test:
          20 reroutes, 0 deadlocks, robots never came closer than a
          full square apart at any point in the run)

  Robots must stay more than 0.7 squares apart to count as not touching.
  153 of 153 tests pass. Purity guard clean. No throughput lost — distance is
  the same as before the fixes.

- **Known limit, unchanged and still honest:** at 50% packet loss, 1 collision
  in 3 minutes. Robots cannot respect bookings they never heard. Clean at 0%,
  10% and 25%. Worth saying out loud to a judge rather than being caught by it.

---

### 2026-09-08 — Phase 6: robots give way — WAREHOUSE WORKS NOW

- **What:** The robot that loses a square now **goes around** instead of just
  sitting there. Plus a real priority score with a fairness rule.

  - `shared/fleetx_core/priority.py` — NEW. The score (04 §6) and the aging
    rule (05 §9): the longer you wait, the more you are owed. Also `bucket()`,
    which is what makes robots agree — see the bug notes below.
  - `shared/fleetx_core/reservations.py` — added `avoidance_cost`, a routing
    rule that makes booked squares EXPENSIVE, never forbidden. Like a maps app
    with traffic: a jammed road is not closed, just slow. Banning them would
    make a robot in a one-way corridor report BLOCKED, which is worse.
  - `shared/fleetx_core/robot.py` — the wait-or-reroute decision, a proximity
    safety reflex, and a live priority.
  - `shared/fleetx_core/world.py` — reroute counters, a decision log, and a
    `negotiation_enabled` switch that brings Phase 5 behaviour back.
  - `grid_sim/web/index.html` — priority on the cards (visibly climbing),
    REROUTING in violet, a "who gave way" panel in plain words.
  - `tests/test_phase6.py` — NEW, 24 tests.

- **Why:** Phase 5 got collisions to zero but jammed the warehouse solid —
  distance fell from 893 to 44. Zero crashes and zero work done is not a
  product. Phase 6 is what makes the speed half of the goal possible.

- **Milestone:** Phase 6 — Negotiation.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Open http://localhost:8000, press **Head-on**. One robot stops, its priority
  ticks up 5.0 → 5.6 → 6.2, then it turns violet (REROUTING) and goes around.

- **Results measured (not guessed) — 120s free roam, same map, same jobs:**

      setting                    collisions  distance  deadlocked  reroutes
      blind (Phase 2-4)              14         893         0          0
      booking only (Phase 5)          0          44         3          0
      booking + giving way (6)        0         830         0          7

  93% of the distance recovered with zero collisions.
  Scenarios: head-on 0 collisions / 0 deadlocks, intersection the same.
  139 of 139 tests pass. Purity guard clean.

- **THREE REAL BUGS FOUND — the third one matters most:**

  1. **Priority thrash.** The moment a waiting robot gained priority it won the
     square, stopped waiting, INSTANTLY lost the priority, and the other robot
     won again. They swapped roles every tick and crept into each other. Fixed
     by making the waiting credit fade slowly instead of resetting.

  2. **The stop-gate only bit at a square boundary.** Once a robot was mid-move
     it could not be held, so two robots could commit to the same square from
     opposite ends.

  3. **ROBOTS DISAGREED ABOUT WHO WON.** Found in a trace: priorities were
     flickering by a TENTH of a point, and at one instant R1's table said R1
     owned the square while R2's table said R2 did. Both believed they had won,
     and drove into each other.

     The cause is fundamental, not a typo: every robot holds a slightly
     different, slightly older copy of the table, so its numbers are never
     identical to anybody else's. If a tenth of a point can decide ownership,
     two robots reading the same situation reach opposite conclusions.

     Fix: compare priorities in WHOLE POINTS and fall back to the lower robot
     ID. Small differences of opinion then cannot change the answer. This is
     the single most important safety property in the system. There is a test
     that feeds four slightly different views of one situation and asserts they
     all agree.

- **The real last line of defence is not the booking table.** It is a proximity
  reflex: never drive into a space that already has a robot in it or entering
  it. Booking is decided in an instant but driving takes time, so measuring the
  actual gap is what closes the final hole. On a real robot this is the LiDAR
  safety layer from 03 §8, which needs no messages at all — which is exactly
  why it is the right last line of defence.

- **Other decisions worth remembering:**
  - A robot waits 1.5s before even considering a detour. Most hold-ups clear on
    their own, and a robot that recalculated 20 times a second would flap.
  - It refuses a detour that is more than ~2.5x the remaining route. Going 40
    squares to save 2 is silly.
  - `World(negotiation_enabled=False)` brings Phase 5 behaviour back, and the
    Phase 5 deadlock tests now use it.

- **Not done yet:** deadlocks reached zero in these runs, but there is still no
  actual cycle DETECTION. A circular jam where every alternative is also
  blocked would still hang. That is Phase 7.

---

### 2026-09-08 — Phase 5: robots book squares — COLLISIONS NOW ZERO

- **What:** Robots can now claim a square before driving onto it, with one hard
  rule: **a robot may not enter a square it has not booked.** If it cannot book
  the square ahead, it stops before it and waits.

  - `shared/fleetx_core/reservations.py` — NEW. The booking table. Every robot
    keeps its OWN copy, built from radio messages. No server — 02_ARCHITECTURE
    §3 says the backend must not be the only thing preventing a collision.
  - `shared/fleetx_core/messages.py` — added PATH_RESERVATION (04 §2).
  - `shared/fleetx_core/robot.py` — books the next 3 squares, releases them
    behind it, and stops short when refused.
  - `shared/fleetx_core/world.py` — counts waiting time and deadlocks, and got
    a `reservations_enabled` switch that restores the old blind behaviour.
  - `grid_sim/web/index.html` — booked squares tinted in the owner's colour, a
    live reservation table "as R1 sees it", amber bar on stopped robots.
  - `tests/test_phase5.py` — NEW, 27 tests.

- **Why:** This is the phase that delivers the project's headline number.
  00_README: "0 inter-robot collisions".

- **Milestone:** Phase 5 — Reservation.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Open http://localhost:8000, press **Intersection**. Three robots converge,
  one takes the junction, the other two stop dead. No red burst.

- **Results measured (not guessed):**
  - Intersection: 0 collisions (was 3)
  - Head-on: 0 collisions (was 1)
  - Free roam 120s: 0 collisions (was 11)
  - Verified every tick for 600 ticks that no two robots share a square
  - 114 of 114 tests pass
  - Purity guard: still clean

- **THE HONEST BAD NEWS — the warehouse now jams solid.**

      booking OFF ->  7 collisions,  446 squares travelled, 0 deadlocks
      booking ON  ->  0 collisions,   44 squares travelled, 3 deadlocked

  All three robots end up waiting for each other forever. Zero crashes and
  zero work done. This is expected: I designed the patrol routes in Phase 2 so
  R1 and R2 meet head-on, which was great for forcing crashes and is now a
  guaranteed deadlock. Phase 6 (loser reroutes instead of waiting) and Phase 7
  (spot the circular wait and force someone to yield) are what fix it. There
  are tests that ASSERT deadlocks appear, so nobody can quietly hide them.

- **Two real bugs found and fixed:**
  1. **A robot drove onto a square another robot was parked on.** R2 correctly
     stopped and waited — then R1, having a lower ID, won the tie-break for
     that same square and drove into it. Occupancy was being treated as a
     preference you could be outranked out of. Fixed two ways: a robot claims
     the square it is standing on at OCCUPANCY_PRIORITY which nothing beats,
     PLUS a local safety check that refuses to enter an occupied square no
     matter what the table says — the LiDAR reflex from 03 §8.
  2. **A failed robot kept its bookings forever.** It stopped moving but never
     released its squares, so the fleet would route around holes with nothing
     in them. 03 §5 says to release a dead robot's reservations. Now it does.

- **Design decisions worth remembering:**
  - Ownership is worked out fresh from all claims on file, never decided as
    claims arrive. That makes it ORDER-INDEPENDENT: robots reach the same
    answer even when messages arrive in different orders. There is a test that
    tries all six orderings of three claims and asserts one answer.
  - Tie-break: higher priority wins, then lower robot ID (04 §7).
  - Robots book only 3 squares ahead. Booking a whole route would let one robot
    own half the warehouse.
  - An aisle segment is the same resource in both directions, which is what
    stops a head-on.
  - A waiting robot always stops exactly ON a square, never halfway down an
    aisle. It finishes the segment it started.
  - `World(reservations_enabled=False)` brings back the blind behaviour. The
    Phase 2/3/4 tests now use it, and Phase 15 will use the same switch to run
    the "before" side of the benchmark on identical code.

- **Known limitation:** at 50% packet loss, 3 collisions came back. Robots
  cannot respect bookings they never heard. Worth knowing before a judge asks.

---

### 2026-09-08 — Phase 4: robots predict their own crashes

- **What:** Every robot now compares its own plan against everyone else's, several
  times a second, and announces "we are going to want the same square at the
  same time".

  - `shared/fleetx_core/conflicts.py` — NEW. The time-slot idea. A robot does
    not touch a square at an instant, it ARRIVES and later CLEARS. So its plan
    becomes bookings: "R1 holds (13,8) from 3.0s to 3.6s". Two overlapping
    bookings on one square is a conflict. Detects all four things the roadmap
    asks for: same square, same edge (head-on), junction, time overlap.
  - `shared/fleetx_core/messages.py` — added CONFLICT_ALERT, spec'd in 04 §2.
  - `shared/fleetx_core/grid.py` — added `is_junction()` (3+ ways out).
  - `shared/fleetx_core/robot.py` — each robot builds its own bookings, builds
    what it BELIEVES everyone else's are (from its notebook), compares them, and
    broadcasts alerts. Rate limited so it doesn't flood the radio.
  - `shared/fleetx_core/world.py` — tracks whether each crash was called in
    advance and by how long.
  - `grid_sim/web/index.html` — contested squares pulse amber with a live
    countdown, a dashed line joins the two robots about to clash, a conflict
    list panel, and crashes are logged as "(called 4.6s early)".
  - `tests/test_phase4.py` — NEW, 25 tests.

- **Why:** Until now the crash warning you saw when hovering a square was being
  worked out by the DASHBOARD, for a human. Phase 4 moves that thinking into
  the robots themselves. It is the step that makes Phase 5 (booking squares)
  and Phase 6 (deciding who yields) possible — you cannot resolve a conflict
  you never noticed.

- **Milestone:** Phase 4 — Conflict detection.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Open http://localhost:8000 and press **Intersection**. Amber squares appear
  with countdowns, tick down 3.0 → 2.0 → 1.0, then three red bursts on exactly
  those squares.

- **Results measured (not guessed) — 60 seconds of free roam:**
  - collisions: 8
  - predicted in advance: 8
  - missed completely: 0
  - average warning: 4.53 seconds
  - false alarms (warnings that resolved without a crash): 10
  - Intersection scenario: 3/3 predicted, ~3.0s warning each
  - 87 of 87 tests pass (24 + 18 + 20 + 25)
  - Purity guard: still clean

- **Mistake worth recording — my own metric lied.** The first version reported
  "average warning 0.07s" when alerts were clearly firing 3 seconds early. Cause:
  a live conflict COUNTS DOWN as the robots close in, and I was storing the
  latest countdown value, so 2.9s got overwritten with 0.05s before the crash
  landed. It would have made Phase 5 look pointless. Fixed by recording when the
  alarm FIRST went up and never overwriting it. There is now a test that fails
  if warning times collapse toward zero again.

- **Design decisions worth remembering:**
  - Looking 8 seconds ahead. Longer gives constant noise from plans that change
    before they matter.
  - A small safety gap either side of each booking, because robots have bodies.
    You do not book a meeting room ending at 3.6 and the next starting at 3.6.
  - A parked robot holds its square for the whole horizon — driving into a
    stationary robot is just as much a crash.
  - Late information is shifted earlier: a message that took 0.3s to arrive has
    ETAs that are 0.3s out of date.
  - Detection is honestly decentralised. Each robot uses only its own notebook,
    so a robot it cannot hear is invisible to it. There is a test proving R1
    stops seeing conflicts with a silenced R2. Two robots can disagree about
    whether there is a problem — which is exactly why Phase 6 needs real
    negotiation rather than a simple rule.

- **Deliberately NOT fixed:** nothing yields. Robots announce the crash and
  then have it. There is a test that FAILS if collisions ever reach zero here.

- **The booking table built in this phase IS Phase 5's reservation system.**
  It already exists; robots just cannot claim slots in it yet.

---

### 2026-09-07 — Phase 3: the robots can hear each other

- **What:** Gave every robot a voice and ears. Three new files in the shared
  brain, plus radio controls on the dashboard.

  - `shared/fleetx_core/messages.py` — NEW. The three things robots say:
    **Heartbeat** ("I'm alive, battery 84%", twice a second), **PoseUpdate**
    ("I'm at 13,8", ten times a second), and **IntentUpdate** ("I'm going to the
    drop station, I'll be on square 13,8 in 3.2 seconds"). Field names copied
    from 03_ROBOT_AND_ROS2 §2 so the ROS 2 version is a translation, not a
    rewrite. Every message carries a timestamp and a sequence number.

  - `shared/fleetx_core/bus.py` — NEW. The "socket in the wall". The brain says
    publish() and poll() and never asks how the message travelled. The grid
    simulator plugs in an in-memory bus; the teammate plugs in ROS 2/DDS. Same
    brain, two plugs. The in-memory one can drop and delay messages on purpose.

  - `shared/fleetx_core/fleet_view.py` — NEW. Each robot's private notebook on
    the others. Tracks what they said, how long ago, how many messages went
    missing, and whether they have gone quiet (stale). Has `who_is_heading_for`,
    which answers "which robots say they will be on this square, and when" —
    that is exactly what Phase 4 needs.

  - `shared/fleetx_core/robot.py` — robots now publish and listen each tick.
  - `shared/fleetx_core/world.py` — holds the bus, ticks comms before thinking.
  - `grid_sim/server.py` — silence a robot, adjust packet loss and delay.
  - `grid_sim/web/index.html` — ghost circles showing what the SELECTED robot
    believes about the others, a hover readout of who is coming to a square and
    when, a live radio traffic ticker, silence buttons and quality sliders.
  - `tests/test_phase3.py` — NEW, 20 tests.

- **Why:** This is the actual innovation. 11_SIH_DEMO §5 lists "intent-aware
  coordination" as innovation number one. 04_PROTOCOL §3 puts it best:

      Position-only:  "R1 is here."
      Intent-aware:   "R1 is here. R1 wants to go there. R1 expects to occupy
                       these squares. R1 expects to arrive at this time."

  The first only lets you react after trouble starts. The second lets you see
  trouble coming. Everything in Phases 4-7 is just reading that second block.

- **Milestone:** Phase 3 — Communication. The roadmap's success test is
  "R1 can see R2/R3 intent" — met, and there is a test named after it.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Open http://localhost:8000.
  - Select each robot — the faint ghosts are that robot's beliefs about the others
  - Hover the junction (13,8) — it tells you who is coming and in how many seconds
  - Click a robot in the Radio panel to cut its transmitter; after 2 seconds the
    others mark it STALE
  - Drag the packet loss slider to 70% and watch them cope

- **Results measured (not guessed):**
  - R1 correctly knows R2 and R3's destination, planned squares and per-square ETAs
  - Silencing R2: both other robots marked it stale at 2.25s (2s timeout, correct)
  - Un-silencing: recovered within 1s
  - 70% packet loss + 120ms delay: 310 messages dropped, 94 losses detected by
    sequence gaps, and all 3 robots kept working
  - 80% packet loss (in tests): both robots still reached their goals
  - 62 of 62 tests pass (24 + 18 + 20)
  - Purity guard: still clean

- **Deliberately NOT fixed:** collisions still happen. The robots hear each
  other and do nothing about it. There is a test that FAILS if collisions ever
  reach zero in this phase, to catch avoidance sneaking in early and quietly
  ruining the before/after benchmark.

- **Details that matter for real hardware:**
  - Sequence numbers let a robot prove a message went missing, not just that
    one is late.
  - Out-of-order messages are rejected: a delayed message overtaking a newer one
    must not rewind the picture. There is a test for exactly this.
  - Intent is only sent when the plan CHANGES, not 20 times a second. Repeating
    "still going the same way" would flood real warehouse Wi-Fi.
  - The whole thing is tested with messages being lost, because 04_PROTOCOL §8
    says "The protocol must never assume every message arrives."

---

### 2026-09-07 — Phase 2: three robots, and an honest collision counter

- **What:** Went from one robot to three, with NO coordination between them —
  on purpose. Also rebuilt the collision counter so it tells the truth.

  - `shared/fleetx_core/world.py` — rewrote how crashes are detected. Added
    `phase2_world()` (R1, R2, R3 spread across the map) and `reset_counters()`.
  - `shared/fleetx_core/robot.py` — added `place()`, which drops a robot onto a
    square instantly. Only used to set up demo scenarios.
  - `grid_sim/scenarios.py` — NEW. The patrol loop that keeps robots busy, plus
    the demo setups: head-on, intersection, free roam, stop. This is demo
    scaffolding so it lives OUTSIDE the brain, keeping the brain pure.
  - `grid_sim/server.py` — serves three robots, plus two new buttons'
    worth of actions: pick a scenario, reset the counter.
  - `grid_sim/web/index.html` — three robots in three colours, three status
    cards, click a card to choose which robot your clicks control, red bursts
    where crashes happen, collision counter flashes red, crashes written into
    the event feed. Red warning banner explaining this is broken on purpose.
  - `tests/test_phase2.py` — NEW, 18 tests.

- **Why:** Two reasons.

  1. **To show the problem.** You cannot prove you fixed something you never
     showed was broken. Now there is a screen where three robots visibly drive
     through each other and a counter reading COLLISIONS: 6. Phases 3-7 have to
     drive that to 0 on the same map with the same jobs. That is the headline
     result. 11_SIH_DEMO §7: "Never show a feature without a measurable result."
  2. **Plumbing.** Nothing about conflicts, priority or deadlock can be built or
     tested with one robot.

- **Milestone:** Phase 2 — Three robots.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Open http://localhost:8000. Watch free roam for 30 seconds and count the red
  bursts. Then press "Intersection" — all three robots meet at (13, 8) at the
  same instant and pass straight through each other.

- **Results measured (not guessed):**
  - Head-on scenario: 1 collision
  - Intersection scenario: 3 collisions, all three pairs, all at t=3.0s
  - Free roam for 45 seconds: 6 collisions, about one every 7 seconds
  - 42 of 42 tests pass (24 from Phase 1, 18 new)
  - Purity guard: still clean

- **The important bit — the collision counter had two bugs that would have
  ruined the benchmark:**
  - **It over-counted.** The world ticks 20 times a second, so two robots
    overlapping for one second would have been counted as 20 separate crashes.
    Fixed: one crash now counts once. Verified live — the counter hit 3 at the
    junction and stayed at 3.
  - **It under-counted.** It only looked for two robots on the SAME square. Two
    robots swapping places drive straight through each other and never share a
    square, so the most obvious crash of all was invisible.
    05_PATH_PLANNING §5 warns about exactly this: "Do not check only nodes."

  There are now three crash rules — same square, head-on swap, and bodies
  touching — each with its own test. The swap rule is tested with a forced
  one-tick swap so it cannot quietly become dead code.

- **Mistake worth recording:** the Intersection scenario did not work first
  time. R3 started 7 squares from the junction while R1 was 9 away, so R3 sailed
  through before the others arrived — 1 collision instead of 3. Fixed by putting
  all three exactly 8 squares out so they arrive together.

- **Worth being clear about:** this is NOT the baseline for the 20% speed
  target. That baseline is "stop-and-wait" and gets built in Phase 15. Phase 2
  is the problem picture only.

- **Not built yet:** the robots still cannot see, hear or talk to each other at
  all. That starts in Phase 3.

---

### 2026-09-07 — Phase 1 built and running: one robot moves from A to B

- **What:** Built the first working version. Three new folders:

  **`shared/fleetx_core/` — the shared brain (pure logic, no web, no ROS 2)**
  - `grid.py` — the warehouse floor as a grid of squares, 28 wide by 16 tall.
    Twelve shelf blocks, aisles between them, pick/drop/charger stations.
  - `astar.py` — the route finder. Like a maps app: finds the shortest way
    around the shelves. Has a hook for making busy aisles cost more later.
  - `robot.py` — one robot's brain. Knows where it is, plans a route, drives
    along it, tracks battery and distance.
  - `world.py` — holds the map and every robot, moves time forward, counts
    collisions and works out the dashboard numbers.

  **`grid_sim/` — Part 1, runs on the Mac**
  - `run.py` — one command to start everything.
  - `server.py` — the web server. ALL web code lives here, never in the brain.
  - `web/index.html` — the dashboard: warehouse map, live robot, KPI strip,
    robot card, event feed. Click a square to send the robot there.

  **`tools/` and `tests/`**
  - `tools/check_purity.py` — guard script. Fails loudly if anyone ever puts
    web or ROS 2 code inside the shared brain.
  - `tests/test_phase1.py` — 24 automated tests.

- **Why:** Roadmap Phase 1 asks for exactly one thing — "R1 can move from A to
  B". Doing it on a simple grid first means bugs are visible instantly: if the
  robot walks through a shelf, you can see it on screen. Everything later
  (conflicts, bookings, deadlock) hangs off this same grid.

- **Milestone:** Phase 1 — Warehouse simulation. Phase 0 also closed out.

- **How to see it:**
  ```
  python3 grid_sim/run.py
  ```
  Then open http://localhost:8000. Click any dark floor square to send the
  robot there. Click a grey shelf and it refuses. Stop it with Ctrl+C.

  Run the tests:  `python3 -m unittest discover -s tests`
  Check the brain is still pure:  `python3 tools/check_purity.py`

- **Results measured (not guessed):**
  - 24 of 24 tests pass
  - 0 squares of a full corner-to-corner route went through a shelf
  - 0 illegal jumps (robot always moves one square at a time)
  - 0 collisions
  - Live feed confirmed running at 20 updates per second
  - Purity guard: clean, no web or ROS 2 code in `shared/fleetx_core/`

- **Decisions worth remembering:**
  - Robots move up/down/left/right only, never diagonally. This keeps the map
    a clean set of nodes and edges, which is exactly what the reservation
    system in Phase 5 needs.
  - The robot brain is split three ways so it can be shared: `decide()` is the
    thinking and BOTH versions use it; `advance()` is fake driving and only the
    grid simulator uses it; `update_pose()` is a real robot reporting its true
    position and only the ROS 2 version will use it.
  - The dashboard is plain HTML and JavaScript, not React/Tailwind as
    `07_DASHBOARD_AND_DIGITAL_TWIN.md` suggests, because React needs installing
    and Anushka's rule is zero installs. Swappable later; nothing else changes.
  - Tests use `unittest` (built into Python) instead of `pytest` from
    `08_SIMULATION_AND_TESTING.md`, for the same zero-install reason.

- **Housekeeping:** Moved the two private deck files into `private/` and added
  `private/` to `.git/info/exclude` (NOT `.gitignore`). Confirmed git does not
  see them. Nothing committed. Nothing pushed.

- **Not built yet:** only one robot, and it has no idea other robots exist.
  Talking, conflicts, bookings and negotiation all start from Phase 2 onward.

---

### 2026-09-07 — Read the plan, set up this file

- **What:** Read `MY_NOTES.md` and all 13 project markdown files
  (`00_README.md` through `12_TEAM_TASK_BREAKDOWN.md`). Created this
  `PROGRESS_NOTES.md`. Added a permanent rule to `MY_NOTES.md` saying this file
  must be updated on every change.
- **Why:** Anushka is new to coding and has teammates. We need one plain-English
  place that says what exists so far and what is still missing, so nobody has to
  read code to find out.
- **Milestone:** Phase 0 — Team setup.
- **How to see it:** Open this file. Nothing runs yet — no code has been written.
- **Note:** No code built yet. Waiting for the go-ahead on which phase to start.
