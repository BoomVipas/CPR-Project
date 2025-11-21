# CPR Grid Simulation

A standard-library Python simulation of two 10-robot teams competing to retrieve gold on a 20×20 grid. The code is organized so every major concept from the course—task scheduling, consensus, logical clocks, and message passing—has a clear landing spot, while the current behaviour remains easy to follow from the console.

---

## Repository Layout

```
.
├─ main.py               # entry point (delegates to CLI)
├─ README.md             # what you are reading
├─ simulation_log.txt    # detailed log output from the last run
└─ cpr_sim/
   ├─ __init__.py
   ├─ cli.py             # argparse wrapper → Config → Simulation.run()
   ├─ sim.py             # master simulation loop and rule resolution
   ├─ agents/
   │  └─ robot.py        # behaviour of a single robot (planning, movement, comms)
   ├─ core/
   │  ├─ bus.py          # message bus with Lamport stamps
   │  ├─ clocks.py       # Lamport clock implementation
   │  ├─ logger.py       # structured logging (per tick + summary)
   │  ├─ scheduler.py    # EDF scheduler used by the robots
   │  └─ consensus/
   │     └─ paxos.py     # minimal single-decree Paxos roles (proposer/acceptor)
   └─ world/
      ├─ grid.py         # grid generation, deposits, gold placement
      └─ render.py       # ASCII renderer for the grid state
```

*(All backup files, such as `sim.py.bak`, have been removed; every remaining file is used by the simulation.)*

---

## Robot Lifecycle (`cpr_sim/agents/robot.py`)

Each robot maintains its own state machine driven by an EDF scheduler:

1. **Knowledge & clocks**
   * Lamport clock (`LamportClock`) timestamps all outgoing messages.
   * `known_team_positions` keeps the freshest known `(pos, last_seen_tick)` per teammate; entries expire after 5 ticks without an update.
   * `knowledge_base` stores what the robot has seen (gold amount, robots present) to drive exploration.
2. **Task planning (`plan`)**
   * Always ensures a `sense` task is queued within the next 3 ticks.
   * If carrying gold, schedules a `to_deposit` task immediately.
   * Otherwise, always ensures there is an `explore` task pending—even if the robot recently waited for help. This guarantees the robot eventually leaves a cell if consensus or help fails to materialise.
   * When standing on gold, queues `pair_consensus` and `coordinate` tasks as needed.
3. **Exploration (`step_explore`)**
   * Priority 0: if standing on gold, attempt pickup, requesting help and waiting briefly for a partner.
   * Priority 1: if assigned as a `SUPPORTER`, head toward the specific gold cell requested.
   * Priority 2: head toward the nearest known gold location.
   * Priority 3: biased exploration (prefer unseen neighbours, then random forward/rotations).
   * A watchdog counts consecutive ticks spent on the same gold cell. After 6 inert ticks it clears pending help flags and drops the stale target so the planner will resume exploration.
4. **Deposit movement (`step_to_deposit`)**
   * Greedy move toward the home deposit. Rotates first if facing the wrong way; only moves once aligned.
5. **Message digestion (`digest_inbox`)**
   * Beacons update teammate knowledge.
   * `help_request` adds the gold location to the goal list and may switch the robot to `SUPPORTER`.
   * Paxos messages are buffered for later processing.
6. **Consensus integration (`integrate_promises`)**
   * Groups Paxos promises per cell and, once a quorum of fresh peers respond, decides on the carrying pair.
   * Consensus state is garbage-collected if it has been idle for 15 ticks.

### Communication primitives

* `maybe_beacon`: every 3 ticks a robot broadcasts its position, facing, role, and current gold intelligence to teammates.
* `request_help`: sent to the closest known teammate when a robot finds gold but has no partner.
* `pair_consensus`: Paxos prepare requests to all fresh peers on the gold cell (see below).
* All messages are timestamped with an increasing Lamport clock so the bus can enforce causal delivery if needed.

---

## Team Communication & Consensus

### Gossip & help

1. **Beacon phase** — Each robot publishes a `pos` message containing its state and gold sightings. Recipients update their `known_team_positions` with the send time, allowing the freshness filter to expire stale entries.
2. **Help requests** — When a lone robot discovers gold, it asks the closest teammate it still considers fresh. Helpers switch to `SUPPORTER` and head for the rendezvous.

### Paxos pairing

Paxos is used to choose the exact pair when more than two candidates gather on the same gold cell, preventing thrashing:

1. **Peer discovery** — `peers_on_cell` collects teammates whose **fresh** last-known position matches the cell, plus the robot itself if physically present. Entries older than five ticks are ignored.
2. **Prepare/Promise** — The minimum-ID robot among those fresh peers becomes the proposer for that round. It broadcasts `paxos_prepare(n)` where `n` encodes a unique ballot. Acceptors (every robot) respond with `paxos_promise`, relaying any previously accepted proposal.
3. **Decision** — After gathering a quorum of promises (majority of active peers), the proposer chooses the highest-numbered previously accepted pair or its own suggested pair. That value is committed locally and announced to teammates. (Because we only need knowledge of the decided pair, we stop after promises instead of running a full accept phase.)
4. **Garbage collection** — Consensus entries reset if the local robot has not been on the cell recently or if no progress has been made for 15 ticks.

This directly addresses the freeze observed earlier: stale teammates no longer count toward the quorum, and the watchdog resets robots that wait too long on a cell.

---

## Robot POV: Discovery → Help → Consensus

1. **Spot the gold** – While executing `step_explore`, a robot sees that its current cell holds gold. The next `sense` task refreshes teammate sightings (with Lamport timestamps) and confirms who is nearby.
2. **Ask for backup** – From the fresh subset of `known_team_positions`, it picks the closest teammate and sends a Lamport-stamped `help_request` naming the gold cell. `_help_requested` and `_help_wait_counter` flags track the waiting period.
3. **Hold position** – For up to six ticks it rotates in place, watching for teammates whose fresh last-seen position matches the same cell. If no partner arrives before the watchdog fires, it clears the request and immediately requeues exploration, guaranteeing it won’t freeze.
4. **Start consensus** – Whenever it stands on gold, `pair_consensus` runs. `peers_on_cell` gathers only those teammates seen within the freshness window. If there are at least two, the lowest-ID robot proposes a pair and broadcasts `paxos_prepare` to the fresh peers.
5. **Reach agreement** – Robots reply with `paxos_promise`; once a majority of fresh peers agree, `integrate_promises` records the decided pair. Robots not in the pair continue exploring; those chosen switch to `TRANSPORTER`, announce coordinated pickup intents, and move toward deposit.

Freshness filtering and consensus garbage collection ensure only robots physically present participate, while the watchdog lets anyone resume exploring if coordination stalls.

---

## Simulation Loop (`cpr_sim/sim.py`)

Each tick performs the following phases:

1. **Message delivery** — Both team buses deliver any pending messages into robot inboxes.
2. **Planning** — Robots schedule future tasks (`sense`, `explore`, `to_deposit`, `pair_consensus`, `coordinate`).
3. **Communication processing** — Ready `sense` tasks fire first; robots process inboxes, handle Paxos promises, send beacons, and potentially request help.
4. **Consensus phase** — Robots with `pair_consensus` tasks run Paxos prepare logic for their current gold cell.
5. **Promise integration** — Paxos promises collected during the phase above are integrated, potentially deciding a carrying pair.
6. **Coordinate phase** — Robots record their intent to pick up gold when instructed by consensus.
7. **Movement** — `explore` and `to_deposit` tasks update positions. Carrying pairs move in lockstep via `coordinate_pair_movement`; attempts that would desynchronise revert to their original positions.
8. **Pickup resolution** — Applies the exact-two and 2+2 rules, forming `CarryPair` instances and triggering score adjustments upon deposit.
9. **Logging & rendering** — The logger records state, messages, and consensus snapshots; optional animation clears the console and redraws the grid.

The CLI (`cpr_sim/cli.py`) exposes runtime controls like `--ticks`, `--gold`, `--sleep-sec`, and `--animate`.

---

## Logging & Observability (`cpr_sim/core/logger.py`)

* Per tick, the logger captures robot states, tasks, message traffic, and active consensus slots.
* A final map snapshot is appended when the simulation ends. This mirrors the ASCII grid shown in the terminal.
* `simulation_log.txt` is automatically overwritten on each run so logs remain current.

---

## Recent Fixes (documented changes)

1. **Removed unused backups** — Old `sim.py.bak` has been deleted.
2. **Terminal animation** — CLI now supports `--animate` with optional `--sleep-sec` to emulate live playback.
3. **Logging upgrade** — Final grid snapshot is written to `simulation_log.txt` for post-run review.
4. **Carry coordination** — Pair movement now reverts illegal moves and keeps pre-/post-move separation in sync, preventing phantom steps.
5. **Consensus freshness** — Teammate positions expire automatically; Paxos quorums only include robots seen in the last five ticks.
6. **Anti-freeze watchdog** — Robots stuck waiting on gold reset their help state and re-enter exploration; consensus state is garbage-collected after prolonged inactivity.

---

## Running the Simulation

```bash
# quick stream (no animation)
python main.py --ticks 60 --seed 123 --gold 10 --print-every 1

# smooth 5 FPS animation with clear-screen redraws
python main.py --ticks 60 --seed 123 --gold 10 --print-every 1 --sleep-sec 0.2 --animate
```

Key flags:

* `--robots-per-team` — set team size (default 10).  
* `--gold` — initial piles (default 10).  
* `--consensus {paxos,handshake}` — choose coordination mode (handshake mirrors the CSP logic from Milestone 3).  
* `--log-file`, `--frames-file`, `--analysis-file` — redirect artefacts or disable by passing `""`.  
* `--run-until-finished / --no-run-until-finished` — control whether the sim continues beyond `--ticks`.  

The engine keeps stepping until all gold is deposited (unless `--no-run-until-finished`), guaranteeing that
the generated `frames.json` captures the entire match. A hard ceiling of 1,000 ticks remains in place via
`--max-ticks` to prevent runaway simulations.

---

## Visualization

Every run refreshes `frames.json`, which drives the in-repo viewer:

```bash
python main.py --ticks 200 --print-every 20 --consensus handshake
python3 -m http.server 8000
```

Point your browser at `http://localhost:8000/visualization.html` to scrub through frames, track robots,
adjust playback speed down to 0.06 s, and view the winner overlay once the final frame is reached.

---

## CPS/PFA analysis

`--analysis-file game_analysis.json` (the default) emits a JSON bundle containing the tracked environment properties (gold remaining, team scores, carrying pairs, consensus/handshake pressure), the cyber agents, their message-channel latency/drop parameters, and the observed transition matrix `T`. Set `--analysis-file ""` to disable the export.

---

## Next Extensions

* Finish wiring the accept phase of Paxos if you need explicit acknowledgement semantics.
* Implement real FOV so robots prefer newly observed gold instead of historical sightings.
* Add scenario tests (pytest) to cover pickup, deposit, and consensus edge cases.
* Persist telemetry (CSV/JSON) for offline analysis or visual dashboards.

Vipas Chantrapanichkul

Cyber Physical Robots: Term project
August 9, 2025
1 Overview
The goal of this project is to demonstrate mastery of the concepts of Cyber Physical Robotics, including distributed design, mathematical formalisms for distributed systems, and simulation methodologies. This mastery will be demonstrated through the design and implementation of several simulations and an
associated report, describing this process.
The final simulation must encompass:
• A grid-world of (i.e., square matrix) of 20x20.
• 2 groups of robots: each group consists of 10 robots. Robots in each group
may communicate with each other, but not with the other group. Each
robot can ”see” 8 positions, depending on the direction it’s facing: the 3
position directly in front, in the next row/column, and the 5 after that
(e.g., assuming x and y grow in the east and north directions, respectively: if a robot is at position (5,5), facing north, it can observe positions
(4,6),(5,6),(6,6) and (3,7),(4,7),(5,7),(6,7),(7.7)).
• Each robot must have its dedicated control logic, responding only to its
history, currently sensed positions, and whatever messages it has received
thus far.
• At each simulation step, each robot may either change facing direction,
move one position in the direction it’s facing, or perform an action on the
current position. Several robots may occupy the same position at once.
• Apart from robots, in each position there is either: nothing; one or more
gold bars; a deposit point. In the whole grid, there should be only 2
deposit points, one per robot group (robots are allowed to know where
these are). Gold bars should be initialized at random.
• A gold bar can be picked up from a position if and only if two robots
of the same group are on that position, and exactly two of them perform
the action pick up on a simulation step. If only one, or more than 2, try
to do this, picking up fails. If 4 robots (2 from each group) all attempt
1
to pick up simultaneously, each gets a gold bar if there are at least 2 gold
bars on that position: if only 1, both groups fail.
• Once a pair of robots is holding a gold bar, they can move it if they both
move in the same direction at each simulation time step. If they fail to do
so (i.e., move in different directions) the gold bar is dropped.
• If robots can reach the deposit point while carrying a gold bar, the bar
disappears, and that group gets a point.
At each submission milestone, students will provide an updated simulation
where robot control and coordination is improved, using the concepts learned in
the course so far. The report at each milestone must describe the mathematics,
and corresponding description of software implementation, of those concepts in
the simulation.
2 Assessment
Assessment will prioritize students’ demonstration of mastery over the course’s
learning objectives, rather than the sophistication of the prototype. However,
it is a necessary pre-condition that the prototype works.
Each assessment milestone accounts for 25% of the final grade.
3 Timeline
There are 4 milestones throughout the term. At each milestone, you must upload
the report thus far, and demonstrate current working simulation, up until the
current level of development.
1. 12 of September
2. 03 of October
3. 24 of October
4. 21 of November
