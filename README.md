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
   * Otherwise, keeps an `explore` task pending—even after waiting on help—so every robot eventually vacates a stalled cell.
   * When standing on gold, cooperates with the broker (`pair_offer`, `coordinate`) so only one confirmed pair attempts the pickup.
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

### Pair Broker (distance-aware carry window)

The Paxos phase nominates a pair, but the dedicated broker at `cpr_sim/core/consensus/pair_broker.py` enforces the lifecycle:

1. **Offer intake** — solo robots standing on gold register offers; the broker pairs the freshest two teammates.
2. **Readiness handshake** — both robots must report in on the cell within the `handshake_ttl` window before the pair is considered active.
3. **Dynamic TTL** — once confirmed, the broker grants an active window sized to the deposit distance (`max(active_ttl_min, manhattan(cell, deposit) * 2 + margin)`), preventing long-haul carriers from expiring mid-run while still clearing genuine stalemates.
4. **Release** — after deposit, each carrier calls `release`; once both check in the broker frees the slot for future offers.
5. **Diagnostics** — every state change is queued so the simulation logger can emit a complete event trace per tick.

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

1. **Scheduler hygiene** — Arbitrary task removals now re-heapify each robot’s EDF queue, ensuring deadlines are honoured and stale jobs never resurface.
2. **Transport coordination** — `coordinate_pair_movement` only accepts moves that shorten the route or safely converge, eliminating the “spin at deposit” deadlock.
3. **Distance-aware pair TTL** — Brokers size the carry window to the pickup-to-deposit distance, so legitimate long hauls no longer time out early.
4. **Gold intel lifecycle** — Robots retain sightings with timestamps (`GOLD_TTL_TICKS`); beacons share only the fresh, certain subset with teammates.
5. **Observability polish** — Console output, `simulation_log.txt`, and the ASCII renderer all reflect transporter state and final grid snapshots for quick post-run analysis.

---

## Running the Simulation

```bash
# quick stream (no animation)
python main.py --ticks 60 --seed 123 --gold 25 --print_every 1

# smooth 5 FPS animation with clear-screen redraws
python main.py --ticks 60 --seed 123 --gold 25 --print_every 1 --sleep-sec 0.2 --animate
```

The CLI also honours `--num-per-team`, `--log-file`, and `--no-detailed-log` for customised experiments.

---

## Next Extensions

* Finish wiring the accept phase of Paxos if you need explicit acknowledgement semantics.
* Implement real FOV so robots prefer newly observed gold instead of historical sightings.
* Add scenario tests (pytest) to cover pickup, deposit, and consensus edge cases.
* Persist telemetry (CSV/JSON) for offline analysis or visual dashboards.
* Explore congestion-aware routing or adaptive TTL margins for even faster deliveries.

Vipas Chantrapanichkul
