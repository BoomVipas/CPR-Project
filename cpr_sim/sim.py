from __future__ import annotations
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass
import random
import heapq

from .world.grid import GridWorld
from .world.render import render_ascii
from .agents.robot import Robot, CarryPair
from .core.types import DIR_ORDER, DIRS, addv
from .core.bus import Bus
from .core.logger import SimulationLogger
from .core.consensus import PairBroker, PairDecision, PairState

@dataclass
class Config:
    ticks: int = 100   # Shorter for watching
    seed: int = 123
    gold: int = 10     # Less gold for easier watching
    print_every: int = 1
    log_file: str = "simulation_log.txt"
    detailed_log: bool = True
    num_per_team: int = 10
    sleep_sec: float = 0.0
    animate: bool = False

class Simulation:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.gw = GridWorld(seed=cfg.seed, gold=cfg.gold)
        rng = random.Random(cfg.seed+555)
        self.busA = Bus(rng, delay_max=1, drop_prob=0.0)
        self.busB = Bus(rng, delay_max=1, drop_prob=0.0)
        self.robots: List[Robot] = []
        self.team_visited: Dict[str, Set[Tuple[int, int]]] = {"A": set(), "B": set()}
        self.deposit_positions = {"A": self.gw.deposA, "B": self.gw.deposB}
        for i in range(self.cfg.num_per_team):
            pos = (rng.randint(0,5), rng.randint(0,5))
            self.robots.append(Robot(rid=i, group="A", pos=pos, facing=rng.choice(DIR_ORDER), bus=self.busA, seed=cfg.seed, team_visited=self.team_visited["A"]))
        for i in range(self.cfg.num_per_team, self.cfg.num_per_team*2):
            pos = (self.gw.size-1 - rng.randint(0,5), self.gw.size-1 - rng.randint(0,5))
            self.robots.append(Robot(rid=i, group="B", pos=pos, facing=rng.choice(DIR_ORDER), bus=self.busB, seed=cfg.seed, team_visited=self.team_visited["B"]))
        self.scoreA = 0
        self.scoreB = 0
        self.t = 0
        self.pair_brokers: Dict[Tuple[int, int], PairBroker] = {}

        # Initialize logging
        self.logger = SimulationLogger(cfg.log_file, cfg.detailed_log)
        if cfg.detailed_log:
            self.logger.log_simulation_start(
                cfg.seed, cfg.gold, cfg.ticks, self.gw.size,
                self.gw.deposA, self.gw.deposB
            )

    def robot_by_id(self, rid:int) -> Optional[Robot]:
        for r in self.robots:
            if r.id==rid: return r
        return None

    def broker_for(self, cell: Tuple[int, int]) -> PairBroker:
        broker = self.pair_brokers.get(cell)
        if broker is None:
            broker = PairBroker(cell, deposit_positions=self.deposit_positions)
            self.pair_brokers[cell] = broker
        return broker

    def print_grid(self):
        print(render_ascii(self.gw, self.robots, self.scoreA, self.scoreB, self.t))

    def step(self):
        self.logger.log_tick_start(self.t, self.scoreA, self.scoreB, self.robots, self.gw, self.team_visited)

        broker_events: List[dict] = []
        expired_pairs: List[dict] = []
        for cell, broker in self.pair_brokers.items():
            expired = broker.expire_all(self.t)
            if expired:
                for ev in expired:
                    expired_pairs.append(ev)

        def remove_task(robot: Robot, task: Task) -> None:
            """Remove a task from a robot's scheduler and keep the heap valid."""
            robot.sched.h.remove(task)
            heapq.heapify(robot.sched.h)

        # Step 1: Message Delivery Phase
        for m in self.busA.deliver(self.t):
            r = self.robot_by_id(m.dst)
            if r: r.inbox.append(m)
        for m in self.busB.deliver(self.t):
            r = self.robot_by_id(m.dst)
            if r: r.inbox.append(m)

        # Provide team lists to robots
        teamA = [r.id for r in self.robots if r.group == "A"]
        teamB = [r.id for r in self.robots if r.group == "B"]

        # Step 2: Planning Phase - all robots plan their tasks
        for r in self.robots:
            r.plan(self.t, self.gw, teamA if r.group == "A" else teamB)

        # Step 3: Communication Phase - process sense tasks and messages
        for r in self.robots:
            # Process sense tasks first to handle communication
            sense_task = None
            for task in r.sched.h:
                if task.name == "sense" and task.release <= self.t:
                    sense_task = task
                    break

            if sense_task:
                remove_task(r, sense_task)
                r.digest_inbox(self.t, r.inbox)
                r.maybe_beacon(self.t, teamA if r.group == "A" else teamB)

        # Step 4: Pair offer phase - interact with brokers
        pair_decisions: List[PairDecision] = []
        for r in self.robots:
            offer_task = None
            for task in r.sched.h:
                if task.name == "pair_offer" and task.release <= self.t:
                    offer_task = task
                    break

            if offer_task:
                remove_task(r, offer_task)
                if not r.carry and self.gw.gold_at(r.pos) > 0:
                    broker = self.broker_for(r.pos)
                    decisions = r.offer_pair(self.t, broker)
                    pair_decisions.extend(decisions)

        confirmed_states: Dict[Tuple[int, int], PairState] = {}
        for decision in pair_decisions:
            broker = self.broker_for(decision.cell)
            for rid in decision.pair:
                robot = self.robot_by_id(rid)
                if robot:
                    robot.receive_pair(self.t, decision)
                    state = broker.mark_ready(robot.group, rid, self.t)
                    if state and state.confirmed:
                        confirmed_states[state.pair] = state
            self.logger.log_pair_formed(decision.team, decision.pair, decision.cell, decision.decided_at)

        # Keep-alive readiness checks for robots still waiting on confirmation.
        for r in self.robots:
            if r.carry:
                continue
            if r.current_pair and not r.current_pair.confirmed and r.current_pair.cell == r.pos:
                broker = self.broker_for(r.current_pair.cell)
                state = broker.mark_ready(r.group, r.id, self.t)
                if state and state.confirmed:
                    confirmed_states[state.pair] = state

        # Notify robots whose pairs just became confirmed.
        for pair_key, state in confirmed_states.items():
            for rid in pair_key:
                robot = self.robot_by_id(rid)
                if robot:
                    robot.on_pair_confirmed(self.t)
            self.logger.log_pair_confirmed(state.team, list(pair_key), state.cell, self.t)

        for cell, broker in self.pair_brokers.items():
            drained = broker.drain_events()
            if drained:
                broker_events.extend(drained)

        # Handle broker expirations (timeouts) and log raw broker events.
        for ev in expired_pairs:
            pair_tuple = tuple(ev.get("pair", []))
            for rid in pair_tuple:
                robot = self.robot_by_id(rid)
                if robot:
                    robot.on_pair_timeout(self.t, ev["cell"], ev.get("reason", "unknown"), pair_tuple if pair_tuple else None)
            self.logger.log_pair_expired(ev["team"], list(ev.get("pair", [])), ev["cell"], ev["tick"], ev.get("reason", "unknown"))

        if broker_events:
            self.logger.log_broker_events(broker_events)

        # Step 5: Coordination Phase - handle coordinate tasks
        intentsA: Dict[Tuple[int,int], List[int]] = {}
        intentsB: Dict[Tuple[int,int], List[int]] = {}

        for r in self.robots:
            coord_task = None
            for task in r.sched.h:
                if task.name == "coordinate" and task.release <= self.t:
                    coord_task = task
                    break

            if coord_task:
                remove_task(r, coord_task)
                cell = r.pos
                if self.gw.cells[cell[1]][cell[0]]["gold"] > 0 and not r.carry:
                    if r.current_pair and r.current_pair.cell == cell:
                        mate = self.robot_by_id(r.current_pair.partner_id)
                        if mate and mate.pos == cell and not mate.carry and r.current_pair.confirmed:
                            if r.group == "A":
                                intentsA.setdefault(cell, []).append(r.id)
                            else:
                                intentsB.setdefault(cell, []).append(r.id)

        # Step 6: Movement Phase - handle movement tasks
        planned_moves: Dict[int, Optional[str]] = {}
        processed_pairs = set()
        pre_move_positions = {r.id: r.pos for r in self.robots}

        for r in self.robots:
            # Initialize all robots with no planned move
            planned_moves[r.id] = None

            # Check for movement tasks (explore or to_deposit)
            movement_task = None
            for task in r.sched.h:
                if (task.name in ["explore", "to_deposit"] and task.release <= self.t):
                    movement_task = task
                    break

            if movement_task:
                remove_task(r, movement_task)
                if movement_task.name == "explore":
                    mv = r.step_explore(self.gw, self.robots)
                    # Handle all possible return values from step_explore
                    if mv == "move":
                        planned_moves[r.id] = "move"
                    elif mv == "pickup":
                        planned_moves[r.id] = "pickup"
                    elif mv in ["N", "S", "E", "W"]:  # Direction movements
                        planned_moves[r.id] = mv
                    else:
                        planned_moves[r.id] = None  # rot or other non-movement actions
                elif movement_task.name == "to_deposit":
                    # Handle paired movement for carrying robots
                    if r.carry:
                        mate = self.robot_by_id(r.carry.mate_id)
                        pair_key = tuple(sorted([r.id, mate.id]))

                        if pair_key not in processed_pairs:
                            processed_pairs.add(pair_key)
                            # Coordinate movement for both robots
                            planned_moves[r.id], planned_moves[mate.id] = self.coordinate_pair_movement(r, mate)
                    else:
                        # Single robot movement
                        planned_moves[r.id] = r.step_to_deposit(self.gw)

        # Handle pickup intents from movement phase
        for r in self.robots:
            if planned_moves.get(r.id) == "pickup":
                cell = r.pos
                if self.gw.cells[cell[1]][cell[0]]["gold"] > 0 and not r.carry:
                    if r.group == "A":
                        intentsA.setdefault(cell, []).append(r.id)
                    else:
                        intentsB.setdefault(cell, []).append(r.id)

        # Collect all messages sent this step for logging
        all_messages = []
        for r in self.robots:
            all_messages.extend(r.get_and_clear_sent_messages())

        # Log messages and consensus states
        self.logger.log_messages(all_messages)
        self.logger.log_broker_states(self.pair_brokers)

        self.resolve_pickups(intentsA, intentsB)

        visited = set()
        for r in self.robots:
            if not r.carry:
                continue
            mate = self.robot_by_id(r.carry.mate_id)
            if not mate or not mate.carry:
                r.carry = None
                continue
            key = tuple(sorted([r.id, mate.id]))
            if key in visited:
                continue
            visited.add(key)

            d1 = planned_moves.get(r.id)
            d2 = planned_moves.get(mate.id)
            tgt = self.gw.deposA if r.group == "A" else self.gw.deposB

            r_pre = pre_move_positions.get(r.id, r.pos)
            mate_pre = pre_move_positions.get(mate.id, mate.pos)

            r_dist_before = abs(r_pre[0] - tgt[0]) + abs(r_pre[1] - tgt[1])
            mate_dist_before = abs(mate_pre[0] - tgt[0]) + abs(mate_pre[1] - tgt[1])

            r_pos_after = r.pos
            mate_pos_after = mate.pos

            r_dist_after = abs(r_pos_after[0] - tgt[0]) + abs(r_pos_after[1] - tgt[1])
            mate_dist_after = abs(mate_pos_after[0] - tgt[0]) + abs(mate_pos_after[1] - tgt[1])

            max_separation = 3  # Allow some separation for pathfinding
            pair_separation = abs(r_pos_after[0] - mate_pos_after[0]) + abs(r_pos_after[1] - mate_pos_after[1])

            if ((r_dist_after > r_dist_before and mate_dist_after > mate_dist_before) or
                pair_separation > max_separation):
                r.carry = None
                mate.carry = None
                continue

            # Check for successful deposit
            if r.pos == tgt and mate.pos == tgt:
                gold_deposited = r.carry.gold_count
                if r.group == "A":
                    self.scoreA += gold_deposited
                    self.logger.log_gold_deposit('A', [r.id, mate.id], gold_deposited, self.scoreA)
                else:
                    self.scoreB += gold_deposited
                    self.logger.log_gold_deposit('B', [r.id, mate.id], gold_deposited, self.scoreB)
                origin_cell = r.carry.origin_cell
                broker = self.broker_for(origin_cell)
                release_a = r.release_pair(self.t, broker, mate.id)
                release_b = mate.release_pair(self.t, broker, r.id)
                cleared_state = release_a or release_b
                if cleared_state:
                    self.logger.log_pair_released(
                        cleared_state.team,
                        list(cleared_state.pair),
                        cleared_state.cell,
                        cleared_state.cleared_at if cleared_state.cleared_at is not None else self.t
                    )
                else:
                    self.logger.log_pair_release_pending(
                        r.group,
                        [r.id, mate.id],
                        origin_cell,
                        self.t
                    )
                for bot in (r, mate):
                    if hasattr(bot, 'add_cooldown'):
                        bot.add_cooldown(origin_cell, self.t)
                    if hasattr(bot, '_forget_gold'):
                        bot._forget_gold(origin_cell)
                    if hasattr(bot, 'wander_dir'):
                        bot.wander_dir = None
                        bot.wander_steps = 0
                self.clear_pair(r, mate)

        # Log actions taken this tick
        self.logger.log_actions(planned_moves, intentsA, intentsB)

        self.t += 1

    def clear_pair(self, r1, r2):
        """Fully reset a carrying pair after deposit."""
        for r in (r1, r2):
            r.carry = None
            if hasattr(r, 'carry_pair'):
                r.carry_pair = None
            r.role = "SCOUT"
            r.target_gold = None
            if hasattr(r, 'current_pair'):
                r.current_pair = None
            if hasattr(r, 'pending_offer_cell'):
                r.pending_offer_cell = None
            if hasattr(r, 'offer_wait_deadline'):
                r.offer_wait_deadline = None
            if hasattr(r, 'cell_cooldowns') and hasattr(r, 'pos'):
                r.cell_cooldowns.pop(r.pos, None)
            if hasattr(r, '_help_requested'):
                delattr(r, '_help_requested')
            if hasattr(r, '_help_wait_counter'):
                delattr(r, '_help_wait_counter')
            if hasattr(r, 'sched'):
                r.sched.h = [task for task in r.sched.h if task.name != 'to_deposit']
                heapq.heapify(r.sched.h)
            if hasattr(r, 'wander_dir'):
                r.wander_dir = None
                r.wander_steps = 0
            if hasattr(r, 'enqueue_explore_soon'):
                r.enqueue_explore_soon(self.t)
        if hasattr(self.logger, 'log_pair_cleared'):
            self.logger.log_pair_cleared([r1.id, r2.id], self.t)

    def resolve_pickups(self, intentsA, intentsB):
        all_cells = set(intentsA.keys()) | set(intentsB.keys())
        for cell in all_cells:
            wantA = intentsA.get(cell, [])
            wantB = intentsB.get(cell, [])
            gold_here = self.gw.cells[cell[1]][cell[0]]["gold"]
            if len(wantA)==2 and len(wantB)==2:
                if gold_here >= 2:
                    self.gw.cells[cell[1]][cell[0]]["gold"] -= 2
                    self.pair(wantA[0], wantA[1], cell)
                    self.pair(wantB[0], wantB[1], cell)
                    self.logger.log_gold_pickup('A', wantA, cell, 1)
                    self.logger.log_gold_pickup('B', wantB, cell, 1)
                continue
            if len(wantA)==2 and gold_here>=1:
                self.gw.cells[cell[1]][cell[0]]["gold"] -= 1
                self.pair(wantA[0], wantA[1], cell)
                self.logger.log_gold_pickup('A', wantA, cell, 1)
            if len(wantB)==2 and gold_here>=1:
                self.gw.cells[cell[1]][cell[0]]["gold"] -= 1
                self.pair(wantB[0], wantB[1], cell)
                self.logger.log_gold_pickup('B', wantB, cell, 1)

    def coordinate_pair_movement(self, r1, r2):
        """Coordinate movement for a pair of robots carrying gold together."""
        tgt = self.gw.deposA if r1.group == 'A' else self.gw.deposB

        current_separation = abs(r1.pos[0] - r2.pos[0]) + abs(r1.pos[1] - r2.pos[1])

        r1_state = (r1.pos, getattr(r1, 'last_pos', r1.pos), getattr(r1, 'last_moved_tick', 0))
        r2_state = (r2.pos, getattr(r2, 'last_pos', r2.pos), getattr(r2, 'last_moved_tick', 0))
        r1_prev = r1_state[0]
        r2_prev = r2_state[0]

        def manhattan(pos):
            return abs(pos[0] - tgt[0]) + abs(pos[1] - tgt[1])

        r1_dist_before = manhattan(r1_prev)
        r2_dist_before = manhattan(r2_prev)

        r1_move = r1.step_to_deposit(self.gw)
        r1_after = r1.pos
        r1_dist_after = manhattan(r1_after)
        r1_moved = bool(r1_move and r1_move in DIRS)

        r2_move = r2.step_to_deposit(self.gw)
        r2_after = r2.pos
        r2_dist_after = manhattan(r2_after)
        r2_moved = bool(r2_move and r2_move in DIRS)

        def revert_r1():
            nonlocal r1_move
            if r1_moved:
                r1.pos, r1.last_pos, r1.last_moved_tick = r1_state
            r1_move = None

        def revert_r2():
            nonlocal r2_move
            if r2_moved:
                r2.pos, r2.last_pos, r2.last_moved_tick = r2_state
            r2_move = None

        if r1_move and r1_move in DIRS and r2_move and r2_move in DIRS:
            if r1_after == r2_after:
                return r1_move, r2_move
            new_separation = abs(r1_after[0] - r2_after[0]) + abs(r1_after[1] - r2_after[1])
            if (new_separation <= current_separation and
                r1_dist_after <= r1_dist_before and
                r2_dist_after <= r2_dist_before):
                return r1_move, r2_move
            sep_r1_only = abs(r1_after[0] - r2_prev[0]) + abs(r1_after[1] - r2_prev[1])
            sep_r2_only = abs(r1_prev[0] - r2_after[0]) + abs(r1_prev[1] - r2_after[1])
            r1_improves = (r1_dist_after < r1_dist_before) or (
                r1_dist_after == r1_dist_before and sep_r1_only <= current_separation)
            r2_improves = (r2_dist_after < r2_dist_before) or (
                r2_dist_after == r2_dist_before and sep_r2_only <= current_separation)
            if r1_improves and not r2_improves:
                revert_r2()
                return r1_move, None
            if r2_improves and not r1_improves:
                revert_r1()
                return None, r2_move
            if r1_improves and r2_improves:
                if r1_dist_after <= r2_dist_after:
                    revert_r2()
                    return r1_move, None
                revert_r1()
                return None, r2_move
            revert_r1()
            revert_r2()
            return None, None

        if r1_move and r1_move in DIRS and (not r2_move or r2_move not in DIRS):
            new_separation = abs(r1_after[0] - r2_after[0]) + abs(r1_after[1] - r2_after[1])
            if new_separation <= current_separation:
                return r1_move, None
            revert_r1()
            return None, None

        if r2_move and r2_move in DIRS and (not r1_move or r1_move not in DIRS):
            new_separation = abs(r1_after[0] - r2_after[0]) + abs(r1_after[1] - r2_after[1])
            if new_separation <= current_separation:
                return None, r2_move
            revert_r2()
            return None, None

        return None, None

    def pair(self, r1_id:int, r2_id:int, cell: Tuple[int, int]):
        r1 = self.robot_by_id(r1_id); r2 = self.robot_by_id(r2_id)
        if r1 and r2 and not r1.carry and not r2.carry:
            r1.carry = CarryPair(mate_id=r2.id, gold_count=1, origin_cell=cell)
            r2.carry = CarryPair(mate_id=r1.id, gold_count=1, origin_cell=cell)
            r1.current_pair = None
            r2.current_pair = None

            # Reset robot states for transition to transport mode
            r1.role = "TRANSPORTER"
            r2.role = "TRANSPORTER"
            r1.target_gold = None
            r2.target_gold = None
            if hasattr(r1, '_help_requested'):
                delattr(r1, '_help_requested')
            if hasattr(r2, '_help_requested'):
                delattr(r2, '_help_requested')

    def run(self):
        import time

        clear_seq = "\033[2J\033[H"

        for _ in range(self.cfg.ticks):
            self.step()
            if (self.t - 1) % self.cfg.print_every == 0:
                if self.cfg.animate:
                    print(clear_seq, end="", flush=True)

                # Show current step info
                print(f"=== STEP {self.t:03d} ===")
                print(f"Score: Team A = {self.scoreA}, Team B = {self.scoreB}")
                remaining_gold = sum(
                    cell["gold"]
                    for row in self.gw.cells
                    for cell in row
                )
                print(f"Gold remaining: {remaining_gold}")

                # Show robot status
                print("\nRobot Status:")
                for r in self.robots:
                    carry_info = f" [Carrying with R{r.carry.mate_id}]" if r.carry else ""
                    role_info = f" [{r.role}]"
                    print(
                        f"  R{r.id} (Team {r.group}): ({r.pos[0]:2d},{r.pos[1]:2d}) "
                        f"facing {r.facing}{role_info}{carry_info}"
                    )

                print()
                self.print_grid()

                if self.cfg.sleep_sec > 0:
                    time.sleep(self.cfg.sleep_sec)

        print(f"\nFinal Score: Team A = {self.scoreA}, Team B = {self.scoreB}")

        # Close log file
        self.logger.log_simulation_end(self.scoreA, self.scoreB, self.gw, self.robots, self.t)
        self.logger.close()
