from __future__ import annotations
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import random
import heapq

from .world.grid import GridWorld
from .world.render import render_ascii
from .agents.robot import Robot, CarryPair
from .core.types import DIR_ORDER, DIRS, addv
from .core.bus import Bus
from .core.logger import SimulationLogger

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
        for i in range(self.cfg.num_per_team):
            pos = (rng.randint(0,5), rng.randint(0,5))
            self.robots.append(Robot(rid=i, group="A", pos=pos, facing=rng.choice(DIR_ORDER), bus=self.busA, seed=cfg.seed))
        for i in range(self.cfg.num_per_team, self.cfg.num_per_team*2):
            pos = (self.gw.size-1 - rng.randint(0,5), self.gw.size-1 - rng.randint(0,5))
            self.robots.append(Robot(rid=i, group="B", pos=pos, facing=rng.choice(DIR_ORDER), bus=self.busB, seed=cfg.seed))
        self.scoreA = 0
        self.scoreB = 0
        self.t = 0

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

    def print_grid(self):
        print(render_ascii(self.gw, self.robots, self.scoreA, self.scoreB, self.t))

    def step(self):
        self.logger.log_tick_start(self.t, self.scoreA, self.scoreB, self.robots, self.gw)

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
        promises = []
        accepted = []

        for r in self.robots:
            # Process sense tasks first to handle communication
            sense_task = None
            for task in r.sched.h:
                if task.name == "sense" and task.release <= self.t:
                    sense_task = task
                    break

            if sense_task:
                r.sched.h.remove(sense_task)
                pr, acc = r.digest_inbox(self.t, r.inbox)
                promises.extend(pr)
                accepted.extend(acc)
                r.maybe_beacon(self.t, teamA if r.group == "A" else teamB)

        # Step 4: Consensus Phase - handle consensus-related tasks
        for r in self.robots:
            consensus_task = None
            for task in r.sched.h:
                if task.name == "pair_consensus" and task.release <= self.t:
                    consensus_task = task
                    break

            if consensus_task:
                r.sched.h.remove(consensus_task)
                r.try_consensus_pair(self.t, self.gw, teamA if r.group == "A" else teamB)

        # Integrate Paxos messages after all robots have processed communication
        for r in self.robots:
            r.integrate_promises_and_accepts(
                self.t, promises, accepted, teamA if r.group == "A" else teamB
            )

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
                r.sched.h.remove(coord_task)
                cell = r.pos
                if self.gw.cells[cell[1]][cell[0]]["gold"] > 0 and not r.carry:
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
                r.sched.h.remove(movement_task)
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
        self.logger.log_consensus_states(self.robots)

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
            if hasattr(r, '_help_requested'):
                delattr(r, '_help_requested')
            if hasattr(r, '_help_wait_counter'):
                delattr(r, '_help_wait_counter')
            if hasattr(r, 'consensus'):
                cells = list(r.consensus.keys())
                for cell in cells:
                    st = r.consensus.get(cell)
                    if not st or st.get('decided') is None or r.id in st.get('decided', []):
                        r.consensus.pop(cell, None)
            if hasattr(r, 'sched'):
                r.sched.h = [task for task in r.sched.h if task.name != 'to_deposit']
                heapq.heapify(r.sched.h)
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
                    self.pair(wantA[0], wantA[1])
                    self.pair(wantB[0], wantB[1])
                    self.logger.log_gold_pickup('A', wantA, cell, 1)
                    self.logger.log_gold_pickup('B', wantB, cell, 1)
                continue
            if len(wantA)==2 and gold_here>=1:
                self.gw.cells[cell[1]][cell[0]]["gold"] -= 1
                self.pair(wantA[0], wantA[1])
                self.logger.log_gold_pickup('A', wantA, cell, 1)
            if len(wantB)==2 and gold_here>=1:
                self.gw.cells[cell[1]][cell[0]]["gold"] -= 1
                self.pair(wantB[0], wantB[1])
                self.logger.log_gold_pickup('B', wantB, cell, 1)

    def coordinate_pair_movement(self, r1, r2):
        """Coordinate movement for a pair of robots carrying gold together."""
        tgt = self.gw.deposA if r1.group == 'A' else self.gw.deposB

        if r1.facing != r2.facing:
            def rotate_toward(robot, target_facing):
                current_idx = DIR_ORDER.index(robot.facing)
                target_idx = DIR_ORDER.index(target_facing)
                right_turns = (target_idx - current_idx) % 4
                left_turns = (current_idx - target_idx) % 4
                if right_turns == 2 and left_turns == 2:
                    robot.rotate_back()
                elif right_turns <= left_turns:
                    robot.rotate_right()
                else:
                    robot.rotate_left()

            r1_dist = abs(r1.pos[0] - tgt[0]) + abs(r1.pos[1] - tgt[1])
            r2_dist = abs(r2.pos[0] - tgt[0]) + abs(r2.pos[1] - tgt[1])

            if r1_dist <= r2_dist:
                rotate_toward(r2, r1.facing)
            else:
                rotate_toward(r1, r2.facing)
            return None, None

        current_separation = abs(r1.pos[0] - r2.pos[0]) + abs(r1.pos[1] - r2.pos[1])

        r1_state = (r1.pos, getattr(r1, 'last_pos', r1.pos), getattr(r1, 'last_moved_tick', 0))
        r2_state = (r2.pos, getattr(r2, 'last_pos', r2.pos), getattr(r2, 'last_moved_tick', 0))

        r1_move = r1.step_to_deposit(self.gw)
        r1_after = r1.pos
        r1_moved = bool(r1_move and r1_move in DIRS)

        r2_move = r2.step_to_deposit(self.gw)
        r2_after = r2.pos
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
            if r1_move == r2_move:
                return r1_move, r2_move
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

    def pair(self, r1_id:int, r2_id:int):
        r1 = self.robot_by_id(r1_id); r2 = self.robot_by_id(r2_id)
        if r1 and r2 and not r1.carry and not r2.carry:
            r1.carry = CarryPair(mate_id=r2.id, gold_count=1)
            r2.carry = CarryPair(mate_id=r1.id, gold_count=1)

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




