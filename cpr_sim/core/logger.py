from typing import List, Dict, Tuple, Optional, TextIO
from ..agents.robot import Robot
from ..world.render import render_ascii


class SimulationLogger:
    """Handles detailed logging for CPR simulation debugging."""

    def __init__(self, log_file_path: str, enabled: bool = True):
        self.enabled = enabled
        self.log_file: Optional[TextIO] = None

        if self.enabled:
            self.log_file = open(log_file_path, 'w')

    def log_simulation_start(self, seed: int, gold: int, ticks: int, grid_size: int,
                           deposit_a: Tuple[int, int], deposit_b: Tuple[int, int]):
        """Log simulation initialization parameters."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write("=== CPR SIMULATION LOG ===\n")
        self.log_file.write(f"Seed: {seed}, Gold: {gold}, Ticks: {ticks}\n")
        self.log_file.write(f"Grid Size: {grid_size}x{grid_size}\n")
        self.log_file.write(f"Deposit A: {deposit_a}, Deposit B: {deposit_b}\n\n")
        self.log_file.flush()

    def log_tick_start(self, step: int, score_a: int, score_b: int, robots: List[Robot], gw):
        """Log the start of a new step with detailed robot states."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write(f"\n--- STEP {step:03d} START ---\n")
        self.log_file.write(f"Current Score: A={score_a}, B={score_b}\n")

        # Log gold locations and amounts
        gold_locations = []
        total_gold = 0
        for y in range(gw.size):
            for x in range(gw.size):
                gold_amount = gw.cells[y][x]['gold']
                if gold_amount > 0:
                    gold_locations.append(f"({x},{y}):G{gold_amount}")
                    total_gold += gold_amount

        if gold_locations:
            self.log_file.write(f"Gold on map (total: {total_gold}): {', '.join(gold_locations)}\n")
        else:
            self.log_file.write("No gold remaining on map\n")
        self.log_file.write("\n")

        for r in robots:
            visible = r.visible_cells(gw, robots)
            visible_str = []
            for cell in visible:
                if cell['gold'] > 0 or cell['robots'] or (hasattr(cell, 'robots_with_facing') and cell.get('robots_with_facing')):
                    robots_info = cell['robots']
                    if 'robots_with_facing' in cell and cell['robots_with_facing']:
                        facing_info = [f"R{ri['id']}({ri['facing']})" for ri in cell['robots_with_facing']]
                        robots_info = f"[{','.join(facing_info)}]"
                    visible_str.append(f"({cell['pos'][0]},{cell['pos'][1]}):G{cell['gold']}R{robots_info}")

            carry_info = ""
            if r.carry:
                carry_info = f" [Carrying with R{r.carry.mate_id}, Gold:{r.carry.gold_count}]"

            # Add role and additional state info
            role_info = f" [{r.role}]"
            state_info = ""
            if hasattr(r, 'target_gold') and r.target_gold:
                state_info += f" target_gold={r.target_gold}"
            if hasattr(r, '_help_requested') and r._help_requested:
                state_info += f" help_requested"
            if hasattr(r, '_help_wait_counter') and r._help_wait_counter:
                state_info += f" waiting={r._help_wait_counter}"

            self.log_file.write(f"Robot {r.id:2d} (Group {r.group}): pos=({r.pos[0]:2d},{r.pos[1]:2d}) facing={r.facing}{role_info}{carry_info}{state_info}\n")
            if visible_str:
                self.log_file.write(f"  Sees: {', '.join(visible_str)}\n")

            # Enhanced task and knowledge info
            task_names = [t.name for t in r.sched.h]
            self.log_file.write(f"  Tasks: {task_names}\n")

            if hasattr(r, 'gold_locations') and r.gold_locations:
                self.log_file.write(f"  Known gold: {r.gold_locations}\n")

            if hasattr(r, 'known_team_positions') and r.known_team_positions:
                teammate_pos = {rid: pos for rid, pos in r.known_team_positions.items() if rid != r.id}
                if teammate_pos:
                    self.log_file.write(f"  Teammate positions: {teammate_pos}\n")

        # Add map visualization
        self.log_file.write("Current Map:\n")
        map_lines = render_ascii(gw, robots, score_a, score_b, step).split('\n')
        for line in map_lines:
            self.log_file.write(f"  {line}\n")

        self.log_file.write("\n")
        self.log_file.flush()

    def log_consensus_states(self, robots: List):
        """Log current consensus states for debugging."""
        if not self.enabled or not self.log_file:
            return

        active_consensus = False
        for r in robots:
            if r.consensus:
                if not active_consensus:
                    self.log_file.write("Active Consensus States:\n")
                    active_consensus = True

                for cell, state in r.consensus.items():
                    status = "DECIDED" if state.get("decided") else "IN_PROGRESS"
                    decided_pair = state.get("decided", "None")
                    quorum = state.get("quorum", "?")
                    proposed = state.get("proposed", (None, None))

                    self.log_file.write(f"  Robot {r.id} for cell {cell}: {status}\n")
                    self.log_file.write(f"    Decided: {decided_pair}, Quorum: {quorum}\n")
                    if proposed[0]:
                        self.log_file.write(f"    Proposed: n={proposed[0]}, pair={proposed[1]}\n")

        if active_consensus:
            self.log_file.write("\n")

    def log_messages(self, messages_sent: List[dict]):
        """Log all messages sent between robots this step."""
        if not self.enabled or not self.log_file:
            return

        if messages_sent:
            self.log_file.write("Messages sent this step:\n")
            for msg in messages_sent:
                self.log_file.write(f"  R{msg['src']} -> R{msg['dst']}: {msg['kind']}")
                if msg['kind'] == 'pos':
                    payload = msg['payload']
                    role = payload.get('role', 'UNKNOWN')
                    target_gold = payload.get('target_gold', None)
                    target_str = f", target={target_gold}" if target_gold else ""
                    self.log_file.write(f" [pos={payload['pos']}, facing={payload['facing']}, carrying={payload['carrying']}, role={role}{target_str}]")
                elif msg['kind'] == 'help_request':
                    payload = msg['payload']
                    self.log_file.write(f" [gold_pos={payload.get('gold_pos', 'N/A')}, requester_pos={payload.get('requester_pos', 'N/A')}]")
                elif msg['kind'] == 'paxos_prepare':
                    payload = msg['payload']
                    self.log_file.write(f" [cell={payload['cell']}, n={payload['n']}, pair={payload.get('pair', 'N/A')}]")
                elif msg['kind'] == 'paxos_promise':
                    payload = msg['payload']
                    self.log_file.write(f" [cell={payload['cell']}, n={payload['n']}, accepted_n={payload['accepted_n']}]")
                else:
                    # For any other message types, show the raw payload
                    payload = msg.get('payload', {})
                    if payload:
                        self.log_file.write(f" {payload}")
                self.log_file.write("\n")
            self.log_file.write("\n")

    def log_actions(self, planned_moves: Dict[int, Optional[str]],
                   intents_a: Dict[Tuple[int,int], List[int]],
                   intents_b: Dict[Tuple[int,int], List[int]]):
        """Log actions taken and coordination attempts."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write("Actions taken:\n")
        for robot_id, move in planned_moves.items():
            if move:
                self.log_file.write(f"  Robot {robot_id}: {move}\n")

        # Log coordination attempts
        if intents_a or intents_b:
            self.log_file.write("Coordination attempts:\n")
            for cell, robots in intents_a.items():
                if len(robots) >= 2:
                    self.log_file.write(f"  Group A at {cell}: robots {robots}\n")
            for cell, robots in intents_b.items():
                if len(robots) >= 2:
                    self.log_file.write(f"  Group B at {cell}: robots {robots}\n")

        self.log_file.flush()

    def log_gold_pickup(self, group: str, robot_ids: List[int], position: Tuple[int, int], gold_amount: int):
        """Log successful gold pickup."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write(f"GOLD PICKUP: Group {group} robots {robot_ids} picked up {gold_amount} gold at {position}\n")
        self.log_file.flush()

    def log_gold_deposit(self, group: str, robot_ids: List[int], gold_amount: int, new_score: int):
        """Log successful gold deposit."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write(f"GOLD DEPOSIT: Group {group} robots {robot_ids} deposited {gold_amount} gold. New score: {new_score}\n")
        self.log_file.flush()

    def log_simulation_end(self, final_score_a: int, final_score_b: int, gw, robots, final_tick: int):
        """Log simulation completion with final map snapshot."""
        if not self.enabled or not self.log_file:
            return

        self.log_file.write(f"\n=== SIMULATION COMPLETE ===\n")
        self.log_file.write(f"Final Score: A={final_score_a}  B={final_score_b}\n")

        self.log_file.write("Final Map:\n")
        map_lines = render_ascii(gw, robots, final_score_a, final_score_b, final_tick).split('\n')
        for line in map_lines:
            self.log_file.write(f"  {line}\n")

        self.log_file.flush()

    def close(self):
        """Close the log file."""
        if self.log_file:
            self.log_file.close()
            self.log_file = None
