from dataclasses import dataclass
import heapq
import random
from typing import Dict, Tuple, Optional, List, Set, TYPE_CHECKING
from ..core.types import DIRS, DIR_ORDER, addv
from ..core.bus import Msg
from ..core.scheduler import EDF, Task
from ..core.clocks import LamportClock

if TYPE_CHECKING:
    from ..core.consensus import PairBroker, PairDecision

@dataclass
class CarryPair:
    """
    Represents a carrying relationship between robots.
    """
    mate_id: int
    gold_count: int
    origin_cell: Tuple[int, int]


@dataclass
class PairAssignment:
    """Tracks a tentative pickup pairing before the robots start carrying."""

    partner_id: int
    cell: Tuple[int, int]
    decided_at: int
    confirmed: bool = False
    wait_ticks: int = 0

class Robot:
    """
    Represents a robot agent in the CPR simulation.
    Handles movement, task scheduling, and coordination.
    """
    def __init__(self, rid, group, pos, facing, bus, seed, team_visited: Optional[Set[Tuple[int, int]]] = None):
        """
        Initialize the robot with its ID, group, position, facing direction, bus, and RNG seed.
        """
        self.id = rid
        self.group = group
        self.pos = pos
        self.facing = facing
        self.bus = bus
        self.rng = random.Random(seed + rid * 97)
        self.sched = EDF()
        self.carry = None  # CarryPair or None
        self.inbox = []

        # Communication and coordination
        self.clock = LamportClock()
        self.known_team_positions: dict[int, tuple[tuple[int, int], int]] = {}  # rid -> ((x,y), last_seen_tick)
        self.POS_TTL_TICKS = 5  # only count peers seen 5 ticks ago
        self.last_beacon: int = -1
        self.messages_sent_this_tick: List[dict] = []
        self.tick_now: int = 0
        self.pending_offer_cell: Optional[Tuple[int, int]] = None
        self.offer_wait_deadline: Optional[int] = None
        self.current_pair: Optional[PairAssignment] = None
        self.OFFER_WAIT_TICKS = 3

        # Enhanced AI and knowledge base
        self.knowledge_base: Dict[Tuple[int,int], Dict[str, any]] = {}
        self.gold_locations: List[Tuple[int,int]] = []
        self.gold_last_seen: Dict[Tuple[int, int], int] = {}
        self.gold_certainty: Dict[Tuple[int, int], bool] = {}
        self.GOLD_TTL_TICKS = 12
        self.role = "SCOUT"  # SCOUT, SUPPORTER, TRANSPORTER
        self.target_gold: Optional[Tuple[int,int]] = None
        self.wander_dir: Optional[str] = None
        self.wander_steps: int = 0
        self.last_pos: Tuple[int, int] = self.pos
        self.last_moved_tick: int = 0
        self.team_visited: Set[Tuple[int, int]] = team_visited if team_visited is not None else set()
        self.team_visited.add(self.pos)
        self.cell_cooldowns: Dict[Tuple[int, int], int] = {}
        self.COOLDOWN_TICKS = 6

    def send(self, now: int, dst: int, kind: str, payload: dict):
        """Send message to teammate."""
        ts = self.clock.send()
        msg = Msg(src=self.id, dst=dst, ts=ts, kind=kind, payload=payload)
        self.bus.send(now, msg)

        # Track message for logging
        self.messages_sent_this_tick.append({
            'src': self.id,
            'dst': dst,
            'kind': kind,
            'payload': payload,
            'ts': ts
        })

    def maybe_beacon(self, now: int, teammates: List[int]):
        """Send periodic position beacon to teammates with enhanced info."""
        if self.last_beacon == -1 or now - self.last_beacon >= 3:
            fresh_gold = [
                pos for pos in self.gold_locations
                if self.gold_certainty.get(pos, False)
                and now - self.gold_last_seen.get(pos, now) <= self.GOLD_TTL_TICKS
            ]
            for m in teammates:
                if m != self.id:
                    self.send(now, m, "pos", {
                        "pos": self.pos,
                        "facing": self.facing,
                        "carrying": bool(self.carry),
                        "role": self.role,
                        "gold_locations": fresh_gold,
                        "target_gold": self.target_gold
                    })
            self.last_beacon = now

    def _remember_teammate_position(self, rid: int, pos: Tuple[int, int], now: int):
        self.known_team_positions[rid] = (pos, now)

    def _get_teammate_position(self, rid: int, now: int) -> Optional[Tuple[int, int]]:
        data = self.known_team_positions.get(rid)
        if not data:
            return None
        pos, seen = data
        if now - seen > self.POS_TTL_TICKS:
            return None
        return pos

    def _prune_stale_positions(self, now: int):
        stale = [rid for rid, (_, seen) in self.known_team_positions.items() if now - seen > self.POS_TTL_TICKS]
        for rid in stale:
            self.known_team_positions.pop(rid, None)


    def _forget_gold(self, cell: Tuple[int, int]):
        if cell in self.gold_locations:
            self.gold_locations.remove(cell)
        self.gold_last_seen.pop(cell, None)
        self.gold_certainty.pop(cell, None)

    def _note_gold(self, cell: Tuple[int, int], amount: int, now: int, certainty: bool):
        if amount > 0:
            if certainty or not self._recently_saw_empty(cell, now):
                if cell not in self.gold_locations:
                    self.gold_locations.append(cell)
                self.gold_last_seen[cell] = now
                self.gold_certainty[cell] = certainty or self.gold_certainty.get(cell, False)
        else:
            self._forget_gold(cell)

    def _recently_saw_empty(self, cell: Tuple[int, int], now: int) -> bool:
        info = self.knowledge_base.get(cell)
        if not info:
            return False
        if info.get('gold', 0) > 0:
            return False
        seen_at = info.get('last_seen', now)
        return now - seen_at <= self.GOLD_TTL_TICKS

    def _prune_stale_gold(self, now: int):
        stale = [cell for cell, seen in self.gold_last_seen.items() if now - seen > self.GOLD_TTL_TICKS]
        for cell in stale:
            self._forget_gold(cell)


    def enqueue_explore_soon(self, now: int):
        self.sched.add(Task(deadline=now + 2, release=now, name='explore'))

    def add_cooldown(self, cell: Tuple[int, int], now: int, duration: Optional[int] = None):
        ticks = duration if duration is not None else self.COOLDOWN_TICKS
        self.cell_cooldowns[cell] = max(self.cell_cooldowns.get(cell, now), now) + ticks

    def request_help(self, now: int, teammates: List[int], gold_pos: Tuple[int,int]):
        """Request help from closest teammate for gold pickup."""
        if not teammates:
            return

        # Find closest teammate
        closest_teammate = None
        min_distance = float('inf')

        for tid in teammates:
            if tid == self.id:
                continue
            teammate_pos = self._get_teammate_position(tid, now)
            if teammate_pos is None:
                continue
            distance = abs(teammate_pos[0] - self.pos[0]) + abs(teammate_pos[1] - self.pos[1])
            if distance < min_distance:
                min_distance = distance
                closest_teammate = tid

        if closest_teammate:
            self.send(now, closest_teammate, "help_request", {
                "gold_pos": gold_pos,
                "requester_pos": self.pos,
                "requester_id": self.id
            })

    def digest_inbox(self, now: int, msgs: List[Msg]):
        for m in msgs:
            self.clock.recv(m.ts)
            if m.kind == "pos":
                pos = tuple(m.payload["pos"])
                self.known_team_positions[m.src] = (pos, now)
                if "gold_locations" in m.payload:
                    for gold_pos in m.payload["gold_locations"]:
                        self._note_gold(tuple(gold_pos), 1, now, False)
            elif m.kind == "help_request":
                gold_pos = tuple(m.payload.get("gold_pos", ()))
                if gold_pos and not self.carry and self.role != "TRANSPORTER":
                    self.role = "SUPPORTER"
                    self.target_gold = gold_pos
                    self._note_gold(gold_pos, 1, now, True)
        msgs.clear()

    def get_and_clear_sent_messages(self) -> List[dict]:
        """Get messages sent this tick and clear the buffer."""
        messages = self.messages_sent_this_tick.copy()
        self.messages_sent_this_tick.clear()
        return messages

    def offer_pair(self, now: int, broker: 'PairBroker'):
        """Submit an offer to pair up on the current gold cell."""
        if self.carry:
            return []
        cooldown_until = self.cell_cooldowns.get(self.pos)
        if cooldown_until and now < cooldown_until:
            return []
        if self.current_pair and self.current_pair.cell == self.pos:
            return []
        if self.pending_offer_cell != self.pos:
            self.pending_offer_cell = self.pos
            self.offer_wait_deadline = now + self.OFFER_WAIT_TICKS
        return broker.offer(self.group, self.id, now)

    def receive_pair(self, now: int, decision: 'PairDecision') -> None:
        """Record a tentative broker pairing and wait for confirmation."""
        if self.id not in decision.pair:
            return
        partner_id = decision.pair[0] if decision.pair[1] == self.id else decision.pair[1]
        self.current_pair = PairAssignment(
            partner_id=partner_id,
            cell=decision.cell,
            decided_at=decision.decided_at,
            confirmed=False,
            wait_ticks=0,
        )
        self.pending_offer_cell = None
        self.offer_wait_deadline = None
        # Remove lingering pair_offer tasks; we'll wait for confirmation.
        self.sched.h = [task for task in self.sched.h if task.name != "pair_offer"]
        heapq.heapify(self.sched.h)
        # Drop stale exploration work once assigned so we wait on the brokered pickup window.
        self.sched.h = [task for task in self.sched.h if task.name != "explore"]
        heapq.heapify(self.sched.h)

    def release_pair(self, now: int, broker: 'PairBroker', mate_id: int):
        """Coordinate release handshake with the broker after deposit."""
        if not self.carry:
            return None
        return broker.release(self.group, self.id, mate_id, now)

    def on_pair_confirmed(self, now: int) -> None:
        """Mark the current tentative pair as confirmed and schedule coordination."""
        if not self.current_pair:
            return
        self.current_pair.confirmed = True
        self.current_pair.wait_ticks = 0
        self.sched.add(Task(deadline=now + 1, release=now, name="coordinate"))

    def on_pair_timeout(self, now: int, cell: Tuple[int, int], reason: str, pair: Optional[Tuple[int, int]] = None) -> None:
        """Handle broker notification that a pending pair expired."""
        if self.carry and pair and self.carry.mate_id in pair:
            self.carry = None
        if self.current_pair and self.current_pair.cell == cell and not self.carry:
            self.current_pair = None
            self.sched.h = [task for task in self.sched.h if task.name != "coordinate"]
            heapq.heapify(self.sched.h)
            self.pending_offer_cell = None
            self.offer_wait_deadline = None
            self.add_cooldown(cell, now)
            self.enqueue_explore_soon(now)

    def visible_cells(self, gw, all_robots):
        """
        Return a list of visible cells in the gridworld.
        Robot can see 8 positions in front of it based on facing direction.
        """
        visible = []
        x, y = self.pos

        # Define observation patterns for each direction (8 cells in front)
        patterns = {
            'N': [(-1, -1), (0, -1), (1, -1), (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2)],
            'S': [(-1, 1), (0, 1), (1, 1), (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2)],
            'E': [(1, -1), (1, 0), (1, 1), (2, -2), (2, -1), (2, 0), (2, 1), (2, 2)],
            'W': [(-1, -1), (-1, 0), (-1, 1), (-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2)]
        }

        rel_coords = list(patterns.get(self.facing, []))

        # Add current position and adjacent cells for better awareness
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:  # Skip current position
                    continue
                rel_coords.append((dx, dy))

        for dx, dy in rel_coords:
            nx, ny = x + dx, y + dy
            if gw.inb((nx, ny)):
                cell_data = gw.cells[ny][nx]
                robots_here = [r.id for r in all_robots if r.pos == (nx, ny) and r.id != self.id]
                robots_with_facing = [{'id': r.id, 'facing': r.facing} for r in all_robots if r.pos == (nx, ny) and r.id != self.id]

                # Update knowledge base
                self.knowledge_base[(nx, ny)] = {
                    'gold': cell_data['gold'],
                    'robots': robots_here.copy(),
                    'robots_with_facing': robots_with_facing.copy(),
                    'last_seen': self.tick_now
                }

                # Update known positions for visible teammates
                for robot_info in robots_with_facing:
                    self._remember_teammate_position(robot_info['id'], (nx, ny), self.tick_now)

                # Track gold locations
                self._note_gold((nx, ny), cell_data['gold'], self.tick_now, True)

                visible.append({
                    'pos': (nx, ny),
                    'gold': cell_data['gold'],
                    'robots': robots_here,
                    'robots_with_facing': robots_with_facing
                })
        return visible




    def plan(self, now: int, gw, team_ids: List[int]):
        self.tick_now = now
        self._prune_stale_positions(now)
        self._prune_stale_gold(now)
        self.team_visited.add(self.pos)
        for cell, until in list(self.cell_cooldowns.items()):
            if now >= until:
                del self.cell_cooldowns[cell]
        if self.pending_offer_cell and self.pending_offer_cell != self.pos:
            self.pending_offer_cell = None
            self.offer_wait_deadline = None
        if self.offer_wait_deadline is not None and now > self.offer_wait_deadline:
            self.pending_offer_cell = None
            self.offer_wait_deadline = None
        if self.current_pair and not self.carry and self.current_pair.cell != self.pos:
            self.current_pair = None
        if self.current_pair:
            if self.current_pair.confirmed:
                self.current_pair.wait_ticks = 0
            else:
                self.current_pair.wait_ticks += 1
                if self.current_pair.wait_ticks >= self.OFFER_WAIT_TICKS:
                    self.add_cooldown(self.current_pair.cell, now)
                    self.current_pair = None
                    self.pending_offer_cell = None
                    self.offer_wait_deadline = None
                    self.enqueue_explore_soon(now)
        self.sched.add(Task(deadline=now + 1, release=now, name="sense"))
        if self.carry:
            self.sched.add(Task(deadline=now + 1, release=now, name="to_deposit"))
        else:
            cell = self.pos
            gold_here = gw.gold_at(cell)
            assignment = self.current_pair
            cooldown_until = self.cell_cooldowns.get(cell)
            if gold_here > 0:
                if cooldown_until and now < cooldown_until:
                    self.sched.add(Task(deadline=now + 1, release=now, name="explore"))
                elif assignment and assignment.cell == cell and assignment.confirmed:
                    self.sched.add(Task(deadline=now + 1, release=now, name="coordinate"))
                elif not assignment or assignment.cell != cell:
                    self.sched.add(Task(deadline=now + 1, release=now, name="pair_offer"))
                self.sched.add(Task(deadline=now + 3, release=now, name="explore"))
            else:
                self.sched.add(Task(deadline=now + 3, release=now, name="explore"))


    def rotate_right(self):
        """
        Rotate the robot 90 degrees to the right.
        """
        i = DIR_ORDER.index(self.facing)
        self.facing = DIR_ORDER[(i + 1) % 4]

    def rotate_left(self):
        """
        Rotate the robot 90 degrees to the left.
        """
        i = DIR_ORDER.index(self.facing)
        self.facing = DIR_ORDER[(i - 1) % 4]

    def rotate_back(self):
        """
        Rotate the robot 180 degrees (turn around).
        """
        i = DIR_ORDER.index(self.facing)
        self.facing = DIR_ORDER[(i + 2) % 4]

    def _face_direction(self, target_dir):
        """Helper method to efficiently rotate to face target direction."""
        if self.facing == target_dir:
            return

        current_idx = DIR_ORDER.index(self.facing)
        target_idx = DIR_ORDER.index(target_dir)

        # Calculate shortest rotation
        right_turns = (target_idx - current_idx) % 4
        left_turns = (current_idx - target_idx) % 4

        if right_turns == 1:
            self.rotate_right()
        elif left_turns == 1:
            self.rotate_left()
        elif right_turns == 2:
            self.rotate_back()
        else:
            self.rotate_right()  # fallback

    def _move_to(self, dest: Tuple[int, int], record: bool = True):
        if record and dest != self.pos:
            self.last_pos = self.pos
            self.pos = dest
            self.last_moved_tick = self.tick_now
            self.team_visited.add(dest)
        else:
            self.pos = dest

    def step_explore(self, gw, all_robots=None):
        """
        Intelligent exploration with gold-seeking behavior.
        """
        if self.role != "SCOUT":
            self.wander_dir = None
            self.wander_steps = 0


        # PRIORITY 0: If we're carrying gold, we should not be in explore mode - this is handled by to_deposit task
        if self.carry:
            # If we somehow end up in step_explore while carrying, just rotate (shouldn't happen)
            self.role = "TRANSPORTER"
            self.rotate_right()
            return 'rot'

        # PRIORITY 1: If we're standing on gold, sync with the brokered flow.
        current_cell = gw.cells[self.pos[1]][self.pos[0]]
        if current_cell['gold'] > 0 and not self.carry and self.role == "SCOUT":
            # Check if we have a teammate at the same location
            # First update positions of any visible teammates at this location
            visible = self.visible_cells(gw, all_robots or [])
            for cell in visible:
                if cell['pos'] == self.pos and 'robots_with_facing' in cell:
                    for robot_info in cell['robots_with_facing']:
                        self._remember_teammate_position(robot_info['id'], self.pos, self.tick_now)

            teammates_at_gold = [rid for rid, (pos, seen) in self.known_team_positions.items()
                               if rid != self.id and pos == self.pos and (self.tick_now - seen) <= self.POS_TTL_TICKS]

            if self.current_pair and self.current_pair.cell == self.pos:
                # Already matched; let the coordinate task drive pickup rather than freelancing here.
                return 'rot'

            if self.pending_offer_cell == self.pos:
                # Waiting on broker decision – hold position to keep the offer fresh.
                return 'rot'

            cooldown_until = self.cell_cooldowns.get(self.pos)
            if cooldown_until and self.tick_now < cooldown_until:
                dirs = list(DIR_ORDER)
                self.rng.shuffle(dirs)
                for d in dirs:
                    nx = addv(self.pos, DIRS[d])
                    if gw.inb(nx):
                        if self.facing != d:
                            self._face_direction(d)
                            return 'rot'
                        self._move_to(nx)
                        return 'move'
                self.rotate_right()
                return 'rot'

            # If more than two teammates are already here, non-essential scouts back off
            present_ids = sorted({self.id} | set(teammates_at_gold))
            if len(present_ids) > 2 and self.id not in present_ids[:2]:
                self.cell_cooldowns[self.pos] = self.tick_now + self.COOLDOWN_TICKS
                if hasattr(self, '_help_requested'):
                    delattr(self, '_help_requested')
                if hasattr(self, '_help_wait_counter'):
                    delattr(self, '_help_wait_counter')
                dirs = list(DIR_ORDER)
                self.rng.shuffle(dirs)
                for d in dirs:
                    nx = addv(self.pos, DIRS[d])
                    if gw.inb(nx):
                        if self.facing != d:
                            self._face_direction(d)
                            return 'rot'
                        self._move_to(nx)
                        return 'move'
                self.rotate_right()
                return 'rot'

            if len(teammates_at_gold) > 0:
                # Teammate present but no pair yet; wait for broker or re-offer.
                self.rotate_right()
                return 'rot'

            # If we've already requested help, wait a bit but don't wait forever
            if hasattr(self, '_help_requested'):
                if not hasattr(self, '_help_wait_counter'):
                    self._help_wait_counter = 0
                self._help_wait_counter += 1

                # Wait for up to 3 steps, then continue exploring (reduced from 5)
                if self._help_wait_counter < 3:
                    self.rotate_right()
                    return 'rot'
                else:
                    # Reset and continue exploring (no help coming)
                    delattr(self, '_help_requested')
                    delattr(self, '_help_wait_counter')

            # Request help from available teammates
            available_teammates = [tid for tid, (pos, seen) in self.known_team_positions.items()
                                 if tid != self.id and (self.tick_now - seen) <= self.POS_TTL_TICKS and not hasattr(self, '_help_requested')]
            if available_teammates:
                self.request_help(self.tick_now, available_teammates, self.pos)
                self._help_requested = True
                self._help_wait_counter = 0
                self.rotate_right()
                return 'rot'

        # PRIORITY 1: If we're a SUPPORTER, head to the gold location we're supposed to help with
        if self.role == "SUPPORTER" and self.target_gold and not self.carry:
            target = self.target_gold

            # Calculate direction to target gold
            dx = target[0] - self.pos[0]
            dy = target[1] - self.pos[1]

            # If we've reached the target gold, switch back to SCOUT to participate in consensus
            if dx == 0 and dy == 0:
                self.role = "SCOUT"
                self.target_gold = None
                return 'rot'  # Just rotate to end the action

            # Choose best direction to reach target
            if abs(dx) >= abs(dy):
                want_dir = 'E' if dx > 0 else 'W' if dx < 0 else None
            else:
                want_dir = 'S' if dy > 0 else 'N' if dy < 0 else None

            if want_dir and want_dir != self.facing:
                self._face_direction(want_dir)
                return 'rot'
            elif want_dir:
                nx = addv(self.pos, DIRS[want_dir])
                if gw.inb(nx):
                    self._move_to(nx)
                    return 'move'

        # PRIORITY 2: If we know about gold locations, head towards the nearest one
        if self.gold_locations and not self.carry and self.role == "SCOUT":
            target_gold = min(self.gold_locations, key=lambda g: abs(g[0] - self.pos[0]) + abs(g[1] - self.pos[1]))
            self.target_gold = target_gold

            # Calculate direction to gold
            dx = target_gold[0] - self.pos[0]
            dy = target_gold[1] - self.pos[1]

            # Choose best direction
            if abs(dx) >= abs(dy):
                want_dir = 'E' if dx > 0 else 'W' if dx < 0 else None
            else:
                want_dir = 'S' if dy > 0 else 'N' if dy < 0 else None

            if want_dir and want_dir != self.facing:
                self._face_direction(want_dir)
                return 'rot'
            elif want_dir:
                nx = addv(self.pos, DIRS[want_dir])
                if gw.inb(nx):
                    self._move_to(nx)
                    return 'move'

        # Enhanced random exploration - avoid backtracking and prefer unexplored areas
        visible = self.visible_cells(gw, all_robots or [])
        unexplored = [(nx, ny) for cell in visible
                      for nx, ny in [cell['pos']]
                      if (nx, ny) not in self.knowledge_base]

        if unexplored:
            # Head towards unexplored area
            target = self.rng.choice(unexplored)
            dx = target[0] - self.pos[0]
            dy = target[1] - self.pos[1]

            if abs(dx) >= abs(dy):
                want_dir = 'E' if dx > 0 else 'W' if dx < 0 else None
            else:
                want_dir = 'S' if dy > 0 else 'N' if dy < 0 else None

            if want_dir and want_dir != self.facing:
                self._face_direction(want_dir)
                return 'rot'
            elif want_dir:
                nx = addv(self.pos, DIRS[want_dir])
                if gw.inb(nx):
                    self._move_to(nx)
                    return 'move'

        # Directed wandering to keep covering new territory
        if self.role == "SCOUT":
            if self.wander_steps <= 0 or self.wander_dir is None:
                dirs = list(DIR_ORDER)
                self.rng.shuffle(dirs)
                chosen = None
                for d in dirs:
                    nx = addv(self.pos, DIRS[d])
                    if gw.inb(nx):
                        chosen = d
                        break
                self.wander_dir = chosen
                self.wander_steps = self.rng.randint(4, 8) if chosen else 0
            if self.wander_dir:
                if self.facing != self.wander_dir:
                    self._face_direction(self.wander_dir)
                    return 'rot'
                nx = addv(self.pos, DIRS[self.wander_dir])
                if gw.inb(nx):
                    self._move_to(nx)
                    self.wander_steps -= 1
                    return 'move'
                self.wander_dir = None
                self.wander_steps = 0

        if self.rng.random() < 0.7:  # Higher chance to move forward
            nx = addv(self.pos, DIRS[self.facing])
            if gw.inb(nx):
                self._move_to(nx)
                return 'move'

        # Smart rotation - prefer turning towards center or unexplored areas
        rotation_choice = self.rng.choice(['right', 'left'])
        if rotation_choice == 'right':
            self.rotate_right()
        else:
            self.rotate_left()
        return 'rot'

    def step_to_deposit(self, gw):
        """
        Move towards the group's deposit location.
        Prioritizes horizontal movement, then vertical.
        Rotates if not facing the desired direction.
        """
        tgt = gw.deposA if self.group == 'A' else gw.deposB
        dx = tgt[0] - self.pos[0]
        dy = tgt[1] - self.pos[1]
        # Decide which direction to move
        if abs(dx) >= abs(dy):
            want = 'E' if dx > 0 else ('W' if dx < 0 else None)
        else:
            want = 'S' if dy > 0 else ('N' if dy < 0 else None)
        # Rotate if not facing the desired direction
        if want and want != self.facing:
            # Calculate the most efficient rotation
            current_idx = DIR_ORDER.index(self.facing)
            target_idx = DIR_ORDER.index(want)
            # Calculate turns needed in both directions
            right_turns = (target_idx - current_idx) % 4
            left_turns = (current_idx - target_idx) % 4

            # Choose the shortest rotation
            if right_turns == 1:
                self.rotate_right()
            elif left_turns == 1:
                self.rotate_left()
            elif right_turns == 2:  # 180 degrees
                self.rotate_back()
            else:
                self.rotate_right()  # fallback
            return None
        # Move if possible
        if want:
            nx = addv(self.pos, DIRS[want])
            if gw.inb(nx):
                self._move_to(nx)
                return want
        return None
