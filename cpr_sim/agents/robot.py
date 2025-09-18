from dataclasses import dataclass
import random
from typing import Dict, Tuple, Optional, List
from ..core.types import DIRS, DIR_ORDER, addv
from ..core.bus import Msg
from ..core.scheduler import EDF, Task
from ..core.clocks import LamportClock
from ..core.consensus.paxos import Proposer, Acceptor

@dataclass
class CarryPair:
    """
    Represents a carrying relationship between robots.
    """
    mate_id: int
    gold_count: int

class Robot:
    """
    Represents a robot agent in the CPR simulation.
    Handles movement, task scheduling, and coordination.
    """
    def __init__(self, rid, group, pos, facing, bus, seed):
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
        # Paxos book-keeping per cell
        # consensus[cell] = {
        #   "n": int, "decided": list[int]|None,
        #   "last_proposer_id": int|None, "last_proposer_tick": int,
        #   "updated_at": int
        # }
        self.consensus: dict[tuple[int, int], dict] = {}
        self.acceptor = Acceptor(self.id)
        self.proposer = Proposer(self.id, quorum=[])
        self.messages_sent_this_tick: List[dict] = []
        self.tick_now: int = 0

        # Enhanced AI and knowledge base
        self.knowledge_base: Dict[Tuple[int,int], Dict[str, any]] = {}
        self.gold_locations: List[Tuple[int,int]] = []
        self.role = "SCOUT"  # SCOUT, SUPPORTER, TRANSPORTER
        self.target_gold: Optional[Tuple[int,int]] = None
        self.wander_dir: Optional[str] = None
        self.wander_steps: int = 0
        self.last_pos: Tuple[int, int] = self.pos
        self.last_moved_tick: int = 0

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
            for m in teammates:
                if m != self.id:
                    self.send(now, m, "pos", {
                        "pos": self.pos,
                        "facing": self.facing,
                        "carrying": bool(self.carry),
                        "role": self.role,
                        "gold_locations": self.gold_locations.copy(),
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


    def enqueue_explore_soon(self, now: int):
        self.sched.add(Task(deadline=now + 2, release=now, name='explore'))

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
        out_promises: List[Msg] = []
        out_accepted: List[Msg] = []
        for m in msgs:
            self.clock.recv(m.ts)
            if m.kind == "pos":
                pos = tuple(m.payload["pos"])
                self.known_team_positions[m.src] = (pos, now)
                if "gold_locations" in m.payload:
                    for gold_pos in m.payload["gold_locations"]:
                        tup = tuple(gold_pos)
                        if tup not in self.gold_locations:
                            self.gold_locations.append(tup)
            elif m.kind == "help_request":
                gold_pos = tuple(m.payload.get("gold_pos", ()))
                if gold_pos and not self.carry and self.role != "TRANSPORTER":
                    self.role = "SUPPORTER"
                    self.target_gold = gold_pos
                    if gold_pos not in self.gold_locations:
                        self.gold_locations.append(gold_pos)
            elif m.kind == "paxos_prepare":
                cell = tuple(m.payload["cell"])
                ok, na, va = self.acceptor.on_prepare(m.payload["n"])
                if ok:
                    self.send(now, m.src, "paxos_promise", {
                        "cell": cell,
                        "n": m.payload["n"],
                        "na": na,
                        "va": va
                    })
            elif m.kind == "paxos_promise":
                out_promises.append(m)
            elif m.kind == "paxos_accept":
                ok = self.acceptor.on_accept(m.payload["n"], m.payload.get("v"))
                if ok:
                    self.send(now, m.src, "paxos_accepted", {
                        "cell": tuple(m.payload["cell"]),
                        "n": m.payload["n"],
                        "v": m.payload.get("v")
                    })
            elif m.kind == "paxos_accepted":
                out_accepted.append(m)
        msgs.clear()
        return out_promises, out_accepted

    def get_and_clear_sent_messages(self) -> List[dict]:
        """Get messages sent this tick and clear the buffer."""
        messages = self.messages_sent_this_tick.copy()
        self.messages_sent_this_tick.clear()
        return messages

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

        rel_coords = patterns.get(self.facing, [])

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
                    'last_seen': self.clock.t
                }

                # Update known positions for visible teammates
                for robot_info in robots_with_facing:
                    self._remember_teammate_position(robot_info['id'], (nx, ny), self.tick_now)

                # Track gold locations
                if cell_data['gold'] > 0 and (nx, ny) not in self.gold_locations:
                    self.gold_locations.append((nx, ny))
                elif cell_data['gold'] == 0 and (nx, ny) in self.gold_locations:
                    self.gold_locations.remove((nx, ny))

                visible.append({
                    'pos': (nx, ny),
                    'gold': cell_data['gold'],
                    'robots': robots_here,
                    'robots_with_facing': robots_with_facing
                })
        return visible

    def peers_on_cell(self, now: int, cell: Tuple[int, int], team_ids: List[int]) -> List[int]:
        fresh: List[int] = []
        for rid in team_ids:
            if rid == self.id:
                if self.pos == cell:
                    fresh.append(rid)
                continue
            rec = self.known_team_positions.get(rid)
            if not rec:
                continue
            pos, t_seen = rec
            if pos == cell and (now - t_seen) <= self.POS_TTL_TICKS:
                fresh.append(rid)
        return sorted(fresh)



    def try_consensus_pair(self, now: int, gw, team_ids: List[int]):
        cell = self.pos
        if gw.gold_at(cell) <= 0:
            return
        peers = self.peers_on_cell(now, cell, team_ids)
        if len(peers) < 2:
            return
        st = self.consensus.setdefault(cell, {
            "n": 0,
            "decided": None,
            "last_proposer_id": None,
            "last_proposer_tick": -1,
            "updated_at": now
        })
        st["updated_at"] = now
        proposer = peers[0]
        if st["last_proposer_tick"] == (now - 1) and not st["decided"]:
            last = st["last_proposer_id"] or proposer
            if last in peers:
                proposer = peers[(peers.index(last) + 1) % len(peers)]
        if self.id != proposer or st["decided"]:
            return
        st["last_proposer_id"] = self.id
        st["last_proposer_tick"] = now
        st["n"] = self.proposer.next_n()
        cand = peers[:2]
        for rid in peers:
            self.send(now, rid, "paxos_prepare", {"cell": cell, "n": st["n"], "pair": cand})

    def integrate_promises_and_accepts(self, now: int, promises: List[Msg], accepted: List[Msg], team_ids: List[int]):
        by_cell: Dict[Tuple[int, int], List[Msg]] = {}
        for m in promises:
            cell = tuple(m.payload["cell"])
            by_cell.setdefault(cell, []).append(m)
        for cell, msgs in by_cell.items():
            st = self.consensus.get(cell)
            if not st or st.get("decided"):
                continue
            peers = self.peers_on_cell(now, cell, team_ids)
            quorum = len(peers) // 2 + 1
            eligible = [m for m in msgs if m.payload.get("n") == st.get("n") and m.src in peers]
            if len({m.src for m in eligible}) < quorum:
                continue
            na_max = -1
            value = None
            for m in eligible:
                na = m.payload.get("na", -1)
                va = m.payload.get("va")
                if va is not None and na > na_max:
                    na_max = na
                    value = va
            if value is None:
                value = peers[:2]
            for rid in peers:
                self.send(now, rid, "paxos_accept", {"cell": cell, "n": st["n"], "v": value})
            st["updated_at"] = now
        by_cell_accepted: Dict[Tuple[int, int], List[Msg]] = {}
        for m in accepted:
            cell = tuple(m.payload["cell"])
            by_cell_accepted.setdefault(cell, []).append(m)
        for cell, msgs in by_cell_accepted.items():
            st = self.consensus.get(cell)
            if not st or st.get("decided"):
                continue
            peers = self.peers_on_cell(now, cell, team_ids)
            quorum = len(peers) // 2 + 1
            eligible = [m for m in msgs if m.payload.get("n") == st.get("n") and m.src in peers]
            if len({m.src for m in eligible}) < quorum:
                continue
            st["decided"] = eligible[0].payload.get("v")
            st["updated_at"] = now



    def plan(self, now: int, gw, team_ids: List[int]):
        self.tick_now = now
        self._prune_stale_positions(now)
        self.gc_consensus(now)
        self.sched.add(Task(deadline=now + 1, release=now, name="sense"))
        if self.carry:
            self.sched.add(Task(deadline=now + 1, release=now, name="to_deposit"))
        else:
            cell = self.pos
            gold_here = gw.gold_at(cell)
            decided = self.consensus.get(cell, {}).get("decided")
            if gold_here > 0:
                if decided and self.id in decided:
                    self.sched.add(Task(deadline=now + 1, release=now, name="coordinate"))
                else:
                    self.sched.add(Task(deadline=now + 1, release=now, name="pair_consensus"))
            if not (decided and self.id in decided):
                self.sched.add(Task(deadline=now + 3, release=now, name="explore"))

    def gc_consensus(self, now: int):
        CONS_TTL = 15
        for cell, st in list(self.consensus.items()):
            if (now - st.get("updated_at", now)) > CONS_TTL:
                del self.consensus[cell]


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

        # PRIORITY 1: If we're standing on gold, ATTEMPT PICKUP!
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

            # If more than two teammates are already here, non-essential scouts back off
            present_ids = sorted({self.id} | set(teammates_at_gold))
            if len(present_ids) > 2 and self.id not in present_ids[:2]:
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

            # If we have a teammate here, BOTH try to pick up gold!
            if len(teammates_at_gold) > 0:
                print(f"DEBUG: Robot {self.id} sees teammates {teammates_at_gold} at gold {self.pos} - ATTEMPTING PICKUP!")
                # Return 'pickup' to signal pickup attempt
                return 'pickup'

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
                self.request_help(self.clock.t, available_teammates, self.pos)
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

        # Fallback to improved random exploration
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



