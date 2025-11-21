from __future__ import annotations

from collections import defaultdict
from typing import Any, Callable, Dict, List, Optional, Tuple

from .bus import MessageBus
from .models import Frame, GridWorld, Message, Robot
from .toolset import deposit, move_forward, pick_up, turn_left, turn_right


class SimulationEngine:
    """
    Thin engine that executes one tick of the CPR simulation.

    Responsibilities:
    1. Deliver queued broadcasts
    2. Collect actions from robot strategies
    3. Apply movement / rotation
    4. Enforce 2-robot pickup rule
    5. Ensure carrying robots stay synchronised
    6. Process deposits
    7. Queue new broadcasts
    """

    def __init__(
        self,
        world: GridWorld,
        robots: List[Robot],
        strategies: Dict[int, Callable[[Robot, GridWorld, "SimulationEngine"], Any]],
        cfg,
        rng,
    ):
        self.world = world
        self.robots = robots
        self.strategies = strategies
        self.scores = {"A": 0, "B": 0}
        self.pickups = {"A": 0, "B": 0}
        self.logs: List[str] = []
        self.step = 0
        self.cfg = cfg
        self.rng = rng
        self.message_bus = MessageBus(
            latency_range=cfg.message_delay_range,
            drop_chance=cfg.message_loss_chance,
            reorder_chance=cfg.message_reorder_chance,
            rng=rng,
        )

    def tick(self) -> Frame:
        self._deliver_broadcasts()
        actions: Dict[int, Any] = {}
        broadcasts: Dict[int, Dict[str, Any]] = {}

        order = list(self.robots)
        self.rng.shuffle(order)
        for robot in order:
            strategy = self.strategies.get(robot.id)
            if not strategy:
                actions[robot.id] = None
                continue
            result = strategy(robot, self.world, self)
            if isinstance(result, tuple):
                action, payload = result
                actions[robot.id] = action
                if payload is not None:
                    broadcasts[robot.id] = payload
            else:
                actions[robot.id] = result

        movements = self._apply_movement_and_rotation(actions)
        self._enforce_carrying_cohesion(movements)
        pickups = self._process_pickups(actions)
        self._process_deposits(actions)

        for robot_id, payload in broadcasts.items():
            robot = next((r for r in self.robots if r.id == robot_id), None)
            if robot:
                delay = self.message_bus.queue(robot, payload, self.step)
                if delay is None:
                    self.logs.append(
                        f"Network dropped {payload.get('type', 'message')} from R{robot.id} (Team {robot.team})."
                    )
                elif delay > 1:
                    self.logs.append(
                        f"Message {payload.get('type', 'message')} from R{robot.id} delayed {delay - 1} ticks."
                    )

        frame = Frame(
            step=self.step,
            robots=[Robot(
                id=r.id,
                team=r.team,
                number=r.number,
                pos=r.pos,
                facing=r.facing,
                carrying=r.carrying,
                carrying_with=r.carrying_with,
                messages=list(r.messages),
                history=list(r.history),
                paxos_state=r.paxos_state,
                proposal_id=r.proposal_id,
                promises_received=set(r.promises_received),
                current_plan=r.current_plan.copy() if r.current_plan else None,
                leader_id=r.leader_id,
                assigned_gold=r.assigned_gold,
                assigned_partner=r.assigned_partner,
                execution_state=r.execution_state,
                pending_broadcast=dict(r.pending_broadcast) if r.pending_broadcast else None,
                target_gold_for_consensus=r.target_gold_for_consensus,
                known_gold=set(r.known_gold),
                gold_report_pending=list(r.gold_report_pending) if r.gold_report_pending else None,
                control_state=r.control_state,
                state_entered_step=r.state_entered_step,
                timers=dict(r.timers),
                awaiting_ack={k: set(v) for k, v in r.awaiting_ack.items()},
                consensus_deadline=r.consensus_deadline,
                consensus_retries=r.consensus_retries,
            ) for r in self.robots],
            gold=list(self.world.gold),
            scores=dict(self.scores),
            pickups=dict(self.pickups),
            deposits=dict(self.world.deposits),
            width=self.world.width,
            height=self.world.height,
            logs=list(self.logs),
        )

        for robot in self.robots:
            robot.history.append(
                {
                    "step": self.step,
                    "action": actions.get(robot.id),
                    "pos": robot.pos,
                    "carrying": robot.carrying,
                    "control_state": robot.control_state,
                }
            )

        self.step += 1
        self.logs = []
        return frame

    # ------------------------------------------------------------------#
    # Stage helpers                                                      #
    # ------------------------------------------------------------------#

    def _deliver_broadcasts(self) -> None:
        ready = self.message_bus.drain(self.step)
        for transmission in ready:
            message = transmission.message
            recipients = [
                robot for robot in self.robots if robot.team == message.team and robot.id != message.sender_id
            ]
            if transmission.latency > 1:
                self.logs.append(
                    f"Delivered delayed {message.content.get('type', 'message')} from R{message.sender_id} "
                    f"(delay={transmission.latency - 1} ticks)."
                )
            for robot in recipients:
                robot.messages.append(message)

    def _apply_movement_and_rotation(self, actions: Dict[int, Any]) -> Dict[int, Tuple[int, int]]:
        movements: Dict[int, Tuple[int, int]] = {}
        for robot in self.robots:
            action = actions.get(robot.id)
            if action == "turn_left":
                turn_left(robot)
                self.logs.append(f"Robot {robot.id} (Team {robot.team}) turned left.")
            elif action == "turn_right":
                turn_right(robot)
                self.logs.append(f"Robot {robot.id} (Team {robot.team}) turned right.")
            elif action == "move_forward":
                old_pos = robot.pos
                if move_forward(robot, self.world):
                    movements[robot.id] = robot.pos
                    self.logs.append(f"Robot {robot.id} (Team {robot.team}) moved forward to {robot.pos}.")
                else:
                    self.logs.append(f"Robot {robot.id} (Team {robot.team}) failed to move (boundary).")
        return movements

    def _enforce_carrying_cohesion(self, movements: Dict[int, Tuple[int, int]]) -> None:
        checked = set()
        for robot in self.robots:
            if not robot.carrying or robot.carrying_with is None:
                continue
            pair = tuple(sorted((robot.id, robot.carrying_with)))
            if pair in checked:
                continue
            checked.add(pair)
            partner = next((r for r in self.robots if r.id == robot.carrying_with), None)
            if not partner:
                continue
            moved_a = robot.id in movements
            moved_b = partner.id in movements
            if moved_a != moved_b or (moved_a and movements[robot.id] != movements[partner.id]):
                self._drop_gold(robot, partner)

    def _drop_gold(self, robot_a: Robot, robot_b: Robot) -> None:
        self.world.gold.add(robot_a.pos)
        robot_a.carrying = False
        robot_a.carrying_with = None
        robot_b.carrying = False
        robot_b.carrying_with = None
        self.logs.append(
            f"Robots {robot_a.id} and {robot_b.id} (Team {robot_a.team}) dropped gold at {robot_a.pos} (lost coordination)."
        )

    def _process_pickups(self, actions: Dict[int, Any]) -> Dict[Tuple[int, int], Dict[str, List[int]]]:
        attempts: Dict[Tuple[int, int], Dict[str, List[int]]] = defaultdict(lambda: defaultdict(list))
        for robot in self.robots:
            action = actions.get(robot.id)
            if action == "pick_up" and pick_up(robot, self.world):
                attempts[robot.pos][robot.team].append(robot.id)

        for pos, by_team in attempts.items():
            if pos not in self.world.gold:
                continue
            successes: List[Tuple[str, List[int]]] = []
            for team, ids in by_team.items():
                if len(ids) == 2:
                    successes.append((team, ids))
                elif len(ids) == 1:
                    self.logs.append(f"Robot {ids[0]} (Team {team}) failed pickup at {pos} (needs partner).")
                else:
                    self.logs.append(f"Team {team} failed pickup at {pos} ({len(ids)} robots, need exactly 2).")

            if len(successes) == 2:
                available = sum(1 for g in self.world.gold if g == pos)
                if available < 2:
                    self.logs.append(f"Both teams failed pickup at {pos} (only 1 gold, 4 robots attempted).")
                else:
                    for team, ids in successes:
                        self._complete_pickup(pos, team, ids)
            elif len(successes) == 1:
                team, ids = successes[0]
                self._complete_pickup(pos, team, ids)

        return attempts

    def _complete_pickup(self, pos: Tuple[int, int], team: str, robot_ids: List[int]) -> None:
        if pos not in self.world.gold:
            return
        self.world.gold.remove(pos)
        if team in self.pickups:
            self.pickups[team] += 1
        else:
            self.pickups[team] = 1
        r1 = next(r for r in self.robots if r.id == robot_ids[0])
        r2 = next(r for r in self.robots if r.id == robot_ids[1])
        r1.carrying = True
        r1.carrying_with = r2.id
        r2.carrying = True
        r2.carrying_with = r1.id
        self.logs.append(f"Robots {r1.id} and {r2.id} (Team {team}) picked up gold at {pos}.")

    def _process_deposits(self, actions: Dict[int, Any]) -> None:
        processed = set()
        for robot in self.robots:
            if not robot.carrying or robot.carrying_with is None:
                continue
            if actions.get(robot.id) != "deposit":
                continue
            mate = next((r for r in self.robots if r.id == robot.carrying_with), None)
            if not mate:
                continue
            pair = tuple(sorted((robot.id, mate.id)))
            if pair in processed:
                continue
            if actions.get(mate.id) != "deposit":
                self.logs.append(f"Robot {robot.id} (Team {robot.team}) failed deposit (partner not coordinated).")
                continue
            depot = self.world.deposits.get(robot.team)
            if not depot or robot.pos != depot or mate.pos != depot:
                self.logs.append(f"Robot {robot.id} (Team {robot.team}) failed deposit (not at depot).")
                continue

            robot.carrying = False
            robot.carrying_with = None
            mate.carrying = False
            mate.carrying_with = None
            self.scores[robot.team] += 1
            processed.add(pair)
            self.logs.append(
                f"Robots {robot.id} and {mate.id} (Team {robot.team}) deposited gold. Score: {self.scores[robot.team]}"
            )
