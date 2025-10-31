"""
Paxos-based coordination strategy for multi-robot gold collection.

Mirrors the reference implementation from CPR-Term-Project, adapted to this workspace.
"""

from __future__ import annotations

import random
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple, Union

from .debug import (
    debug_log,
    log_execution_action,
    log_leader_election,
    log_majority_reached,
    log_message_flow,
    log_plan_execution,
    log_plan_proposal,
    log_promise,
)
from .models import GridWorld, Message, Robot


CONSENSUS_TIMEOUT = 4
MAX_PREPARE_RETRIES = 3
PARTNER_WAIT_TIMEOUT = 6


def paxos_strategy(
    robot: Robot, world: GridWorld, engine
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    """Main Paxos strategy entry point."""

    _process_messages(robot, engine)
    _update_control_state(robot, engine)

    visible_gold = _get_visible_gold(robot, world)
    newly_seen = _add_known_gold(robot, world, visible_gold)
    if newly_seen:
        _queue_gold_report(robot, newly_seen)

    candidate_gold = _select_candidate_gold(robot, world, visible_gold, engine)

    if robot.paxos_state == "idle":
        result: Union[str, Tuple[str, Optional[Dict[str, Any]]]] = _handle_idle_state(
            robot, world, engine, candidate_gold
        )
    elif robot.paxos_state == "prepare":
        result = _handle_prepare_state(robot, world, engine)
    elif robot.paxos_state == "promising":
        result = _handle_promising_state(robot, engine)
    elif robot.paxos_state == "committing":
        result = _handle_committing_state(robot, engine)
    elif robot.paxos_state == "finished":
        result = _handle_execution_state(robot, world, engine)
    else:
        result = _explore_randomly(robot)

    return _finalize_action(robot, engine, result)


# ---------------------------------------------------------------------------#
# Message handling                                                           #
# ---------------------------------------------------------------------------#

def _process_messages(robot: Robot, engine) -> None:
    for msg in list(robot.messages):
        kind = msg.content.get("type")
        if kind == "prepare":
            _handle_prepare_message(robot, msg, engine)
        elif kind == "promise":
            _handle_promise_message(robot, msg, engine)
        elif kind == "commit":
            _handle_commit_message(robot, msg, engine)
        elif kind == "gold_report":
            _handle_gold_report(robot, msg, engine)
    robot.messages.clear()


def _handle_prepare_message(robot: Robot, msg: Message, engine) -> None:
    proposal_id = msg.content.get("proposal_id", 0)
    leader_id = msg.sender_id
    if proposal_id >= robot.proposal_id:
        robot.proposal_id = proposal_id
        robot.leader_id = leader_id
        robot.paxos_state = "promising"
        _set_control_state(robot, engine, "coordinating")
        _set_timer(robot, "await_commit", engine.step + CONSENSUS_TIMEOUT)
        log_promise(robot, proposal_id, leader_id)
        robot.pending_broadcast = {
            "type": "promise",
            "proposal_id": proposal_id,
            "sender_id": robot.id,
        }


def _handle_promise_message(robot: Robot, msg: Message, engine) -> None:
    if robot.paxos_state != "prepare":
        return
    proposal_id = msg.content.get("proposal_id", 0)
    sender_id = msg.content.get("sender_id")
    if proposal_id != robot.proposal_id or sender_id is None:
        return

    robot.promises_received.add(sender_id)
    _resolve_ack(robot, "promise", sender_id)
    teammates = _get_teammates(robot, engine)
    majority = len(teammates) // 2 + 1
    if len(robot.promises_received) >= majority:
        robot.paxos_state = "committing"
        robot.consensus_deadline = engine.step + CONSENSUS_TIMEOUT
        robot.consensus_retries = 0
        log_majority_reached(robot, len(robot.promises_received), len(teammates))
        robot.pending_broadcast = {
            "type": "commit",
            "proposal_id": robot.proposal_id,
            "plan": robot.current_plan,
        }


def _handle_commit_message(robot: Robot, msg: Message, engine) -> None:
    proposal_id = msg.content.get("proposal_id", 0)
    plan = msg.content.get("plan", {})
    if proposal_id != robot.proposal_id:
        return
    robot.current_plan = plan
    robot.paxos_state = "finished"
    robot.consensus_deadline = None
    robot.consensus_retries = 0
    _cancel_timer(robot, "await_commit")
    robot.awaiting_ack.pop("promise", None)
    if str(robot.id) in plan:
        assignment = plan[str(robot.id)]
        robot.assigned_gold = tuple(assignment.get("gold_pos", (0, 0)))
        if robot.assigned_gold:
            robot.known_gold.discard(robot.assigned_gold)
        robot.assigned_partner = assignment.get("partner")
        robot.execution_state = "moving_to_gold"
        _set_control_state(robot, engine, "seeking_partner")
        if robot.assigned_partner is not None:
            log_plan_execution(robot, robot.assigned_gold, robot.assigned_partner)
    else:
        _reset_robot_to_idle(robot, engine)


# ---------------------------------------------------------------------------#
# Paxos state handlers                                                       #
# ---------------------------------------------------------------------------#

def _handle_idle_state(
    robot: Robot,
    world: GridWorld,
    engine,
    candidate_gold: List[Tuple[int, int]],
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    teammates = _get_teammates(robot, engine)
    idle_teammates = [r for r in teammates if r.paxos_state == "idle" and r.execution_state == "idle"]
    idle_teammates.append(robot)

    if candidate_gold:
        active_leaders = [r for r in teammates if r.paxos_state in {"prepare", "promising", "committing"}]
        max_leaders = min(5, len(idle_teammates) // 3 + 1)
        primary_target = candidate_gold[0]
        if len(active_leaders) < max_leaders:
            if not any(
                getattr(leader, "target_gold_for_consensus", None) == primary_target for leader in active_leaders
            ):
                robot.target_gold_for_consensus = primary_target
                return _become_leader(robot, world, engine, candidate_gold)

        action = _navigate_to_target(robot, primary_target)
        log_execution_action(robot, action, f"moving toward known gold at {primary_target}")
        return action

    return _explore_randomly(robot)


def _become_leader(
    robot: Robot,
    world: GridWorld,
    engine,
    candidate_gold: List[Tuple[int, int]],
) -> Tuple[str, Optional[Dict[str, Any]]]:
    robot.proposal_id = robot.id * 1000 + random.randint(1, 999)
    robot.paxos_state = "prepare"
    robot.promises_received = {robot.id}
    robot.consensus_retries = 0
    robot.consensus_deadline = engine.step + CONSENSUS_TIMEOUT
    _set_control_state(robot, engine, "coordinating")

    teammates = _get_teammates(robot, engine)
    idle_teammates = [r for r in teammates if r.paxos_state == "idle" and r.execution_state == "idle"]
    idle_teammates.append(robot)
    plan = _generate_plan(world, idle_teammates, candidate_gold, engine.step)
    robot.current_plan = plan
    recipients = [r.id for r in idle_teammates if r.id != robot.id]
    _expect_ack(robot, "promise", recipients)

    log_leader_election(robot, robot.proposal_id)
    log_plan_proposal(robot, plan)

    if not plan:
        _reset_robot_to_idle(robot, engine)
        return _explore_randomly(robot), None

    prepare_msg = {"type": "prepare", "proposal_id": robot.proposal_id, "plan": plan}
    log_message_flow(robot, "prepare")
    return "idle", prepare_msg


def _handle_prepare_state(
    robot: Robot, world: GridWorld, engine
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    if robot.pending_broadcast is not None:
        payload = robot.pending_broadcast
        robot.pending_broadcast = None
        return _explore_randomly(robot), payload
    if robot.consensus_deadline and engine.step >= robot.consensus_deadline:
        return _retry_prepare(robot, world, engine)
    return _explore_randomly(robot), None


def _retry_prepare(
    robot: Robot, world: GridWorld, engine
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    if robot.consensus_retries >= MAX_PREPARE_RETRIES:
        _reset_robot_to_idle(robot, engine)
        return _explore_randomly(robot), None

    robot.consensus_retries += 1
    robot.proposal_id += 1
    robot.promises_received = {robot.id}
    candidate_gold = _select_candidate_gold(robot, world, [], engine)
    teammates = _get_teammates(robot, engine)
    idle_teammates = [r for r in teammates if r.paxos_state == "idle" and r.execution_state == "idle"]
    idle_teammates.append(robot)
    plan = _generate_plan(world, idle_teammates, candidate_gold, engine.step)
    if not plan:
        _reset_robot_to_idle(robot, engine)
        return _explore_randomly(robot), None

    robot.current_plan = plan
    recipients = [r.id for r in idle_teammates if r.id != robot.id]
    _expect_ack(robot, "promise", recipients)
    robot.consensus_deadline = engine.step + CONSENSUS_TIMEOUT
    log_plan_proposal(robot, plan)
    payload = {"type": "prepare", "proposal_id": robot.proposal_id, "plan": plan}
    log_message_flow(robot, "prepare(retry)")
    return _explore_randomly(robot), payload


def _handle_promising_state(robot: Robot, engine) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    if robot.pending_broadcast is not None:
        payload = robot.pending_broadcast
        robot.pending_broadcast = None
        log_message_flow(robot, "promise", f"R{robot.leader_id}")
        return _explore_randomly(robot), payload
    if _timer_expired(robot, "await_commit", engine.step):
        _reset_robot_to_idle(robot, engine)
    return _explore_randomly(robot)


def _handle_committing_state(robot: Robot, engine) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    if robot.pending_broadcast is not None:
        payload = robot.pending_broadcast
        robot.pending_broadcast = None
        robot.paxos_state = "finished"
        if robot.current_plan and str(robot.id) in robot.current_plan:
            assignment = robot.current_plan[str(robot.id)]
            robot.assigned_gold = tuple(assignment.get("gold_pos", (0, 0)))
            robot.assigned_partner = assignment.get("partner")
            robot.execution_state = "moving_to_gold"
            _set_control_state(robot, engine, "seeking_partner")
        else:
            _reset_robot_to_idle(robot, engine)
        log_message_flow(robot, "commit")
        robot.consensus_deadline = None
        return _explore_randomly(robot), payload
    if robot.consensus_deadline and engine.step >= robot.consensus_deadline and robot.current_plan:
        robot.pending_broadcast = {
            "type": "commit",
            "proposal_id": robot.proposal_id,
            "plan": robot.current_plan,
        }
        robot.consensus_deadline = engine.step + CONSENSUS_TIMEOUT
    return _explore_randomly(robot), None


def _handle_execution_state(robot: Robot, world: GridWorld, engine) -> str:
    if robot.carrying and robot.execution_state != "carrying":
        robot.execution_state = "carrying"
        log_execution_action(robot, "carrying gold", f"with R{robot.carrying_with}")

    if robot.execution_state == "moving_to_gold":
        return _execute_move_to_gold(robot, world, engine)
    if robot.execution_state == "waiting_at_gold":
        return _execute_wait_at_gold(robot, world, engine)
    if robot.execution_state == "carrying":
        return _execute_carrying(robot, world, engine)
    if robot.execution_state == "depositing":
        return _execute_depositing(robot, world, engine)

    _reset_robot_to_idle(robot, engine)
    return "idle"


# ---------------------------------------------------------------------------#
# Execution helpers                                                          #
# ---------------------------------------------------------------------------#

def _execute_move_to_gold(robot: Robot, world: GridWorld, engine) -> str:
    if not robot.assigned_gold or robot.assigned_gold not in world.gold:
        _reset_robot_to_idle(robot, engine)
        return "idle"

    if robot.pos == robot.assigned_gold:
        robot.execution_state = "waiting_at_gold"
        _set_control_state(robot, engine, "seeking_partner")
        _set_timer(robot, "wait_partner", engine.step + PARTNER_WAIT_TIMEOUT)
        log_execution_action(robot, "idle", f"waiting for R{robot.assigned_partner}")
        return "idle"

    action = _navigate_to_target(robot, robot.assigned_gold)
    _set_control_state(robot, engine, "seeking_partner")
    log_execution_action(robot, action, f"moving to {robot.assigned_gold}")
    return action


def _execute_wait_at_gold(robot: Robot, world: GridWorld, engine) -> str:
    if not robot.assigned_gold or robot.assigned_gold not in world.gold:
        _reset_robot_to_idle(robot, engine)
        return "idle"

    partner = _get_robot_by_id(engine, robot.assigned_partner)
    if partner and partner.pos == robot.pos:
        if partner.assigned_gold == robot.assigned_gold and partner.assigned_partner == robot.id:
            log_execution_action(robot, "pick_up", f"with R{robot.assigned_partner}")
            return "pick_up"
    if _timer_expired(robot, "wait_partner", engine.step):
        _reset_robot_to_idle(robot, engine)
    return "idle"


def _execute_carrying(robot: Robot, world: GridWorld, engine) -> str:
    if not robot.carrying:
        _reset_robot_to_idle(robot, engine)
        return "idle"
    _cancel_timer(robot, "wait_partner")

    deposit = world.deposits.get(robot.team)
    if not deposit:
        return "idle"
    if robot.pos == deposit:
        robot.execution_state = "depositing"
        return "deposit"

    partner = _get_robot_by_id(engine, robot.carrying_with)
    if not partner or not partner.carrying:
        _reset_robot_to_idle(robot, engine)
        return "idle"

    midpoint = ((robot.pos[0] + partner.pos[0]) // 2, (robot.pos[1] + partner.pos[1]) // 2)
    dx = deposit[0] - midpoint[0]
    dy = deposit[1] - midpoint[1]
    if abs(dx) > abs(dy):
        desired = "E" if dx > 0 else "W"
    else:
        desired = "N" if dy < 0 else "S"

    if robot.facing != desired:
        directions = ["N", "E", "S", "W"]
        current_idx = directions.index(robot.facing)
        target_idx = directions.index(desired)
        diff = (target_idx - current_idx) % 4
        action = "turn_right" if diff in {1, 2} else "turn_left"
    else:
        action = "move_forward"
    _set_control_state(robot, engine, "carrying")
    log_execution_action(robot, action, f"coordinated carry with R{partner.id} toward deposit")
    return action


def _execute_depositing(robot: Robot, world: GridWorld, engine) -> str:
    _set_control_state(robot, engine, "depositing")
    log_execution_action(robot, "deposit", "resetting to idle")
    _reset_robot_to_idle(robot, engine)
    return "deposit"


# ---------------------------------------------------------------------------#
# Plan generation                                                            #
# ---------------------------------------------------------------------------#

def _generate_plan(
    world: GridWorld,
    idle_robots: List[Robot],
    candidate_gold: List[Tuple[int, int]],
    step: int,
) -> Dict[str, Dict[str, Any]]:
    plan: Dict[str, Dict[str, Any]] = {}
    if not idle_robots:
        return plan

    ordered_robots = sorted(idle_robots, key=lambda r: r.id)
    rotation = step % len(ordered_robots) if ordered_robots else 0
    ordered_robots = ordered_robots[rotation:] + ordered_robots[:rotation]

    available_gold = [g for g in candidate_gold if g in world.gold]
    available_gold.sort()
    if available_gold:
        gold_rotation = step % len(available_gold)
        available_gold = available_gold[gold_rotation:] + available_gold[:gold_rotation]

    queue = ordered_robots[:]
    while len(queue) >= 2 and available_gold:
        r1 = queue.pop(0)
        available_gold.sort(key=lambda g: _manhattan_distance(r1.pos, g))
        gold = available_gold.pop(0)
        partner = min(queue, key=lambda r: _manhattan_distance(r.pos, gold))
        if partner is None:
            break
        queue.remove(partner)

        plan[str(r1.id)] = {"partner": partner.id, "gold_pos": list(gold)}
        plan[str(partner.id)] = {"partner": r1.id, "gold_pos": list(gold)}

    return plan


# ---------------------------------------------------------------------------#
# Utility helpers                                                            #
# ---------------------------------------------------------------------------#


def _add_known_gold(
    robot: Robot, world: GridWorld, positions: Iterable[Tuple[int, int]]
) -> List[Tuple[int, int]]:
    robot.known_gold.intersection_update(world.gold)
    new_spots: List[Tuple[int, int]] = []
    for raw in positions:
        if isinstance(raw, (list, tuple)) and len(raw) == 2:
            tup = (int(raw[0]), int(raw[1]))
            if tup in world.gold and tup not in robot.known_gold:
                robot.known_gold.add(tup)
                new_spots.append(tup)
    return new_spots


def _queue_gold_report(robot: Robot, new_spots: List[Tuple[int, int]]) -> None:
    if not new_spots:
        return
    pending = list(robot.gold_report_pending) if robot.gold_report_pending else []
    existing = set(pending)
    for spot in new_spots:
        if spot not in existing:
            pending.append(spot)
            existing.add(spot)
    robot.gold_report_pending = pending


def _reserved_gold_positions(robot: Robot, engine) -> Set[Tuple[int, int]]:
    reserved: Set[Tuple[int, int]] = set()
    for teammate in engine.robots:
        if teammate.team != robot.team:
            continue
        if teammate.assigned_gold and teammate.execution_state in {
            "moving_to_gold",
            "waiting_at_gold",
            "carrying",
            "depositing",
        }:
            reserved.add(tuple(teammate.assigned_gold))
        target = getattr(teammate, "target_gold_for_consensus", None)
        if target:
            reserved.add(tuple(target))
        plan = getattr(teammate, "current_plan", None)
        if plan and teammate.paxos_state in {"prepare", "committing"}:
            for assignment in plan.values():
                gold = assignment.get("gold_pos") if isinstance(assignment, dict) else None
                if isinstance(gold, (list, tuple)) and len(gold) == 2:
                    reserved.add((int(gold[0]), int(gold[1])))
    return reserved


def _select_candidate_gold(
    robot: Robot,
    world: GridWorld,
    visible_gold: List[Tuple[int, int]],
    engine,
) -> List[Tuple[int, int]]:
    robot.known_gold.intersection_update(world.gold)
    reserved = _reserved_gold_positions(robot, engine)

    def _normalise_positions(candidates: Iterable[Tuple[int, int]]) -> List[Tuple[int, int]]:
        result: List[Tuple[int, int]] = []
        for raw in candidates:
            if isinstance(raw, (list, tuple)) and len(raw) == 2:
                pos = (int(raw[0]), int(raw[1]))
                if pos in world.gold and pos not in reserved:
                    result.append(pos)
        return result

    known_candidates = _normalise_positions(robot.known_gold)
    if known_candidates:
        return sorted(known_candidates, key=lambda g: _manhattan_distance(robot.pos, g))

    visible_candidates = _normalise_positions(visible_gold)
    if visible_candidates:
        return sorted(visible_candidates, key=lambda g: _manhattan_distance(robot.pos, g))

    world_candidates = _normalise_positions(world.gold)
    return sorted(world_candidates, key=lambda g: _manhattan_distance(robot.pos, g)) if world_candidates else []


def _handle_gold_report(robot: Robot, msg: Message, engine) -> None:
    positions = msg.content.get("positions", [])
    if not isinstance(positions, Iterable):
        return
    tuples = [
        (int(pos[0]), int(pos[1]))
        for pos in positions
        if isinstance(pos, (list, tuple)) and len(pos) == 2
    ]
    if tuples:
        _add_known_gold(robot, engine.world, tuples)


def _finalize_action(
    robot: Robot,
    engine,
    result: Union[str, Tuple[str, Optional[Dict[str, Any]]]],
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    if not robot.gold_report_pending:
        return result

    payload = {
        "type": "gold_report",
        "positions": [list(pos) for pos in robot.gold_report_pending],
    }

    if isinstance(result, tuple):
        action, existing = result
        if existing is None:
            robot.gold_report_pending = None
            return action, payload
        return result

    robot.gold_report_pending = None
    return result, payload

def _get_visible_gold(robot: Robot, world: GridWorld) -> List[Tuple[int, int]]:
    x, y = robot.pos
    if robot.facing == "N":
        front = [(x - 1, y - 1), (x, y - 1), (x + 1, y - 1)]
        far = [(x - 2, y - 2), (x - 1, y - 2), (x, y - 2), (x + 1, y - 2), (x + 2, y - 2)]
    elif robot.facing == "S":
        front = [(x - 1, y + 1), (x, y + 1), (x + 1, y + 1)]
        far = [(x - 2, y + 2), (x - 1, y + 2), (x, y + 2), (x + 1, y + 2), (x + 2, y + 2)]
    elif robot.facing == "E":
        front = [(x + 1, y - 1), (x + 1, y), (x + 1, y + 1)]
        far = [(x + 2, y - 2), (x + 2, y - 1), (x + 2, y), (x + 2, y + 1), (x + 2, y + 2)]
    else:
        front = [(x - 1, y - 1), (x - 1, y), (x - 1, y + 1)]
        far = [(x - 2, y - 2), (x - 2, y - 1), (x - 2, y), (x - 2, y + 1), (x - 2, y + 2)]

    visible: List[Tuple[int, int]] = []
    for pos in front + far:
        if 0 <= pos[0] < world.width and 0 <= pos[1] < world.height and pos in world.gold:
            visible.append(pos)
    return visible


def _get_teammates(robot: Robot, engine) -> List[Robot]:
    return [r for r in engine.robots if r.team == robot.team and r.id != robot.id]


def _get_robot_by_id(engine, robot_id: Optional[int]) -> Optional[Robot]:
    if robot_id is None:
        return None
    return next((r for r in engine.robots if r.id == robot_id), None)


def _navigate_to_target(robot: Robot, target: Tuple[int, int]) -> str:
    dx = target[0] - robot.pos[0]
    dy = target[1] - robot.pos[1]
    if dx == 0 and dy == 0:
        return "idle"
    if abs(dx) > abs(dy):
        desired = "E" if dx > 0 else "W"
    else:
        desired = "S" if dy > 0 else "N"

    directions = ["N", "E", "S", "W"]
    if robot.facing != desired:
        current_idx = directions.index(robot.facing)
        target_idx = directions.index(desired)
        diff = (target_idx - current_idx) % 4
        return "turn_right" if diff in {1, 2} else "turn_left"
    return "move_forward"


def _manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _explore_randomly(robot: Robot) -> str:
    if random.random() < 0.3:
        return random.choice(["turn_left", "turn_right"])
    return "move_forward"


def _set_control_state(robot: Robot, engine, state: str) -> None:
    if robot.control_state != state:
        robot.control_state = state
        robot.state_entered_step = engine.step


def _set_timer(robot: Robot, name: str, expires_at: int) -> None:
    robot.timers[name] = expires_at


def _cancel_timer(robot: Robot, name: str) -> None:
    robot.timers.pop(name, None)


def _timer_expired(robot: Robot, name: str, current_step: int) -> bool:
    expiry = robot.timers.get(name)
    return expiry is not None and expiry <= current_step


def _expect_ack(robot: Robot, key: str, recipients: List[int]) -> None:
    robot.awaiting_ack[key] = set(recipients)


def _resolve_ack(robot: Robot, key: str, sender_id: int) -> None:
    pending = robot.awaiting_ack.get(key)
    if not pending:
        return
    pending.discard(sender_id)
    if not pending:
        robot.awaiting_ack.pop(key, None)


def _update_control_state(robot: Robot, engine) -> None:
    if robot.carrying:
        _set_control_state(robot, engine, "carrying")
    elif robot.execution_state == "depositing":
        _set_control_state(robot, engine, "depositing")
    elif robot.execution_state in {"moving_to_gold", "waiting_at_gold"}:
        _set_control_state(robot, engine, "seeking_partner")
    elif robot.paxos_state in {"prepare", "promising", "committing"}:
        _set_control_state(robot, engine, "coordinating")
    else:
        _set_control_state(robot, engine, "exploring")


def _reset_robot_to_idle(robot: Robot, engine) -> None:
    robot.paxos_state = "idle"
    robot.execution_state = "idle"
    robot.assigned_gold = None
    robot.assigned_partner = None
    robot.current_plan = None
    robot.promises_received.clear()
    robot.leader_id = None
    robot.target_gold_for_consensus = None
    robot.pending_broadcast = None
    robot.control_state = "exploring"
    robot.state_entered_step = engine.step
    robot.timers.clear()
    robot.awaiting_ack.clear()
    robot.consensus_deadline = None
    robot.consensus_retries = 0
