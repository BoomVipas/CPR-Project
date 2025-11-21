"""
Handshake-based coordination strategy derived from the Milestone 3 CSP model.

Finder
------
1. Discover gold while exploring and stop at an adjacent tile.
2. Broadcast a `found` message with a monotonically increasing index.
3. Accept the first `response`, send an `ack`, and wait for the helper
   to reach the opposite side of the gold (`here`).
4. When the helper arrives, send `ack2`, move onto the gold, and only
   pick up once the helper is also ready.

Helper
------
1. When idle, respond to the first `found` for which it can help.
2. Wait for the matching `ack`, then move to the opposite side.
3. Announce arrival with `here` until `ack2` is heard.
4. Upon `ack2`, move onto the gold and synchronise the pickup.
"""

from __future__ import annotations

import random
from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

from .models import GridWorld, Robot

HANDSHAKE_RESPONSE_TIMEOUT = 6
HANDSHAKE_ARRIVAL_TIMEOUT = 10
HANDSHAKE_START_TIMEOUT = 6
HANDSHAKE_HELPER_ACK_TIMEOUT = 6


def _normalise_stage(robot: Robot) -> None:
    legacy_map = {
        "broadcast_offer": "finder_broadcast",
        "await_response": "finder_wait_response",
        "await_here": "finder_wait_here",
        "send_start": "finder_wait_pickup",
        "sync_pickup": "finder_wait_pickup",
        "await_ack": "helper_wait_ack",
        "move_to_gold": "helper_move_to_gold",
        "waiting_start": "helper_wait_ack2",
        "helper_pickup": "helper_ready_pickup",
    }
    stage = getattr(robot, "handshake_stage", "idle")
    if stage in legacy_map:
        robot.handshake_stage = legacy_map[stage]
        if robot.handshake_stage.startswith("finder"):
            robot.handshake_role = "finder"
        elif robot.handshake_stage.startswith("helper"):
            robot.handshake_role = "helper"


def handshake_strategy(
    robot: Robot, world: GridWorld, engine
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    _normalise_stage(robot)
    if robot.handshake_stage == "idle":
        robot.handshake_stage = "exploring"
        robot.handshake_role = "exploring"
    _process_messages(robot, engine)
    _update_state_from_world(robot, world, engine)
    action = _determine_action(robot, world, engine)
    return _finalise_action(robot, action)


def _process_messages(robot: Robot, engine) -> None:
    current_step = engine.step
    for msg in list(robot.messages):
        payload = msg.content or {}
        mtype = payload.get("type")
        index = payload.get("index")

        if mtype in {"found", "finder_offer"}:
            _handle_found_offer(robot, payload, current_step)
            continue

        if robot.handshake_active_index is None or index != robot.handshake_active_index:
            continue

        if mtype in {"response", "helper_response"} and robot.handshake_role == "finder":
            if robot.handshake_stage == "finder_wait_response":
                _handle_helper_response(robot, payload, current_step)
            continue

        if mtype in {"here", "helper_here"} and robot.handshake_role == "finder":
            if (
                robot.handshake_stage == "finder_wait_here"
                and payload.get("helper") == robot.handshake_partner
            ):
                _handle_helper_here(robot, current_step)
            continue

        if mtype in {"ack", "finder_ack"} and robot.handshake_role == "helper":
            _handle_finder_ack(robot, payload, current_step)
            continue

        if mtype in {"ack2", "start_pickup"} and robot.handshake_role == "helper":
            _handle_finder_ack2(robot, payload)
            continue

        if (
            mtype in {"ack", "finder_ack"}
            and robot.handshake_stage == "helper_wait_ack"
            and payload.get("helper") != robot.id
        ):
            _reset_handshake(robot)

    robot.messages.clear()


def _update_state_from_world(robot: Robot, world: GridWorld, engine) -> None:
    robot.known_gold.intersection_update(world.gold)

    if robot.carrying:
        robot.handshake_stage = "carrying"
    elif robot.handshake_stage == "carrying":
        _reset_handshake(robot)

    step = engine.step

    if robot.handshake_stage == "finder_wait_response" and _timer_expired(
        robot, "handshake_wait_response", step
    ):
        robot.handshake_stage = "finder_broadcast"
        robot.handshake_partner = None
        robot.pending_broadcast = None

    if robot.handshake_stage == "finder_wait_here" and _timer_expired(
        robot, "handshake_wait_here", step
    ):
        _reset_handshake(robot)

    if robot.handshake_stage == "finder_wait_pickup":
        if _timer_expired(robot, "handshake_wait_pickup", step):
            _reset_handshake(robot)
        else:
            _ensure_ack2_broadcast(robot)

    if robot.handshake_stage == "helper_wait_ack" and _timer_expired(
        robot, "handshake_wait_ack", step
    ):
        _reset_handshake(robot)

    if robot.handshake_stage == "helper_wait_ack2":
        if _timer_expired(robot, "handshake_wait_ack2", step):
            _reset_handshake(robot)
        else:
            _ensure_helper_here_broadcast(robot)


def _determine_action(robot: Robot, world: GridWorld, engine) -> str:
    if robot.carrying:
        return _carry_to_deposit(robot, world, engine)

    if robot.handshake_stage == "finder_broadcast":
        _queue_found_broadcast(robot)
        robot.handshake_stage = "finder_wait_response"
        _set_timer(robot, "handshake_wait_response", engine.step + HANDSHAKE_RESPONSE_TIMEOUT)
        return "idle"

    if robot.handshake_stage in {"finder_wait_response", "finder_wait_here"}:
        return "idle"

    if robot.handshake_stage == "finder_wait_pickup":
        if robot.pos != robot.handshake_target:
            return _navigate_to_target(robot, robot.handshake_target)
        if _partner_ready_for_pickup(robot, engine, {"helper_wait_pickup", "helper_ready_pickup", "carrying"}):
            robot.handshake_stage = "finder_ready_pickup"
            return "pick_up"
        return "idle"

    if robot.handshake_stage == "finder_ready_pickup":
        if _partner_ready_for_pickup(robot, engine, {"helper_ready_pickup", "carrying"}):
            return "pick_up"
        return "idle"

    if robot.handshake_stage == "helper_wait_ack":
        return "idle"

    if robot.handshake_stage == "helper_move_to_opposite":
        destination = _helper_opposite_destination(robot, world)
        if destination is None:
            _reset_handshake(robot)
            return "idle"
        if robot.pos == destination:
            robot.handshake_stage = "helper_wait_ack2"
            _set_timer(robot, "handshake_wait_ack2", engine.step + HANDSHAKE_START_TIMEOUT)
            _ensure_helper_here_broadcast(robot)
            return "idle"
        return _navigate_to_target(robot, destination)

    if robot.handshake_stage == "helper_wait_ack2":
        _ensure_helper_here_broadcast(robot)
        return "idle"

    if robot.handshake_stage == "helper_move_to_gold":
        if robot.handshake_target and robot.pos == robot.handshake_target:
            robot.handshake_stage = "helper_wait_pickup"
            return "idle"
        return _navigate_to_target(robot, robot.handshake_target)

    if robot.handshake_stage == "helper_wait_pickup":
        if _partner_ready_for_pickup(robot, engine, {"finder_wait_pickup", "finder_ready_pickup", "carrying"}):
            robot.handshake_stage = "helper_ready_pickup"
        return "idle"

    if robot.handshake_stage == "helper_ready_pickup":
        if robot.handshake_target and robot.pos != robot.handshake_target:
            return _navigate_to_target(robot, robot.handshake_target)
        if _partner_ready_for_pickup(robot, engine, {"finder_ready_pickup", "carrying"}):
            return "pick_up"
        return "idle"

    # Finder discovery logic while exploring.
    gold_targets = _candidate_gold_targets(robot, world)
    if gold_targets and robot.handshake_stage == "exploring":
        target = gold_targets[0]
        distance = _manhattan_distance(robot.pos, target)
        if distance <= 1:
            _start_finder_handshake(robot, target, engine)
            return "idle"
        return _navigate_to_target(robot, target)

    return _explore_randomly(robot)


def _handle_found_offer(robot: Robot, payload: Dict[str, Any], step: int) -> None:
    if robot.team != payload.get("team", robot.team):
        return
    if robot.carrying or robot.handshake_stage not in {"exploring", "idle"}:
        return
    finder_id = payload.get("finder")
    if finder_id == robot.id:
        return
    raw_gold = payload.get("gold")
    if not isinstance(raw_gold, (list, tuple)) or len(raw_gold) != 2:
        return
    gold = (int(raw_gold[0]), int(raw_gold[1]))

    robot.handshake_stage = "helper_wait_ack"
    robot.handshake_role = "helper"
    robot.handshake_partner = finder_id
    robot.handshake_target = gold
    robot.handshake_active_index = payload.get("index")
    anchor = payload.get("finder_pos")
    if isinstance(anchor, (list, tuple)) and len(anchor) == 2:
        robot.handshake_anchor = (int(anchor[0]), int(anchor[1]))
    robot.pending_broadcast = {
        "type": "response",
        "finder": finder_id,
        "helper": robot.id,
        "index": robot.handshake_active_index,
    }
    robot.known_gold.add(gold)
    _set_timer(robot, "handshake_wait_ack", step + HANDSHAKE_HELPER_ACK_TIMEOUT)


def _handle_helper_response(robot: Robot, payload: Dict[str, Any], step: int) -> None:
    helper_id = payload.get("helper")
    if helper_id is None or robot.handshake_partner is not None:
        return
    robot.handshake_partner = helper_id
    robot.pending_broadcast = {
        "type": "ack",
        "finder": robot.id,
        "helper": helper_id,
        "index": robot.handshake_active_index,
        "finder_pos": list(robot.handshake_anchor or robot.pos),
    }
    robot.handshake_stage = "finder_wait_here"
    _set_timer(robot, "handshake_wait_here", step + HANDSHAKE_ARRIVAL_TIMEOUT)


def _handle_helper_here(robot: Robot, current_step: int) -> None:
    robot.pending_broadcast = {
        "type": "ack2",
        "finder": robot.id,
        "helper": robot.handshake_partner,
        "index": robot.handshake_active_index,
    }
    robot.handshake_stage = "finder_wait_pickup"
    _set_timer(robot, "handshake_wait_pickup", current_step + HANDSHAKE_START_TIMEOUT)


def _handle_finder_ack(robot: Robot, payload: Dict[str, Any], step: int) -> None:
    if robot.handshake_stage != "helper_wait_ack":
        return
    if payload.get("helper") != robot.id:
        _reset_handshake(robot)
        return
    anchor = payload.get("finder_pos")
    if isinstance(anchor, (list, tuple)) and len(anchor) == 2:
        robot.handshake_anchor = (int(anchor[0]), int(anchor[1]))
    robot.pending_broadcast = None
    robot.handshake_stage = "helper_move_to_opposite"


def _handle_finder_ack2(robot: Robot, payload: Dict[str, Any]) -> None:
    if robot.handshake_stage != "helper_wait_ack2":
        return
    if payload.get("helper") != robot.id:
        return
    robot.timers.pop("handshake_wait_ack2", None)
    robot.pending_broadcast = None
    robot.handshake_stage = "helper_move_to_gold"


def _start_finder_handshake(robot: Robot, target: Tuple[int, int], engine) -> None:
    robot.handshake_role = "finder"
    robot.handshake_stage = "finder_broadcast"
    robot.handshake_partner = None
    robot.handshake_target = target
    robot.handshake_anchor = robot.pos
    robot.handshake_index_counter += 1
    robot.handshake_active_index = robot.handshake_index_counter
    robot.pending_broadcast = None
    robot.state_entered_step = engine.step


def _queue_found_broadcast(robot: Robot) -> None:
    if robot.handshake_target is None:
        return
    if robot.pending_broadcast is not None:
        return
    robot.pending_broadcast = {
        "type": "found",
        "finder": robot.id,
        "index": robot.handshake_active_index,
        "gold": list(robot.handshake_target),
        "team": robot.team,
        "finder_pos": list(robot.handshake_anchor or robot.pos),
    }


def _ensure_ack2_broadcast(robot: Robot) -> None:
    if robot.pending_broadcast is not None or robot.handshake_partner is None:
        return
    robot.pending_broadcast = {
        "type": "ack2",
        "finder": robot.id,
        "helper": robot.handshake_partner,
        "index": robot.handshake_active_index,
    }


def _ensure_helper_here_broadcast(robot: Robot) -> None:
    if robot.pending_broadcast is not None or robot.handshake_partner is None:
        return
    robot.pending_broadcast = {
        "type": "here",
        "finder": robot.handshake_partner,
        "helper": robot.id,
        "index": robot.handshake_active_index,
        "pos": list(robot.pos),
    }


def _reset_handshake(robot: Robot) -> None:
    robot.handshake_stage = "idle"
    robot.handshake_role = "idle"
    robot.handshake_partner = None
    robot.handshake_active_index = None
    robot.handshake_target = None
    robot.handshake_anchor = None
    robot.pending_broadcast = None
    robot.timers.pop("handshake_wait_response", None)
    robot.timers.pop("handshake_wait_here", None)
    robot.timers.pop("handshake_wait_pickup", None)
    robot.timers.pop("handshake_wait_ack", None)
    robot.timers.pop("handshake_wait_ack2", None)


def _carry_to_deposit(robot: Robot, world: GridWorld, engine) -> str:
    deposit = world.deposits.get(robot.team)
    if not deposit:
        return "idle"
    if robot.pos == deposit:
        return "deposit"
    return _navigate_to_target(robot, deposit)


def _candidate_gold_targets(robot: Robot, world: GridWorld) -> List[Tuple[int, int]]:
    visible = _get_visible_gold(robot, world)
    for pos in visible:
        robot.known_gold.add(pos)
    robot.known_gold.intersection_update(world.gold)
    return sorted(robot.known_gold, key=lambda g: _manhattan_distance(robot.pos, g))


def _navigate_to_target(robot: Robot, target: Tuple[int, int]) -> str:
    if target is None:
        return "idle"
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


def _explore_randomly(robot: Robot) -> str:
    return random.choice(["turn_left", "turn_right", "move_forward"])


def _get_visible_gold(robot: Robot, world: GridWorld) -> List[Tuple[int, int]]:
    x, y = robot.pos
    deltas = {
        "N": [(-1, -1), (0, -1), (1, -1), (-2, -2), (-1, -2), (0, -2), (1, -2), (2, -2)],
        "S": [(-1, 1), (0, 1), (1, 1), (-2, 2), (-1, 2), (0, 2), (1, 2), (2, 2)],
        "E": [(1, -1), (1, 0), (1, 1), (2, -2), (2, -1), (2, 0), (2, 1), (2, 2)],
        "W": [(-1, -1), (-1, 0), (-1, 1), (-2, -2), (-2, -1), (-2, 0), (-2, 1), (-2, 2)],
    }
    visibles = []
    for dx, dy in deltas.get(robot.facing, []):
        nx, ny = x + dx, y + dy
        if 0 <= nx < world.width and 0 <= ny < world.height and (nx, ny) in world.gold:
            visibles.append((nx, ny))
    return visibles


def _helper_opposite_destination(robot: Robot, world: GridWorld) -> Optional[Tuple[int, int]]:
    if robot.handshake_target is None:
        return None
    anchor = robot.handshake_anchor
    gold = robot.handshake_target
    if anchor is None:
        return _closest_adjacent(robot.pos, gold, world)
    dx = gold[0] - anchor[0]
    dy = gold[1] - anchor[1]
    candidate = (gold[0] + dx, gold[1] + dy)
    if _in_bounds(candidate, world):
        return candidate
    for offset in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        adj = (gold[0] + offset[0], gold[1] + offset[1])
        if adj != anchor and _in_bounds(adj, world):
            return adj
    return _closest_adjacent(robot.pos, gold, world)


def _closest_adjacent(current: Tuple[int, int], gold: Tuple[int, int], world: GridWorld) -> Optional[Tuple[int, int]]:
    candidates = []
    for offset in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        adj = (gold[0] + offset[0], gold[1] + offset[1])
        if _in_bounds(adj, world):
            candidates.append(adj)
    if not candidates:
        return None
    return min(candidates, key=lambda pos: _manhattan_distance(current, pos))


def _in_bounds(pos: Tuple[int, int], world: GridWorld) -> bool:
    return 0 <= pos[0] < world.width and 0 <= pos[1] < world.height


def _manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _set_timer(robot: Robot, name: str, expires_at: int) -> None:
    robot.timers[name] = expires_at


def _timer_expired(robot: Robot, name: str, step: int) -> bool:
    expiry = robot.timers.get(name)
    return expiry is not None and step >= expiry


def _get_robot_by_id(engine, robot_id: Optional[int]) -> Optional[Robot]:
    if robot_id is None:
        return None
    return next((r for r in engine.robots if r.id == robot_id), None)


def _partner_ready_for_pickup(robot: Robot, engine, expected_stages: Iterable[str]) -> bool:
    partner = _get_robot_by_id(engine, robot.handshake_partner)
    if not partner:
        return False
    if robot.handshake_target and partner.pos != robot.handshake_target:
        return False
    if partner.carrying:
        return True
    partner_stage = getattr(partner, "handshake_stage", "idle")
    return partner_stage in expected_stages


def _finalise_action(
    robot: Robot, action: Union[str, Tuple[str, Optional[Dict[str, Any]]]]
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    payload = robot.pending_broadcast
    robot.pending_broadcast = None
    if isinstance(action, tuple):
        act, existing = action
        if existing is None and payload is not None:
            return act, payload
        return action
    if payload is not None:
        return action, payload
    return action
