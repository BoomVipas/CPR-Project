"""
Handshake-based coordination strategy derived from Milestone 3's CSP model.

Robots perform a lightweight, message-indexed handshake to agree on a carrying pair
without engaging the full Paxos flow. The protocol is intentionally simple:

Finder (standing on gold):
1. Broadcast a `finder_offer` with a unique index and gold coordinates.
2. Accept the first `helper_response`, send `finder_ack`, and wait for the helper.
3. Once the helper arrives (`helper_here`), emit `start_pickup` so both robots pick up
   in the same tick.

Helper:
1. Respond to the first open `finder_offer` received (`helper_response`).
2. Proceed only when the matching `finder_ack` arrives; otherwise back out.
3. Announce arrival (`helper_here`) and coordinate on `start_pickup`.

This mirrors the CSP specification in the milestone report while fitting the existing
simulation interface (single team broadcast channel with optional latency).
"""

from __future__ import annotations

import random
from typing import Dict, Iterable, List, Optional, Tuple, Union

from .models import GridWorld, Message, Robot

HANDSHAKE_RESPONSE_TIMEOUT = 6
HANDSHAKE_ARRIVAL_TIMEOUT = 10
HANDSHAKE_START_TIMEOUT = 6
HANDSHAKE_HELPER_ACK_TIMEOUT = 6


def handshake_strategy(
    robot: Robot, world: GridWorld, engine
) -> Union[str, Tuple[str, Optional[Dict[str, Any]]]]:
    _process_messages(robot, engine)
    _update_state_from_world(robot, world, engine)
    action = _determine_action(robot, world, engine)
    return _finalise_action(robot, action)


def _process_messages(robot: Robot, engine) -> None:
    current_step = engine.step
    for msg in list(robot.messages):
        payload = msg.content
        mtype = payload.get("type")
        index = payload.get("index")
        finder = payload.get("finder")
        helper = payload.get("helper")

        # Finder-specific handling
        if robot.id == finder and mtype in {"helper_response", "helper_here"}:
            if robot.handshake_active_index != index:
                continue
            if mtype == "helper_response":
                _handle_helper_response(robot, helper, current_step)
            elif mtype == "helper_here":
                _handle_helper_here(robot, helper, payload, engine)
            continue

        # Helper-specific handling
        if robot.id == helper and mtype in {"finder_ack", "start_pickup"}:
            if robot.handshake_active_index != index:
                if mtype == "finder_ack" and payload.get("helper") != robot.id:
                    # Ack meant for someone else using same index -> abort.
                    if robot.handshake_stage == "await_ack":
                        _reset_handshake(robot)
                continue
            if mtype == "finder_ack":
                _handle_finder_ack(robot, payload)
            elif mtype == "start_pickup":
                _handle_start_pickup(robot)
            continue

        if (
            mtype == "finder_ack"
            and robot.handshake_stage == "await_ack"
            and robot.handshake_active_index == index
            and payload.get("helper") != robot.id
        ):
            _reset_handshake(robot)
            continue

        # Potential helper receiving offer
        if mtype == "finder_offer":
            _handle_finder_offer(robot, payload, current_step)

    robot.messages.clear()


def _update_state_from_world(robot: Robot, world: GridWorld, engine) -> None:
    # Drop stale known gold.
    robot.known_gold.intersection_update(world.gold)

    # Finder reset after deposit/drop.
    if robot.handshake_stage in {"sync_pickup", "send_start", "helper_pickup"} and robot.carrying:
        robot.handshake_stage = "carrying"

    if robot.handshake_stage == "carrying" and not robot.carrying:
        # Deposit complete.
        _reset_handshake(robot)

    # Timeout management for finder.
    if robot.handshake_stage == "await_response" and _timer_expired(
        robot, "handshake_wait_response", engine.step
    ):
        robot.handshake_stage = "broadcast_offer"
        robot.handshake_partner = None
        robot.pending_broadcast = None

    if robot.handshake_stage == "await_here" and _timer_expired(
        robot, "handshake_wait_here", engine.step
    ):
        _reset_handshake(robot)

    if robot.handshake_stage == "send_start" and not robot.pending_broadcast:
        payload = {
            "type": "start_pickup",
            "finder": robot.id,
            "helper": robot.handshake_partner,
            "index": robot.handshake_active_index,
        }
        robot.pending_broadcast = payload
        robot.handshake_stage = "sync_pickup"

    # Helper timeouts.
    if robot.handshake_stage == "await_ack" and _timer_expired(
        robot, "handshake_wait_ack", engine.step
    ):
        _reset_handshake(robot)

    if robot.handshake_stage == "waiting_start" and _timer_expired(
        robot, "handshake_wait_start", engine.step
    ):
        _reset_handshake(robot)


def _determine_action(robot: Robot, world: GridWorld, engine) -> str:
    if robot.carrying:
        return _carry_to_deposit(robot, world, engine)

    # Finder sees gold at current position -> initiate handshake.
    if (
        robot.handshake_stage == "idle"
        and not robot.carrying
        and robot.pos in world.gold
    ):
        _start_finder_handshake(robot, engine)

    # Finder broadcast stage.
    if robot.handshake_stage == "broadcast_offer":
        _ensure_offer_broadcast(robot)
        robot.handshake_stage = "await_response"
        _set_timer(robot, "handshake_wait_response", engine.step + HANDSHAKE_RESPONSE_TIMEOUT)
        return "idle"

    if robot.handshake_stage == "await_response":
        return "idle"

    if robot.handshake_stage == "await_here":
        return "idle"

    if robot.handshake_stage == "sync_pickup":
        if robot.pos != robot.handshake_target:
            return _navigate_to_target(robot, robot.handshake_target)
        return "pick_up"

    if robot.handshake_stage == "carrying":
        return _carry_to_deposit(robot, world, engine)

    # Helper stages
    if robot.handshake_stage == "await_ack":
        return "idle"

    if robot.handshake_stage == "move_to_gold":
        if robot.handshake_target:
            if robot.pos == robot.handshake_target:
                # Notify finder that helper arrived.
                payload = {
                    "type": "helper_here",
                    "finder": robot.handshake_partner,
                    "helper": robot.id,
                    "index": robot.handshake_active_index,
                    "pos": list(robot.pos),
                }
                robot.pending_broadcast = payload
                robot.handshake_stage = "waiting_start"
                _set_timer(robot, "handshake_wait_start", engine.step + HANDSHAKE_START_TIMEOUT)
                return "idle"
            return _navigate_to_target(robot, robot.handshake_target)

    if robot.handshake_stage == "waiting_start":
        # Re-affirm presence so finder can hear even with losses.
        payload = {
            "type": "helper_here",
            "finder": robot.handshake_partner,
            "helper": robot.id,
            "index": robot.handshake_active_index,
            "pos": list(robot.pos),
        }
        robot.pending_broadcast = payload
        return "idle"

    if robot.handshake_stage == "helper_pickup":
        if robot.pos != robot.handshake_target:
            return _navigate_to_target(robot, robot.handshake_target)
        return "pick_up"

    # Default exploration behaviour.
    gold_targets = _candidate_gold_targets(robot, world)
    if gold_targets:
        return _navigate_to_target(robot, gold_targets[0])

    return _explore_randomly(robot)


def _handle_finder_offer(robot: Robot, payload: Dict[str, Any], step: int) -> None:
    if robot.team != payload.get("team", robot.team):  # default same team
        return
    if robot.carrying or robot.handshake_stage not in {"idle", "exploring"}:
        return
    finder_id = payload.get("finder")
    gold = payload.get("gold")
    index = payload.get("index")
    if finder_id == robot.id or not isinstance(gold, (list, tuple)) or len(gold) != 2:
        return

    # Adopt helper role.
    robot.handshake_stage = "await_ack"
    robot.handshake_role = "helper"
    robot.handshake_partner = finder_id
    robot.handshake_active_index = index
    robot.handshake_target = (int(gold[0]), int(gold[1]))
    robot.pending_broadcast = {
        "type": "helper_response",
        "finder": finder_id,
        "helper": robot.id,
        "index": index,
    }
    _set_timer(robot, "handshake_wait_ack", step + HANDSHAKE_HELPER_ACK_TIMEOUT)


def _handle_helper_response(robot: Robot, helper_id: Optional[int], step: int) -> None:
    if helper_id is None or robot.handshake_stage != "await_response":
        return
    if robot.handshake_partner is not None:
        return

    robot.handshake_partner = helper_id
    robot.pending_broadcast = {
        "type": "finder_ack",
        "finder": robot.id,
        "helper": helper_id,
        "index": robot.handshake_active_index,
    }
    robot.handshake_stage = "await_here"
    _set_timer(robot, "handshake_wait_here", step + HANDSHAKE_ARRIVAL_TIMEOUT)


def _handle_helper_here(robot: Robot, helper_id: Optional[int], payload: Dict[str, Any], engine) -> None:
    if helper_id != robot.handshake_partner or robot.handshake_stage != "await_here":
        return
    robot.pending_broadcast = {
        "type": "start_pickup",
        "finder": robot.id,
        "helper": helper_id,
        "index": robot.handshake_active_index,
    }
    robot.handshake_stage = "sync_pickup"


def _handle_finder_ack(robot: Robot, payload: Dict[str, Any]) -> None:
    if robot.handshake_stage != "await_ack":
        return
    if payload.get("helper") != robot.id:
        _reset_handshake(robot)
        return
    robot.pending_broadcast = None
    robot.handshake_stage = "move_to_gold"


def _handle_start_pickup(robot: Robot) -> None:
    if robot.handshake_stage not in {"waiting_start", "move_to_gold"}:
        return
    robot.handshake_stage = "helper_pickup"


def _start_finder_handshake(robot: Robot, engine) -> None:
    robot.handshake_role = "finder"
    robot.handshake_index_counter += 1
    robot.handshake_active_index = robot.handshake_index_counter
    robot.handshake_partner = None
    robot.handshake_target = robot.pos
    robot.handshake_stage = "broadcast_offer"
    robot.state_entered_step = engine.step


def _ensure_offer_broadcast(robot: Robot) -> None:
    if robot.handshake_target is None:
        robot.handshake_target = robot.pos
    if robot.pending_broadcast is None:
        robot.pending_broadcast = {
            "type": "finder_offer",
            "finder": robot.id,
            "index": robot.handshake_active_index,
            "gold": list(robot.handshake_target),
            "team": robot.team,
        }


def _reset_handshake(robot: Robot) -> None:
    robot.handshake_stage = "idle"
    robot.handshake_role = "idle"
    robot.handshake_partner = None
    robot.handshake_active_index = None
    robot.handshake_target = None
    robot.pending_broadcast = None
    robot.timers.pop("handshake_wait_response", None)
    robot.timers.pop("handshake_wait_here", None)
    robot.timers.pop("handshake_wait_ack", None)
    robot.timers.pop("handshake_wait_start", None)


def _carry_to_deposit(robot: Robot, world: GridWorld, engine) -> str:
    deposit = world.deposits.get(robot.team)
    if not deposit:
        return "idle"
    if robot.pos == deposit:
        return "deposit"
    return _navigate_to_target(robot, deposit)


def _candidate_gold_targets(robot: Robot, world: GridWorld) -> List[Tuple[int, int]]:
    # Merge known gold and visible gold for exploration priorities.
    visible = _get_visible_gold(robot, world)
    for pos in visible:
        robot.known_gold.add(pos)
    robot.known_gold.intersection_update(world.gold)
    return sorted(robot.known_gold, key=lambda g: _manhattan_distance(robot.pos, g))


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


def _manhattan_distance(a: Tuple[int, int], b: Tuple[int, int]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _set_timer(robot: Robot, name: str, expires_at: int) -> None:
    robot.timers[name] = expires_at


def _timer_expired(robot: Robot, name: str, step: int) -> bool:
    expiry = robot.timers.get(name)
    return expiry is not None and step >= expiry


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
