"""
Debug logging helpers mirroring the CPR reference implementation.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from .models import Robot


def debug_log(message: str, robot: Optional[Robot] = None, category: str = "PAXOS") -> str:
    """Emit a formatted debug line and return it for optional aggregation."""
    if robot:
        log_msg = f"[{category}] R{robot.id} {message}"
    else:
        log_msg = f"[{category}] {message}"
    print(log_msg)
    return log_msg


def log_leader_election(robot: Robot, proposal_id: int) -> str:
    return debug_log(f"elected leader, proposal_id={proposal_id}", robot)


def log_plan_proposal(robot: Robot, plan: Dict[str, Any]) -> str:
    pairs: List[str] = []
    seen: set[str] = set()
    for rid, assignment in plan.items():
        if rid in seen:
            continue
        partner = assignment.get("partner")
        gold = assignment.get("gold_pos")
        if partner is None or gold is None:
            continue
        pairs.append(f"{rid},{partner}→{tuple(gold)}")
        seen.add(rid)
        seen.add(str(partner))
    summary = "{" + ", ".join(pairs) + "}"
    return debug_log(f"proposed plan {summary}", robot)


def log_promise(robot: Robot, proposal_id: int, leader_id: int) -> str:
    return debug_log(f"promised proposal {proposal_id} to R{leader_id}", robot)


def log_majority_reached(robot: Robot, promises_count: int, total: int) -> str:
    return debug_log(f"reached majority ({promises_count}/{total}) — committing", robot)


def log_plan_execution(robot: Robot, target_gold: tuple, partner_id: int) -> str:
    return debug_log(f"executing plan (target={target_gold}, partner=R{partner_id})", robot)


def log_message_flow(sender: Robot, message_type: str, recipients: str = "team") -> str:
    return debug_log(f"→ {message_type} to {recipients}", sender, "MSG")


def log_execution_action(robot: Robot, action: str, context: str = "") -> str:
    suffix = f" ({context})" if context else ""
    return debug_log(f"action: {action}{suffix}", robot, "EXEC")


def log_consensus_state(robots: List[Robot], step: int) -> str:
    states: Dict[str, int] = {}
    for robot in robots:
        label = f"{robot.paxos_state}/{robot.execution_state}"
        states[label] = states.get(label, 0) + 1
    summary = ", ".join(f"{state}:{count}" for state, count in states.items())
    return debug_log(f"Step {step} - States: {summary}", category="STATE")
