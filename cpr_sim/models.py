from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

# Canonical facing order
DIRECTIONS = ["N", "E", "S", "W"]


@dataclass
class Message:
    """Team broadcast message queued for delivery on the next tick."""

    sender_id: int
    team: str
    content: Dict[str, Any]
    timestamp: int


@dataclass
class Robot:
    """Lightweight robot state used by the simulation engine."""

    id: int
    team: str
    number: int
    pos: Tuple[int, int]
    facing: str
    carrying: bool = False
    carrying_with: Optional[int] = None
    messages: List[Message] = field(default_factory=list)
    history: List[Dict[str, Any]] = field(default_factory=list)

    # Paxos consensus attributes
    paxos_state: str = "idle"
    proposal_id: int = 0
    promises_received: Set[int] = field(default_factory=set)
    current_plan: Optional[Dict[str, Any]] = None
    leader_id: Optional[int] = None

    # Execution state
    assigned_gold: Optional[Tuple[int, int]] = None
    assigned_partner: Optional[int] = None
    execution_state: str = "idle"

    # Communication state
    pending_broadcast: Optional[Dict[str, Any]] = None
    target_gold_for_consensus: Optional[Tuple[int, int]] = None
    known_gold: Set[Tuple[int, int]] = field(default_factory=set)
    gold_report_pending: Optional[List[Tuple[int, int]]] = None
    control_state: str = "exploring"
    state_entered_step: int = 0
    timers: Dict[str, int] = field(default_factory=dict)
    awaiting_ack: Dict[str, Set[int]] = field(default_factory=dict)
    consensus_deadline: Optional[int] = None
    consensus_retries: int = 0
    handshake_stage: str = "idle"
    handshake_role: str = "idle"
    handshake_index_counter: int = 0
    handshake_active_index: Optional[int] = None
    handshake_partner: Optional[int] = None
    handshake_target: Optional[Tuple[int, int]] = None
    handshake_anchor: Optional[Tuple[int, int]] = None


@dataclass
class GridWorld:
    """Grid topology plus dynamic gold/deposit state."""

    width: int
    height: int
    gold: Set[Tuple[int, int]] = field(default_factory=set)
    deposits: Dict[str, Tuple[int, int]] = field(default_factory=dict)
    broadcast_queue: List[Message] = field(default_factory=list)


@dataclass
class Frame:
    """Complete snapshot of a single engine tick."""

    step: int
    robots: List[Robot]
    gold: List[Tuple[int, int]]
    scores: Dict[str, int]
    deposits: Dict[str, Tuple[int, int]]
    width: int
    height: int
    pickups: Dict[str, int] = field(default_factory=dict)
    logs: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        robot_payloads: List[Dict[str, Any]] = []
        for r in self.robots:
            robot_payloads.append(
                {
                    "id": r.id,
                    "team": r.team,
                    "number": r.number,
                    "pos": list(r.pos),
                    "facing": r.facing,
                    "carrying": r.carrying,
                    "carrying_with": r.carrying_with,
                    "control_state": getattr(r, "control_state", None),
                    "execution_state": getattr(r, "execution_state", None),
                    "paxos_state": getattr(r, "paxos_state", None),
                    "proposal_id": getattr(r, "proposal_id", None),
                    "promises_received": sorted(getattr(r, "promises_received", [])),
                    "leader_id": getattr(r, "leader_id", None),
                    "assigned_gold": list(r.assigned_gold) if getattr(r, "assigned_gold", None) else None,
                    "assigned_partner": getattr(r, "assigned_partner", None),
                    "target_gold_for_consensus": list(r.target_gold_for_consensus)
                    if getattr(r, "target_gold_for_consensus", None)
                    else None,
                    "pending_broadcast": dict(r.pending_broadcast) if getattr(r, "pending_broadcast", None) else None,
                    "known_gold": [list(pos) for pos in sorted(list(r.known_gold))]
                    if getattr(r, "known_gold", None)
                    else [],
                    "current_plan": r.current_plan if getattr(r, "current_plan", None) else None,
                    "timers": dict(r.timers) if getattr(r, "timers", None) else {},
                    "awaiting_ack": {k: sorted(list(v)) for k, v in getattr(r, "awaiting_ack", {}).items()}
                    if getattr(r, "awaiting_ack", None)
                    else {},
                    "consensus_deadline": getattr(r, "consensus_deadline", None),
                    "consensus_retries": getattr(r, "consensus_retries", 0),
                    "handshake_stage": getattr(r, "handshake_stage", None),
                    "handshake_role": getattr(r, "handshake_role", None),
                    "handshake_index_counter": getattr(r, "handshake_index_counter", 0),
                    "handshake_active_index": getattr(r, "handshake_active_index", None),
                    "handshake_partner": getattr(r, "handshake_partner", None),
                    "handshake_target": list(r.handshake_target) if getattr(r, "handshake_target", None) else None,
                    "handshake_anchor": list(r.handshake_anchor) if getattr(r, "handshake_anchor", None) else None,
                }
            )

        return {
            "step": self.step,
            "robots": robot_payloads,
            "gold": [list(g) for g in self.gold],
            "scores": dict(self.scores),
            "pickups": dict(self.pickups),
            "deposits": {team: list(pos) for team, pos in self.deposits.items()},
            "width": self.width,
            "height": self.height,
            "logs": list(self.logs),
        }
