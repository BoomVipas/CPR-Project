"""
Cyber Physical Game analyser that treats the CPS as the tuple {E, C, M} and
approximates its evolution with a Probabilistic Finite Automaton (PFA).

* Capture the shared environment properties (E) that evolve each tick.
* Track every cyber agent (C) and their message channels (M).
* Build a PFA over the observed state sequence to estimate transition probabilities.
"""

from __future__ import annotations

import json
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .models import Frame, Robot


@dataclass(frozen=True)
class CyberPhysicalGameState:
    """Coarse-grained snapshot of the environment properties (E)."""

    gold_remaining: int
    score_a: int
    score_b: int
    carrying_pairs_a: int
    carrying_pairs_b: int
    consensus_pressure: int
    handshake_pressure: int

    def label(self) -> str:
        """Return a compact identifier for the state."""
        return (
            f"G={self.gold_remaining}|A={self.score_a}|B={self.score_b}|"
            f"CA={self.carrying_pairs_a}|CB={self.carrying_pairs_b}|"
            f"PX={self.consensus_pressure}|HS={self.handshake_pressure}"
        )

    def as_dict(self) -> Dict[str, int]:
        return {
            "gold_remaining": self.gold_remaining,
            "score_a": self.score_a,
            "score_b": self.score_b,
            "carrying_pairs_a": self.carrying_pairs_a,
            "carrying_pairs_b": self.carrying_pairs_b,
            "consensus_pressure": self.consensus_pressure,
            "handshake_pressure": self.handshake_pressure,
        }

    @classmethod
    def from_frame(cls, frame: Frame) -> "CyberPhysicalGameState":
        def _count_pairs(team: str) -> int:
            carrying = sum(1 for robot in frame.robots if robot.team == team and robot.carrying)
            return carrying // 2

        total_agents = max(1, len(frame.robots))
        paxos_active = sum(
            1
            for robot in frame.robots
            if getattr(robot, "paxos_state", "idle") not in {"idle", "finished"}
        )
        handshake_active = sum(
            1
            for robot in frame.robots
            if getattr(robot, "handshake_stage", "idle") not in {"idle", "carrying"}
        )
        consensus_pressure = int(round(100 * paxos_active / total_agents))
        handshake_pressure = int(round(100 * handshake_active / total_agents))

        return cls(
            gold_remaining=len(frame.gold),
            score_a=frame.scores.get("A", 0),
            score_b=frame.scores.get("B", 0),
            carrying_pairs_a=_count_pairs("A"),
            carrying_pairs_b=_count_pairs("B"),
            consensus_pressure=consensus_pressure,
            handshake_pressure=handshake_pressure,
        )


class ProbabilisticFiniteAutomaton:
    """Transition counter + probability estimator for CPS behaviour."""

    def __init__(self) -> None:
        self._counts: Dict[str, Dict[str, int]] = defaultdict(lambda: defaultdict(int))

    def add_transition(self, source: str, target: str) -> None:
        if not source or not target:
            return
        self._counts[source][target] += 1

    def as_matrix(self) -> Dict[str, Dict[str, float]]:
        matrix: Dict[str, Dict[str, float]] = {}
        for source, row in self._counts.items():
            total = sum(row.values())
            if not total:
                continue
            matrix[source] = {
                target: round(count / total, 4) for target, count in sorted(row.items())
            }
        return matrix

    def most_likely_sequences(self, depth: int = 4) -> List[Dict[str, Any]]:
        """Return high-probability paths by greedily following the best transition."""
        sequences: List[Dict[str, Any]] = []
        matrix = self.as_matrix()
        for start in matrix:
            path = [start]
            probability = 1.0
            current = start
            for _ in range(depth):
                row = matrix.get(current)
                if not row:
                    break
                target, prob = max(row.items(), key=lambda item: item[1])
                probability *= prob
                path.append(target)
                current = target
                if prob == 0:
                    break
            if len(path) > 1:
                sequences.append({"path": path, "probability": round(probability, 4)})
        sequences.sort(key=lambda item: item["probability"], reverse=True)
        return sequences[:5]


class CyberPhysicalGameAnalyzer:
    """Collect CPS metadata and construct the PFA."""

    def __init__(self, robots: List[Robot], cfg: Any, world_meta: Optional[Dict[str, Any]] = None):
        self.cfg = cfg
        self.robot_manifest = [
            {"id": robot.id, "team": robot.team, "number": robot.number} for robot in robots
        ]
        self.world_meta = world_meta or {}
        self.states: List[CyberPhysicalGameState] = []
        self.catalog: Dict[str, CyberPhysicalGameState] = {}
        self.pfa = ProbabilisticFiniteAutomaton()

    def observe(self, frame: Frame) -> None:
        state = CyberPhysicalGameState.from_frame(frame)
        label = state.label()
        if self.states:
            previous = self.states[-1].label()
            self.pfa.add_transition(previous, label)
        self.states.append(state)
        self.catalog.setdefault(label, state)

    def build_report(self) -> Dict[str, Any]:
        state_records = [
            {"id": label, **state.as_dict()} for label, state in sorted(self.catalog.items())
        ]
        transition_matrix = self.pfa.as_matrix()
        return {
            "cps": self._describe_cps(),
            "states": state_records,
            "transition_matrix": transition_matrix,
            "likely_sequences": self.pfa.most_likely_sequences(),
        }

    def _describe_cps(self) -> Dict[str, Any]:
        latency_min, latency_max = getattr(self.cfg, "message_delay_range", (1, 1))
        drop = getattr(self.cfg, "message_loss_chance", 0.0)
        reorder = getattr(self.cfg, "message_reorder_chance", 0.0)
        environment_properties = [
            {"name": "gold_remaining", "description": "Number of gold piles left on the grid."},
            {"name": "score_a", "description": "Score accumulated by Team A."},
            {"name": "score_b", "description": "Score accumulated by Team B."},
            {
                "name": "carrying_pairs",
                "description": "Number of active gold-carrying pairs per team.",
            },
            {
                "name": "coordination_pressure",
                "description": "Percent of robots engaged in consensus/handshake work.",
            },
        ]
        message_template = {
            "latency_range": {"min": latency_min, "max": latency_max},
            "loss_probability": drop,
            "reorder_probability": reorder,
        }
        channels = [
            {
                "agent": f"c{entry['id']}",
                "mi": dict(message_template),
                "mo": dict(message_template),
            }
            for entry in self.robot_manifest
        ]

        return {
            "environment": {
                "properties": environment_properties,
                **self.world_meta,
            },
            "cyber_agents": self.robot_manifest,
            "message_channels": channels,
        }

    def write_report(self, path: Path) -> None:
        report = self.build_report()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
