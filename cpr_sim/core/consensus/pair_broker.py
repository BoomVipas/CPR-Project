from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Set, Tuple


@dataclass(frozen=True)
class PairDecision:
    """Represents a broker match announcement."""

    team: str
    pair: Tuple[int, int]
    cell: Tuple[int, int]
    decided_at: int
    status: str = "awaiting_ready"


@dataclass
class PairState:
    """Tracks lifecycle of a specific carrier pair for readiness and release handshakes."""

    team: str
    pair: Tuple[int, int]
    cell: Tuple[int, int]
    decided_at: int
    expires_at: int
    ready: Set[int] = field(default_factory=set)
    confirmed: bool = False
    active_deadline: Optional[int] = None
    releases: Set[int] = field(default_factory=set)
    cleared_at: Optional[int] = None

    def register_release(self, robot_id: int, now: int) -> bool:
        """Register a release from one robot; return True when pair fully cleared."""
        if self.cleared_at is not None:
            return True
        self.releases.add(robot_id)
        if len(self.releases) == 2:
            self.cleared_at = now
            return True
        return False


class PairBroker:
    """CSP-style broker that matches carrier pairs on a single gold cell."""

    def __init__(
        self,
        cell: Tuple[int, int],
        ttl: int = 5,
        handshake_ttl: int = 3,
        active_ttl: int = 25,
        deposit_positions: Optional[Dict[str, Tuple[int, int]]] = None,
        active_margin: int = 6,
    ):
        self.cell = cell
        self.ttl = ttl
        self.handshake_ttl = handshake_ttl
        self.active_ttl_min = active_ttl
        self.active_margin = active_margin
        self.deposit_positions = deposit_positions or {}
        self.offers: Dict[str, Dict[int, int]] = {"A": {}, "B": {}}
        self.active_pairs: Dict[str, List[PairState]] = {"A": [], "B": []}
        self.events: List[dict] = []

    def offer(self, team: str, robot_id: int, now: int) -> List[PairDecision]:
        """Register an offer and return any new pair decisions triggered by it."""
        self._expire(now, team)
        if any(robot_id in state.pair and state.cleared_at is None for state in self.active_pairs[team]):
            self.events.append({
                "kind": "offer_ignored",
                "team": team,
                "robot": robot_id,
                "cell": self.cell,
                "tick": now,
                "reason": "already_in_pair",
            })
            return []

        if any(state.cleared_at is None for state in self.active_pairs[team]):
            # Pair in progress; defer this offer until the active pair clears.
            self.offers[team][robot_id] = now
            self.events.append({
                "kind": "offer_deferred",
                "team": team,
                "robot": robot_id,
                "cell": self.cell,
                "tick": now,
            })
            return []

        self.offers[team][robot_id] = now
        decisions: List[PairDecision] = []

        fresh = self._fresh_offers(team, now)
        if len(fresh) >= 2:
            first, second = fresh[0], fresh[1]
            pair_tuple = tuple(sorted((first, second)))
            state = PairState(
                team=team,
                pair=pair_tuple,
                cell=self.cell,
                decided_at=now,
                expires_at=now + self.handshake_ttl,
            )
            self.active_pairs[team].append(state)
            decisions.append(PairDecision(team=team, pair=pair_tuple, cell=self.cell, decided_at=now))

            for rid in pair_tuple:
                self.offers[team].pop(rid, None)

            self.events.append({
                "kind": "pair_created",
                "team": team,
                "pair": pair_tuple,
                "cell": self.cell,
                "tick": now,
                "status": "awaiting_ready",
            })
        return decisions

    def mark_ready(self, team: str, robot_id: int, now: int) -> Optional[PairState]:
        """Mark that a robot is ready on the cell; returns the state when pair confirmed."""
        for state in self.active_pairs[team]:
            if robot_id in state.pair and state.cleared_at is None:
                if state.confirmed:
                    return state
                state.ready.add(robot_id)
                state.expires_at = now + self.handshake_ttl
                self.events.append({
                    "kind": "pair_ready",
                    "team": team,
                    "robot": robot_id,
                    "pair": state.pair,
                    "cell": self.cell,
                    "tick": now,
                })
                if len(state.ready) == 2:
                    state.confirmed = True
                    state.active_deadline = now + self._compute_active_ttl(team)
                    self.events.append({
                        "kind": "pair_confirmed",
                        "team": team,
                        "pair": state.pair,
                        "cell": self.cell,
                        "tick": now,
                    })
                    return state
                return None
        self.events.append({
            "kind": "ready_ignored",
            "team": team,
            "robot": robot_id,
            "cell": self.cell,
            "tick": now,
        })
        return None

    def release(self, team: str, robot_id: int, mate_id: int, now: int) -> Optional[PairState]:
        """Register release for a pair; return the cleared state when both released."""
        pair_key = tuple(sorted((robot_id, mate_id)))
        for state in self.active_pairs[team]:
            if state.pair == pair_key and state.cleared_at is None:
                cleared = state.register_release(robot_id, now)
                if cleared:
                    self.active_pairs[team].remove(state)
                    self.events.append({
                        "kind": "pair_released",
                        "team": team,
                        "pair": state.pair,
                        "cell": self.cell,
                        "tick": now,
                    })
                    return state
                return None
        return None

    def expire_all(self, now: int) -> List[dict]:
        """Expire stale offers and timed-out pair states; returns expiration events."""
        expired: List[dict] = []
        for team in ("A", "B"):
            self._expire(now, team)
            survivors: List[PairState] = []
            for state in self.active_pairs[team]:
                if state.cleared_at is not None:
                    continue
                if not state.confirmed and now > state.expires_at:
                    expired.append(self._record_expiry(team, state, now, "handshake_timeout"))
                    continue
                if state.confirmed and state.active_deadline and now > state.active_deadline:
                    expired.append(self._record_expiry(team, state, now, "active_timeout"))
                    continue
                survivors.append(state)
            self.active_pairs[team] = survivors
        return expired

    def _record_expiry(self, team: str, state: PairState, now: int, reason: str) -> dict:
        event = {
            "team": team,
            "pair": state.pair,
            "cell": self.cell,
            "tick": now,
            "reason": reason,
        }
        self.events.append({
            "kind": "pair_expired",
            "team": team,
            "pair": state.pair,
            "cell": self.cell,
            "tick": now,
            "reason": reason,
        })
        return event

    def _fresh_offers(self, team: str, now: int) -> List[int]:
        return sorted(rid for rid, ts in self.offers[team].items() if now - ts <= self.ttl)

    def _expire(self, now: int, team: str) -> None:
        stale = [rid for rid, ts in self.offers[team].items() if now - ts > self.ttl]
        for rid in stale:
            self.offers[team].pop(rid, None)
            self.events.append({
                "kind": "offer_expired",
                "team": team,
                "robot": rid,
                "cell": self.cell,
                "tick": now,
            })

    def has_active_pair(self, team: str, robot_id: int) -> bool:
        return any(robot_id in state.pair and state.cleared_at is None for state in self.active_pairs[team])

    def drain_events(self) -> List[dict]:
        events = self.events.copy()
        self.events.clear()
        return events

    def _compute_active_ttl(self, team: str) -> int:
        ttl = self.active_ttl_min
        depot = self.deposit_positions.get(team)
        if depot:
            dist = abs(self.cell[0] - depot[0]) + abs(self.cell[1] - depot[1])
            # Give pairs ample time: worst-case path plus extra room for rotations/coordination.
            ttl = max(self.active_ttl_min, dist * 2 + self.active_margin)
        return ttl
