from dataclasses import dataclass
from typing import Optional, List


@dataclass
class Promise:
    """Promise response in Paxos protocol."""
    acceptor_id: int
    n: int
    accepted_n: int
    accepted_v: Optional[List[int]]


class Acceptor:
    """Paxos acceptor role."""

    def __init__(self, node_id: int):
        self.node_id = node_id
        self.promised_n = -1
        self.accepted_n = -1
        self.accepted_v = None

    def on_prepare(self, n: int) -> Optional[Promise]:
        """Handle prepare request."""
        if n > self.promised_n:
            self.promised_n = n
            return Promise(
                acceptor_id=self.node_id,
                n=n,
                accepted_n=self.accepted_n,
                accepted_v=self.accepted_v
            )
        return None


class Proposer:
    """Paxos proposer role."""

    def __init__(self, node_id: int, quorum: List[int]):
        self.node_id = node_id
        self.quorum = quorum
        self.n = 0

    def next_n(self) -> int:
        """Generate next proposal number."""
        self.n += 1
        # Ensure uniqueness across proposers by encoding node_id
        return self.n * 1000 + self.node_id

    def update_quorum(self, peers: List[int]):
        """Update quorum membership."""
        self.quorum = peers
