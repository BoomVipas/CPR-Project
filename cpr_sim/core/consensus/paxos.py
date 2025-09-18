from typing import List, Optional, Tuple


class Acceptor:
    """Paxos acceptor role."""

    def __init__(self, node_id: int):
        self.node_id = node_id
        self.promised_n = -1
        self.accepted_n = -1
        self.accepted_v: Optional[List[int]] = None

    def on_prepare(self, n: int) -> Tuple[bool, int, Optional[List[int]]]:
        """Handle prepare request."""
        if n > self.promised_n:
            self.promised_n = n
            return True, self.accepted_n, self.accepted_v
        return False, self.accepted_n, self.accepted_v

    def on_accept(self, n: int, v: Optional[List[int]]) -> bool:
        """Handle accept request."""
        if n >= self.promised_n:
            self.promised_n = n
            self.accepted_n = n
            self.accepted_v = v
            return True
        return False


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
