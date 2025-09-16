from dataclasses import dataclass
import heapq

@dataclass
class Msg:
    """
    Represents a message sent between agents.
    """
    src: int
    dst: int
    ts: int
    kind: str
    payload: dict

class Bus:
    """
    Simulates a message bus with optional delay and drop probability.
    """
    def __init__(self, rng, delay_max=1, drop_prob=0.0):
        """
        Initialize the bus.

        Args:
            rng: Random number generator.
            delay_max: Maximum delivery delay (int).
            drop_prob: Probability of dropping a message (float).
        """
        self.rng = rng
        self.q = []            # Priority queue for scheduled messages
        self.seq = 0           # Sequence number for tie-breaking
        self.delay_max = delay_max
        self.drop_prob = drop_prob

    def send(self, now, m):
        """
        Send a message, possibly delayed or dropped.

        Args:
            now: Current simulation time.
            m: Msg object to send.
        """
        if self.rng.random() < self.drop_prob:
            return  # Message dropped
        self.seq += 1
        dt = self.rng.randint(0, self.delay_max)
        heapq.heappush(self.q, (now + dt, self.seq, m))

    def deliver(self, now):
        """
        Deliver all messages scheduled for now or earlier.

        Args:
            now: Current simulation time.

        Returns:
            List of delivered Msg objects.
        """
        out = []
        while self.q and self.q[0][0] <= now:
            _, _, m = heapq.heappop(self.q)
            out.append(m)
        return out
