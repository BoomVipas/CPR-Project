class LamportClock:
    """Lamport logical clock for ordering events in distributed system."""

    def __init__(self):
        self.t = 0

    def tick(self):
        """Increment local clock."""
        self.t += 1
        return self.t

    def send(self):
        """Get timestamp for outgoing message."""
        return self.tick()

    def recv(self, msg_ts: int):
        """Update clock on message receipt."""
        self.t = max(self.t, msg_ts) + 1
        return self.t