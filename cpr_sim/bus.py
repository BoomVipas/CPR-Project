from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from .models import Message, Robot


@dataclass
class Transmission:
    """Message scheduled for future delivery."""

    message: Message
    deliver_at: int
    latency: int


class MessageBus:
    """Injects network-level latency, reordering, and loss for team broadcasts."""

    def __init__(
        self,
        latency_range: Tuple[int, int],
        drop_chance: float,
        reorder_chance: float,
        rng,
    ) -> None:
        min_delay = max(1, int(latency_range[0]))
        max_delay = max(min_delay, int(latency_range[1]))
        self.latency_range = (min_delay, max_delay)
        self.drop_chance = min(1.0, max(0.0, drop_chance))
        self.reorder_chance = min(1.0, max(0.0, reorder_chance))
        self.rng = rng
        self.pending: List[Transmission] = []

    def queue(self, robot: Robot, payload: dict, current_step: int) -> Optional[int]:
        """Queue a message; returns latency in ticks or None if it was dropped."""
        delay = self.rng.randint(self.latency_range[0], self.latency_range[1])
        if self.drop_chance and self.rng.random() < self.drop_chance:
            return None

        message = Message(
            sender_id=robot.id,
            team=robot.team,
            content=payload,
            timestamp=current_step,
        )
        transmission = Transmission(
            message=message,
            deliver_at=current_step + delay,
            latency=delay,
        )
        self.pending.append(transmission)
        return delay

    def drain(self, current_step: int) -> List[Transmission]:
        """Return transmissions ready for delivery at this step."""
        ready: List[Transmission] = []
        waiting: List[Transmission] = []
        for transmission in self.pending:
            if transmission.deliver_at <= current_step:
                ready.append(transmission)
            else:
                waiting.append(transmission)
        self.pending = waiting

        if ready and self.reorder_chance and self.rng.random() < self.reorder_chance:
            self.rng.shuffle(ready)
        return ready
