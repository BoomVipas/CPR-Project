from dataclasses import dataclass, field
import heapq

@dataclass(order=True)
class Task:
    """
    Represents a scheduled task with a deadline and release time.
    """
    deadline: int
    release: int
    name: str = field(compare=False)
    meta: dict = field(default_factory=dict, compare=False)

class EDF:
    """
    Earliest Deadline First (EDF) scheduler for managing tasks.
    """
    def __init__(self):
        self.h = []  # Min-heap for tasks

    def add(self, t):
        """
        Add a task to the scheduler.
        """
        heapq.heappush(self.h, t)

    def pop(self, now):
        """
        Pop the next available task whose release time is <= now.

        Args:
            now: Current simulation time.

        Returns:
            The chosen Task or None if no task is ready.
        """
        tmp = []
        chosen = None
        while self.h:
            t = heapq.heappop(self.h)
            if t.release <= now:
                chosen = t
                break
            tmp.append(t)
        for t in tmp:
            heapq.heappush(self.h, t)
        return chosen
