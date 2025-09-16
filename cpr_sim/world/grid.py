import random


class GridWorld:
    def __init__(self, seed=123, size=20, gold=25):
        self.size = size
        self.rng = random.Random(seed)

        # Initialize grid cells
        self.cells = [[{'gold': 0, 'deposit': None} for _ in range(size)]
                      for _ in range(size)]

        # Set deposit locations
        self.deposA = (0, 0)
        self.deposB = (size - 1, size - 1)
        self.cells[0][0]['deposit'] = 'A'
        self.cells[size - 1][size - 1]['deposit'] = 'B'

        # Place gold randomly
        placed = 0
        while placed < gold:
            x = self.rng.randint(0, size - 1)
            y = self.rng.randint(0, size - 1)

            if (x, y) not in (self.deposA, self.deposB):
                self.cells[y][x]['gold'] += 1
                placed += 1

    def inb(self, p):
        """Check if position p is within grid bounds."""
        return 0 <= p[0] < self.size and 0 <= p[1] < self.size
