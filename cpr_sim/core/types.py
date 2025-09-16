DIR_ORDER = ['N', 'E', 'S', 'W']

DIRS = {
    'N': (0, -1),
    'E': (1, 0),
    'S': (0, 1),
    'W': (-1, 0)
}

def addv(a, b):
    """Add two vectors (tuples) together."""
    return (a[0] + b[0], a[1] + b[1])
