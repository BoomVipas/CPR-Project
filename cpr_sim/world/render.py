def render_ascii(gw, robots, scoreA, scoreB, t):
    """Render the grid world as ASCII art with robots, deposits, and gold."""
    s = gw.size

    # Initialize empty layer
    layer = [['.' for _ in range(s)] for _ in range(s)]

    # Place deposits
    ax, ay = gw.deposA
    bx, by = gw.deposB
    layer[ay][ax] = 'D1'
    layer[by][bx] = 'D2'

    # Place gold
    for y in range(s):
        for x in range(s):
            g = gw.cells[y][x]['gold']
            if g > 0:
                layer[y][x] = 'G' if g == 1 else str(min(g, 9))

    # Place robots
    for r in robots:
        ch = 'A' if r.group == 'A' else 'B'
        if getattr(r, 'carry', None):
            ch = ch.lower()
        x, y = r.pos
        layer[y][x] = ch

    # Build output lines
    lines = [f'Tick {t:03d} | Score A={scoreA}  B={scoreB}']
    for row in layer:
        lines.append(' '.join(f'{c:>2}' for c in row))

    return '\n'.join(lines)
