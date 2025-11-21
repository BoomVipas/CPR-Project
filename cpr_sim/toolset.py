from __future__ import annotations

from typing import Dict, Any

from .models import Message, Robot, GridWorld, DIRECTIONS


def move_forward(robot: Robot, world: GridWorld) -> bool:
    """Advance the robot one tile if the destination is in bounds."""
    dx, dy = 0, 0
    if robot.facing == "N":
        dy = -1
    elif robot.facing == "S":
        dy = 1
    elif robot.facing == "E":
        dx = 1
    else:  # "W"
        dx = -1

    new_pos = (robot.pos[0] + dx, robot.pos[1] + dy)
    if 0 <= new_pos[0] < world.width and 0 <= new_pos[1] < world.height:
        robot.pos = new_pos
        return True
    return False


def turn_left(robot: Robot) -> None:
    """Rotate robot 90° counter-clockwise."""
    idx = (DIRECTIONS.index(robot.facing) - 1) % len(DIRECTIONS)
    robot.facing = DIRECTIONS[idx]


def turn_right(robot: Robot) -> None:
    """Rotate robot 90° clockwise."""
    idx = (DIRECTIONS.index(robot.facing) + 1) % len(DIRECTIONS)
    robot.facing = DIRECTIONS[idx]


def pick_up(robot: Robot, world: GridWorld) -> bool:
    """
    Attempt to pick up gold.
    The engine validates the 2-robot rule; we only ensure there's gold and the robot is free.
    """
    return not robot.carrying and robot.pos in world.gold


def deposit(robot: Robot, world: GridWorld) -> bool:
    """
    Attempt to deposit gold.
    Both robots in a carrying pair must deposit together; the engine finalises it.
    """
    if not robot.carrying:
        return False
    depot = world.deposits.get(robot.team)
    return depot is not None and robot.pos == depot


def broadcast_message(robot: Robot, world: GridWorld, content: Dict[str, Any], step: int) -> None:
    """Queue a team broadcast for delivery on the next tick."""
    world.broadcast_queue.append(
        Message(
            sender_id=robot.id,
            team=robot.team,
            content=content,
            timestamp=step,
        )
    )
