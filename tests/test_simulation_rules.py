import random

from cpr_sim.engine import SimulationEngine
from cpr_sim.models import GridWorld, Robot, Message
from cpr_sim import strategy_paxos
from cpr_sim.sim import Config


def _make_engine(robots, gold_positions, strategies=None, consensus: str = "paxos"):
    world = GridWorld(width=5, height=5)
    world.deposits = {"A": (0, 0), "B": (4, 4)}
    world.gold = set(gold_positions)
    cfg = Config(
        log_file=None,
        frames_file=None,
        message_delay_range=(1, 1),
        message_loss_chance=0.0,
        message_reorder_chance=0.0,
        consensus=consensus,
    )
    rng = random.Random(0)
    if strategies is None:
        if consensus.lower() == "handshake":
            from cpr_sim.strategy_handshake import handshake_strategy

            strategies = {robot.id: handshake_strategy for robot in robots}
        else:
            from cpr_sim.strategy_paxos import paxos_strategy

            strategies = {robot.id: paxos_strategy for robot in robots}
    return SimulationEngine(world, robots, strategies, cfg, rng)


def _run_single_tick(robots, gold_positions, strategies):
    engine = _make_engine(robots, gold_positions, strategies)
    frame = engine.tick()
    return engine, frame


def _log_contains(frame, text):
    return any(text in entry for entry in frame.logs)


def test_pickup_requires_partner_for_success():
    robots = [
        Robot(id=0, team="A", number=1, pos=(1, 1), facing="N"),
    ]
    strategies = {0: lambda r, w, e: "pick_up"}

    engine, frame = _run_single_tick(robots, {(1, 1)}, strategies)

    assert (1, 1) in engine.world.gold, "Gold should remain when only one robot attempts pickup."
    assert not robots[0].carrying
    assert _log_contains(frame, "failed pickup"), "Engine should log the failed pickup attempt."


def test_pair_pickup_succeeds_with_exactly_two_teammates():
    robots = [
        Robot(id=0, team="A", number=1, pos=(2, 2), facing="N"),
        Robot(id=1, team="A", number=2, pos=(2, 2), facing="N"),
    ]
    strategies = {
        0: lambda r, w, e: "pick_up",
        1: lambda r, w, e: "pick_up",
    }

    engine, frame = _run_single_tick(robots, {(2, 2)}, strategies)

    assert (2, 2) not in engine.world.gold
    assert robots[0].carrying and robots[0].carrying_with == 1
    assert robots[1].carrying and robots[1].carrying_with == 0
    assert _log_contains(frame, "picked up gold")


def test_cross_team_pickup_requires_two_gold_bars():
    robots = [
        Robot(id=0, team="A", number=1, pos=(3, 3), facing="N"),
        Robot(id=1, team="A", number=2, pos=(3, 3), facing="N"),
        Robot(id=2, team="B", number=1, pos=(3, 3), facing="N"),
        Robot(id=3, team="B", number=2, pos=(3, 3), facing="N"),
    ]
    strategies = {rid: (lambda r, w, e: "pick_up") for rid in range(4)}

    engine, frame = _run_single_tick(robots, {(3, 3)}, strategies)

    assert (3, 3) in engine.world.gold, "Single gold bar cannot be split between teams."
    assert all(not robot.carrying for robot in robots)
    assert _log_contains(frame, "Both teams failed pickup"), "Teams should be notified of the contention failure."


def test_carrying_pair_drops_gold_when_desynchronised():
    robots = [
        Robot(id=0, team="A", number=1, pos=(2, 2), facing="N", carrying=True, carrying_with=1),
        Robot(id=1, team="A", number=2, pos=(2, 2), facing="N", carrying=True, carrying_with=0),
    ]
    strategies = {
        0: lambda r, w, e: "move_forward",
        1: lambda r, w, e: "turn_left",
    }

    engine, frame = _run_single_tick(robots, set(), strategies)

    assert (2, 1) in engine.world.gold, "Dropped gold should reappear on the grid."
    assert not robots[0].carrying and not robots[1].carrying
    assert _log_contains(frame, "dropped gold"), "Engine should log the desynchronisation drop."


def test_deposit_succeeds_only_when_both_robots_commit():
    robots = [
        Robot(id=0, team="A", number=1, pos=(0, 0), facing="N", carrying=True, carrying_with=1),
        Robot(id=1, team="A", number=2, pos=(0, 0), facing="N", carrying=True, carrying_with=0),
    ]
    strategies = {
        0: lambda r, w, e: "deposit",
        1: lambda r, w, e: "deposit",
    }

    engine, frame = _run_single_tick(robots, set(), strategies)

    assert engine.scores["A"] == 1
    assert not robots[0].carrying and not robots[1].carrying
    assert _log_contains(frame, "deposited gold")


def test_deposit_fails_when_partner_not_coordinated():
    robots = [
        Robot(id=0, team="A", number=1, pos=(0, 0), facing="N", carrying=True, carrying_with=1),
        Robot(id=1, team="A", number=2, pos=(0, 0), facing="N", carrying=True, carrying_with=0),
    ]
    strategies = {
        0: lambda r, w, e: "deposit",
        1: lambda r, w, e: "idle",
    }

    engine, frame = _run_single_tick(robots, set(), strategies)

    assert engine.scores["A"] == 0
    assert robots[0].carrying and robots[1].carrying, "Pair should keep carrying when deposit fails."
    assert _log_contains(frame, "failed deposit"), "Engine should log that the deposit was not coordinated."


def test_reserved_gold_is_not_reselected_by_new_leader():
    world = GridWorld(width=6, height=6)
    world.deposits = {"A": (0, 0)}
    world.gold = {(2, 2), (4, 4)}

    movers = Robot(
        id=0,
        team="A",
        number=1,
        pos=(1, 1),
        facing="E",
        assigned_gold=(2, 2),
        execution_state="moving_to_gold",
    )
    leader = Robot(
        id=1,
        team="A",
        number=2,
        pos=(0, 1),
        facing="E",
    )

    class DummyEngine:
        def __init__(self, robots):
            self.robots = robots

    engine = DummyEngine([movers, leader])

    candidates = strategy_paxos._select_candidate_gold(leader, world, [], engine)

    assert (2, 2) not in candidates
    assert (4, 4) in candidates


def test_handshake_successfully_pairs_and_deposits():
    finder = Robot(id=0, team="A", number=1, pos=(1, 1), facing="N")
    helper = Robot(id=1, team="A", number=2, pos=(2, 1), facing="W")
    engine = _make_engine([finder, helper], {(1, 1)}, consensus="handshake")
    engine.world.deposits["A"] = (1, 1)  # deposit on the discovery tile

    carried = False
    for _ in range(40):
        engine.tick()
        if finder.carrying and helper.carrying:
            carried = True
        if engine.scores["A"] >= 1:
            break

    assert carried, "Handshake should coordinate a successful pickup."
    assert engine.scores["A"] == 1, "Robots should deposit when standing on the home depot."

    if finder.handshake_stage != "idle" or helper.handshake_stage != "idle":
        engine.tick()

    assert finder.handshake_stage == "idle"
    assert helper.handshake_stage == "idle"
    assert not finder.carrying
    assert not helper.carrying


def test_handshake_helper_aborts_when_ack_targets_other_robot():
    world = GridWorld(width=4, height=4)
    world.deposits = {"A": (0, 0)}
    world.gold = {(1, 1)}

    helper = Robot(
        id=5,
        team="A",
        number=1,
        pos=(0, 0),
        facing="N",
        handshake_stage="await_ack",
        handshake_role="helper",
        handshake_active_index=3,
        handshake_partner=2,
        handshake_target=(1, 1),
    )
    helper.messages.append(
        Message(
            sender_id=2,
            team="A",
            content={"type": "finder_ack", "finder": 2, "helper": 99, "index": 3},
            timestamp=0,
        )
    )

    class DummyEngine:
        def __init__(self, robots, step=5):
            self.robots = robots
            self.step = step

    from cpr_sim.strategy_handshake import handshake_strategy

    handshake_strategy(helper, world, DummyEngine([helper]))

    assert helper.handshake_stage == "idle"
    assert helper.handshake_partner is None
