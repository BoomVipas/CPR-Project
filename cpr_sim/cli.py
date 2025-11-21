import argparse
from typing import List, Optional

from .sim import Config, Simulation


DEFAULT_CFG = Config()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the CPR Paxos simulation.")
    parser.add_argument("--ticks", type=int, default=DEFAULT_CFG.ticks, help="Ticks to simulate.")
    parser.add_argument(
        "--max-ticks",
        type=int,
        default=DEFAULT_CFG.max_ticks,
        help="Absolute upper bound on ticks (still enforced when continuing until finished).",
    )
    parser.add_argument("--seed", type=int, default=DEFAULT_CFG.seed, help="Random seed.")
    parser.add_argument("--gold", type=int, default=DEFAULT_CFG.gold, help="Initial gold piles.")
    parser.add_argument("--print-every", type=int, default=DEFAULT_CFG.print_every, help="Console render cadence.")
    parser.add_argument(
        "--robots-per-team", type=int, default=DEFAULT_CFG.robots_per_team, help="Robots per team."
    )
    parser.add_argument("--grid-size", type=int, default=DEFAULT_CFG.grid_size, help="Grid width/height.")
    parser.add_argument(
        "--log-file",
        default=DEFAULT_CFG.log_file,
        help="Output log file (omit to disable logging).",
    )
    parser.add_argument(
        "--frames-file",
        default=DEFAULT_CFG.frames_file,
        help="Write animation frames to this JSON file (omit to disable).",
    )
    parser.add_argument(
        "--analysis-file",
        default=DEFAULT_CFG.analysis_file,
        help="Write CPS/PFA analysis to this JSON file (omit to disable).",
    )
    parser.add_argument(
        "--message-delay-min",
        type=int,
        default=DEFAULT_CFG.message_delay_range[0],
        help="Minimum team broadcast latency (ticks).",
    )
    parser.add_argument(
        "--message-delay-max",
        type=int,
        default=DEFAULT_CFG.message_delay_range[1],
        help="Maximum team broadcast latency (ticks).",
    )
    parser.add_argument(
        "--message-loss",
        type=float,
        default=DEFAULT_CFG.message_loss_chance,
        help="Probability [0,1] that a broadcast is dropped.",
    )
    parser.add_argument(
        "--message-reorder",
        type=float,
        default=DEFAULT_CFG.message_reorder_chance,
        help="Probability [0,1] that ready broadcasts are delivered out of order.",
    )
    parser.add_argument(
        "--consensus",
        choices=["paxos", "handshake"],
        default=DEFAULT_CFG.consensus,
        help="Consensus coordination method to use ('paxos' or 'handshake').",
    )
    parser.add_argument(
        "--run-until-finished",
        dest="run_until_finished",
        action="store_true",
        help="Continue stepping past --ticks until all gold has been deposited.",
    )
    parser.add_argument(
        "--no-run-until-finished",
        dest="run_until_finished",
        action="store_false",
        help="Stop strictly after --ticks, even if gold remains.",
    )
    parser.set_defaults(run_until_finished=DEFAULT_CFG.run_until_finished)
    return parser


def create_config(args: argparse.Namespace) -> Config:
    return Config(
        ticks=args.ticks,
        max_ticks=args.max_ticks,
        seed=args.seed,
        gold=args.gold,
        print_every=args.print_every,
        log_file=args.log_file,
        grid_size=args.grid_size,
        robots_per_team=args.robots_per_team,
        frames_file=args.frames_file,
        analysis_file=args.analysis_file,
        message_delay_range=(
            max(1, args.message_delay_min),
            max(max(1, args.message_delay_min), args.message_delay_max),
        ),
        message_loss_chance=max(0.0, min(1.0, args.message_loss)),
        message_reorder_chance=max(0.0, min(1.0, args.message_reorder)),
        consensus=args.consensus.lower(),
        run_until_finished=args.run_until_finished,
    )


def main(argv: Optional[List[str]] = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    sim = Simulation(create_config(args))
    sim.run()


if __name__ == "__main__":
    main()
