import argparse
from typing import Optional

from .sim import Simulation, Config


DEFAULT_CFG = Config()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the CPR grid simulation with optional terminal animation.",
    )
    parser.add_argument("--ticks", type=int, default=DEFAULT_CFG.ticks,
                        help="Number of simulation ticks to run (default: %(default)s)")
    parser.add_argument("--seed", type=int, default=DEFAULT_CFG.seed,
                        help="Seed for the random number generator (default: %(default)s)")
    parser.add_argument("--gold", type=int, default=DEFAULT_CFG.gold,
                        help="Total number of gold piles to spawn (default: %(default)s)")
    parser.add_argument("--print-every", type=int, default=DEFAULT_CFG.print_every,
                        help="Render every Nth tick to stdout (default: %(default)s)")
    parser.add_argument("--sleep-sec", type=float, default=None,
                        help="Delay between rendered frames in seconds")
    parser.add_argument("--num-per-team", type=int, default=DEFAULT_CFG.num_per_team,
                        help="Robots per team (default: %(default)s)")
    parser.add_argument("--log-file", default=DEFAULT_CFG.log_file,
                        help="Detailed log output path (default: %(default)s)")
    parser.add_argument("--no-detailed-log", action="store_true",
                        help="Disable verbose logging to disk")
    parser.add_argument("--animate", action="store_true",
                        help="Enable clear-screen redraws to create a terminal animation")
    return parser


def create_config(args: argparse.Namespace) -> Config:
    cfg_kwargs = {
        "ticks": args.ticks,
        "seed": args.seed,
        "gold": args.gold,
        "print_every": args.print_every,
        "log_file": args.log_file,
        "detailed_log": not args.no_detailed_log,
        "num_per_team": args.num_per_team,
        "animate": args.animate,
    }
    if args.sleep_sec is not None:
        cfg_kwargs["sleep_sec"] = max(0.0, args.sleep_sec)
    return Config(**cfg_kwargs)


def main(argv: Optional[list[str]] = None) -> None:
    parser = build_parser()
    args = parser.parse_args(argv)
    cfg = create_config(args)
    sim = Simulation(cfg)
    sim.run()


if __name__ == "__main__":
    main()
