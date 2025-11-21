from __future__ import annotations

import random
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
import json
from datetime import datetime

from .cyber_physical_game import CyberPhysicalGameAnalyzer
from .engine import SimulationEngine
from .models import Frame, GridWorld, Robot
from .strategy_paxos import paxos_strategy


@dataclass
class Config:
    ticks: int = 200
    max_ticks: int = 1000
    seed: int = 123
    gold: int = 10
    print_every: int = 10
    log_file: Optional[str] = "log.txt"
    grid_size: int = 20
    robots_per_team: int = 10
    frames_file: Optional[str] = "frames.json"
    message_delay_range: Tuple[int, int] = (1, 1)
    message_loss_chance: float = 0.0
    message_reorder_chance: float = 0.0
    consensus: str = "paxos"
    analysis_file: Optional[str] = "game_analysis.json"
    run_until_finished: bool = True


class Simulation:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.rng = random.Random(cfg.seed)
        self.world = GridWorld(width=cfg.grid_size, height=cfg.grid_size)
        self.world.deposits = {"A": (0, 0), "B": (cfg.grid_size - 1, cfg.grid_size - 1)}
        self._seed_gold(cfg.gold)
        self.robots = self._spawn_robots(cfg.robots_per_team)
        if cfg.consensus.lower() == "handshake":
            from .strategy_handshake import handshake_strategy

            strategies = {robot.id: handshake_strategy for robot in self.robots}
        else:
            strategies = {robot.id: paxos_strategy for robot in self.robots}
        self.engine = SimulationEngine(self.world, self.robots, strategies, cfg, self.rng)
        self.tick = 0
        self.log_handle = self._open_log()
        self._log_header()

    # ------------------------------------------------------------------#
    # Public API                                                         #
    # ------------------------------------------------------------------#

    def run(self) -> None:
        frames_for_export: List[Dict[str, Any]] = []
        analyzer: Optional[CyberPhysicalGameAnalyzer] = None
        extended_notice_emitted = False
        if self.cfg.analysis_file:
            world_meta = {
                "grid": {"width": self.world.width, "height": self.world.height},
                "deposits": {team: list(pos) for team, pos in self.world.deposits.items()},
            }
            analyzer = CyberPhysicalGameAnalyzer(list(self.robots), self.cfg, world_meta)
        if self.cfg.run_until_finished or self.cfg.ticks > 0:
            while True:
                frame = self.step()
                frames_for_export.append(frame.to_dict())
                if analyzer:
                    analyzer.observe(frame)
                if (self.tick - 1) % self.cfg.print_every == 0:
                    self._print_frame(frame)
                finished = self._game_finished()
                tick_cap_reached = self.cfg.ticks > 0 and self.tick >= self.cfg.ticks
                hard_cap_reached = self.cfg.max_ticks > 0 and self.tick >= self.cfg.max_ticks
                if hard_cap_reached and not finished:
                    self._emit_max_tick_notice()
                    break
                if finished:
                    break
                if tick_cap_reached:
                    if not self.cfg.run_until_finished:
                        break
                    if not extended_notice_emitted:
                        self._emit_extension_notice()
                        extended_notice_emitted = True
        self._log_footer()
        if self.cfg.frames_file:
            self._write_frames(frames_for_export)
        if analyzer:
            analyzer.write_report(Path(self.cfg.analysis_file))
        if self.log_handle:
            self.log_handle.close()

    def step(self) -> Frame:
        frame = self.engine.tick()
        self.tick += 1
        self._log_frame(frame)
        return frame

    # ------------------------------------------------------------------#
    # Initialisation helpers                                             #
    # ------------------------------------------------------------------#

    def _seed_gold(self, count: int) -> None:
        attempts = 0
        while len(self.world.gold) < count and attempts < count * 10:
            attempts += 1
            pos = (
                self.rng.randint(0, self.world.width - 1),
                self.rng.randint(0, self.world.height - 1),
            )
            if pos in self.world.deposits.values():
                continue
            self.world.gold.add(pos)

    def _spawn_robots(self, per_team: int) -> List[Robot]:
        robots: List[Robot] = []
        for idx in range(per_team):
            pos = (self.rng.randint(0, 4), self.rng.randint(0, 4))
            robots.append(
                Robot(
                    id=idx,
                    team="A",
                    number=idx + 1,
                    pos=pos,
                    facing=self.rng.choice(["N", "E", "S", "W"]),
                )
            )

        base_id = per_team
        for idx in range(per_team):
            pos = (
                self.world.width - 1 - self.rng.randint(0, 4),
                self.world.height - 1 - self.rng.randint(0, 4),
            )
            robots.append(
                Robot(
                    id=base_id + idx,
                    team="B",
                    number=idx + 1,
                    pos=pos,
                    facing=self.rng.choice(["N", "E", "S", "W"]),
                )
            )
        return robots

    # ------------------------------------------------------------------#
    # Logging + rendering                                                #
    # ------------------------------------------------------------------#

    def _open_log(self):
        if not self.cfg.log_file:
            return None
        path = Path(self.cfg.log_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        return path.open("w", encoding="utf-8")

    def _log_header(self) -> None:
        if not self.log_handle:
            return
        self.log_handle.write("=== CPR SIMULATION LOG ===\n")
        self.log_handle.write(
            f"Seed: {self.cfg.seed}, Gold: {len(self.world.gold)}, Ticks: {self.cfg.ticks}\n"
        )
        self.log_handle.write(f"Grid Size: {self.world.width}x{self.world.height}\n")
        self.log_handle.write(
            f"Deposits: A={self.world.deposits['A']}, B={self.world.deposits['B']}\n\n"
        )
        self.log_handle.flush()

    def _log_frame(self, frame: Frame) -> None:
        if not self.log_handle:
            return
        self.log_handle.write(f"--- STEP {frame.step:03d} ---\n")
        self.log_handle.write(
            f"Score A={frame.scores['A']}  B={frame.scores['B']}  Gold remaining={len(frame.gold)}\n"
        )
        if getattr(frame, "pickups", None):
            self.log_handle.write(
                f"Pickups A={frame.pickups.get('A', 0)}  B={frame.pickups.get('B', 0)}\n"
            )
        for robot in frame.robots:
            status = "carrying" if robot.carrying else "idle"
            self.log_handle.write(
                f"  R{robot.id:02d} Team {robot.team} @ {robot.pos} facing {robot.facing} "
                f"[{status}] control={robot.control_state} paxos={robot.paxos_state}/{robot.execution_state}\n"
            )
        if frame.logs:
            self.log_handle.write("  Engine logs:\n")
            for entry in frame.logs:
                self.log_handle.write(f"    - {entry}\n")
        self.log_handle.write("\n")
        self.log_handle.flush()

    def _log_footer(self) -> None:
        if not self.log_handle:
            return
        self.log_handle.write(
            f"=== SIMULATION COMPLETE === Score A={self.engine.scores['A']}  B={self.engine.scores['B']}\n"
        )

    def _print_frame(self, frame: Frame) -> None:
        print(f"=== STEP {frame.step:03d} ===")
        print(f"Score: Team A = {frame.scores['A']}, Team B = {frame.scores['B']}")
        pickups = getattr(frame, "pickups", {}) or {}
        if pickups:
            print(f"Pickups: Team A = {pickups.get('A', 0)}, Team B = {pickups.get('B', 0)}")
        print(f"Gold remaining: {len(frame.gold)}\n")
        print(self._render_ascii(frame))
        print()

    def _render_ascii(self, frame: Frame) -> str:
        grid = [[" ." for _ in range(frame.width)] for _ in range(frame.height)]
        for team, pos in frame.deposits.items():
            marker = "D1" if team == "A" else "D2"
            grid[pos[1]][pos[0]] = marker
        for gold in frame.gold:
            grid[gold[1]][gold[0]] = " G"
        for robot in frame.robots:
            marker = "A" if robot.team == "A" else "B"
            grid[robot.pos[1]][robot.pos[0]] = f" {marker}"
        lines = ["".join(row) for row in grid]
        return "\n".join(lines)

    def _game_finished(self) -> bool:
        if self.world.gold:
            return False
        return not any(robot.carrying for robot in self.robots)

    def _emit_extension_notice(self) -> None:
        message = "Tick budget exhausted; continuing until all gold is secured."
        print(message)
        if self.log_handle:
            self.log_handle.write(message + "\n\n")
            self.log_handle.flush()

    def _emit_max_tick_notice(self) -> None:
        message = f"Hard cap of {self.cfg.max_ticks} ticks reached before all gold was collected."
        print(message)
        if self.log_handle:
            self.log_handle.write(message + "\n\n")
            self.log_handle.flush()

    def _write_frames(self, frames: List[Dict[str, Any]]) -> None:
        path = Path(self.cfg.frames_file)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "generated_at": datetime.utcnow().isoformat(timespec="seconds") + "Z",
            "config": asdict(self.cfg),
            "frames": frames,
        }
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
