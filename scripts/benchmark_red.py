"""Run one real official round per selected scenario and seed, saving scores.

This is a local evaluation tool only; it must not be put in a submission ZIP.
Each scenario resets both RNGs, so changing scenario order cannot change a run.
"""
import argparse
from dataclasses import asdict
import gc
import hashlib
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_rescue.launcher import Launcher
from swarm_rescue.simulation.reporting.bombs_io import exploration_map_output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[17])
    parser.add_argument("--maps", nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    summary = output / "benchmark.json"
    if summary.exists():
        parser.error(f"Refusing to overwrite {summary}")
    output.mkdir(parents=True, exist_ok=True)
    source = ROOT / "src/swarm_rescue/solutions/my_drone_place_baseline.py"
    report = {
        "python": platform.python_version(),
        "config": str(args.config.resolve()),
        "strategy_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "engine_commit": subprocess.check_output(
            ["git", "log", "-1", "--format=%H", "--", "src/swarm_rescue/simulation"],
            cwd=ROOT, text=True).strip(),
        "competition_mode": True,
        "rounds": [],
    }
    failed = False
    gc.disable()  # Same setting as the official launcher CLI.
    for seed in args.seeds:
        os.environ["SWARM_RESCUE_RESULT_PATH"] = str(output / f"seed{seed}")
        launcher = Launcher(config_path=str(args.config))
        scenarios = [ec for ec in launcher.eval_plan.list_eval_config
                     if args.maps is None or ec.map_name in args.maps]
        if not scenarios:
            parser.error("No selected scenario exists in the plan")
        for scenario in scenarios:
            if launcher._resolve_team_mode(scenario).value != "place":
                parser.error("This benchmark accepts red/place scenarios only")
            random.seed(seed)
            np.random.seed(seed)
            gc.collect()
            print(f"BENCHMARK START map={scenario.map_name} seed={seed}", flush=True)
            exploration_path = Path(exploration_map_output_path(
                launcher.result_path, launcher.team_info, scenario, 1))
            if exploration_path.exists():
                raise FileExistsError(f"Refusing to reuse {exploration_path}")
            result = launcher.one_round(
                scenario, 1, headless=True, competition_mode=True)
            if result is None or len(result) != 15 or result[-1] is None:
                raise RuntimeError("Official round returned no red score")
            row = {
                "map": scenario.map_name,
                "seed": seed,
                "zones": scenario.zones_name_casual,
                "zones_declared": [zone.name for zone in scenario.zones_config],
                "zone_verification": "plan_metadata_only",
                "destroyed_percent": result[0],
                "health": result[1],
                "steps": result[2],
                "coverage_percent": result[4],
                "wall_seconds": result[7],
                "walltime_limit_reached": result[8],
                "crashed": result[9],
                "placed": result[11],
                "inventory": result[13],
                "map_submitted": exploration_path.is_file(),
                "scores": asdict(result[14]),
            }
            failed |= bool(row["crashed"] or row["inventory"] or
                           row["walltime_limit_reached"] or not row["map_submitted"])
            report["rounds"].append(row)
            summary.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                               encoding="utf-8")
            print("BENCHMARK RESULT " + json.dumps(row, ensure_ascii=False), flush=True)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
