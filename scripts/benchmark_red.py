"""Run one real official round per selected scenario and seed, saving scores.

This is a local evaluation tool only; it must not be put in a submission ZIP.
Each scenario resets both RNGs, so changing scenario order cannot change a run.
"""
import argparse
from dataclasses import asdict
import gc
import hashlib
import importlib
import inspect
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import time
import types
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_rescue.simulation.reporting.bombs_io import exploration_map_output_path


def class_source(drone_type):
    """记录实际工厂返回类的源码；不能用工作区同名文件冒充 ZIP 源码。"""
    module = inspect.getmodule(drone_type)
    source = module.__file__
    data = module.__loader__.get_data(source)
    return {"class": f"{drone_type.__module__}.{drone_type.__qualname__}",
            "source": source, "sha256": hashlib.sha256(data).hexdigest()}


def load_submission(archive):
    """Import the real solutions package directly from a local ZIP, without extraction.

    Engine/dependencies remain those of this checkout. No workspace solutions
    fallback is allowed, and image-based submissions cannot be run by this tool.
    Call before importing Launcher; its factory is bound at module import time.
    """
    archive = Path(archive).resolve()
    with zipfile.ZipFile(archive) as zf:
        names = zf.namelist()
        for required in ("my_drone_eval.py", "team_info.yml", "req.txt"):
            if names.count(required) != 1:
                raise ValueError(f"ZIP must contain exactly one root {required}")
        if "image_ref.txt" in names and zf.read("image_ref.txt").strip():
            raise ValueError("Image submission: ZIP code is not the evaluated implementation")
        initializer = zf.read("__init__.py") if "__init__.py" in names else b""
        source_hashes = {n: hashlib.sha256(zf.read(n)).hexdigest()
                         for n in names if n.endswith(".py")}
    import swarm_rescue
    prefix = "swarm_rescue.solutions"
    for name in list(sys.modules):
        if name == prefix or name.startswith(prefix + "."):
            del sys.modules[name]
    package = types.ModuleType(prefix)
    package.__package__ = prefix
    package.__path__ = [str(archive)]
    package.__file__ = str(archive / "__init__.py")
    sys.modules[prefix] = package
    swarm_rescue.solutions = package
    exec(compile(initializer, package.__file__, "exec"), package.__dict__)
    entry = importlib.import_module(prefix + ".my_drone_eval")
    factory = entry.drone_class_for_mode
    implementation = class_source(factory("place"))
    return factory, {"zip_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                     "zip": str(archive), "python_sources": source_hashes,
                     "implementation": implementation}


class RoundDiagnostics:
    """仅供评测进程观察；不向策略传递其他无人机的状态或任何地图真值。

    The wrapper delegates every control call to the actual implementation and
    returns its unchanged command. Maps/caches are never edited. Detailed state
    is sampled, or captured when completion/readiness/submission changes.
    Wall-bin counts and the offline union below are NOT map-quality scores.
    """
    def __init__(self, sample_period=100):
        self.sample_period = sample_period
        self.drones = []
        self.samples = []
        self._previous = {}
        self.observer_seconds = 0.0
        self.implementation = None
        self.runtime_class = None

    def instrument(self, base):
        self.implementation = class_source(base)
        observer = self

        class ObservedDrone(base):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                observer.drones.append(self)

            def control(self):
                command = super().control()
                start = time.perf_counter()
                observer.observe(self)
                observer.observer_seconds += time.perf_counter() - start
                return command

        self.runtime_class = f"{ObservedDrone.__module__}.{ObservedDrone.__qualname__}"
        return ObservedDrone

    @staticmethod
    def snapshot(drone):
        step = int(drone._step)
        count = drone._misc_data.number_drones
        completed = sorted(int(i) for i in drone._completed)
        peers = {str(i): {"last_received_step": int(data[0]),
                          "age_steps": step - int(data[0])}
                 for i, data in sorted(drone._peer.items())}
        last_received = max((p["last_received_step"] for p in peers.values()), default=None)
        return {
            "id": int(drone.identifier), "step": step,
            "engine_step": int(drone.elapsed_timestep),
            "wall_bins": len(drone._wall_set), "wall_bin_pixels": drone.WALL_BIN,
            "local_confirmed_bins": sum(v >= 2 for v in drone._wall_votes.values()),
            "free_cells": int(np.count_nonzero(drone._free)),
            "completed": completed, "missing_completed": sorted(set(range(count)) - set(completed)),
            "ready_since": drone._ready_since,
            "submitted": drone._exploration_map_submission is not None,
            "peers": peers,
            "last_contact_age_steps": None if last_received is None else step - last_received,
            "peer_ready": {str(i): {"age_steps": step - int(heard), "ready": bool(ready)}
                           for i, (heard, ready) in sorted(drone._peer_ready.items())},
            "inventory": int(drone.carried_bombs_count()),
            "health": int(drone.drone_health), "removed": bool(drone.removed),
        }

    def observe(self, drone):
        key = int(drone.identifier)
        state = (tuple(sorted(drone._completed)), drone._ready_since,
                 drone._exploration_map_submission is not None)
        changed = self._previous.get(key) != state
        if changed or drone._step % self.sample_period == 0:
            self.samples.append({"event": "state_change" if changed else "periodic",
                                 **self.snapshot(drone)})
        self._previous[key] = state

    def finish(self):
        # Offline only: the strategy never receives this cross-drone union.
        union = set().union(*(d._wall_set for d in self.drones))
        return {"implementation": self.implementation, "runtime_class": self.runtime_class,
                "sample_period": self.sample_period, "observer_seconds": self.observer_seconds,
                "submitters": [int(d.identifier) for d in self.drones
                               if d._exploration_map_submission is not None],
                "union_wall_bins_at_end": len(union),
                "final_drones": [{**self.snapshot(d),
                                  "missing_union_bins_at_end": len(union - d._wall_set)}
                                 for d in self.drones],
                "samples": self.samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--seeds", type=int, nargs="+", default=[17])
    parser.add_argument("--maps", nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--submission-zip", type=Path,
                        help="Load actual solutions from this ZIP; do not use workspace strategy")
    parser.add_argument("--diagnostics-every", type=int, default=100,
                        help="Observer sample period in steps (default: 100)")
    args = parser.parse_args()
    if args.diagnostics_every < 1:
        parser.error("--diagnostics-every must be positive")
    submission = None
    if args.submission_zip:
        factory, submission = load_submission(args.submission_zip)
    else:
        from swarm_rescue.solutions.my_drone_eval import drone_class_for_mode
        factory = drone_class_for_mode
    # Import after ZIP selection. Explicitly select the same factory even if a
    # caller previously imported Launcher in this diagnostic process.
    from swarm_rescue import launcher as launcher_module
    Launcher = launcher_module.Launcher
    implementation = class_source(factory("place"))
    output = args.output.resolve()
    summary = output / "benchmark.json"
    if summary.exists():
        parser.error(f"Refusing to overwrite {summary}")
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "python": platform.python_version(),
        "config": str(args.config.resolve()),
        "strategy_sha256": implementation["sha256"],
        "implementation": implementation, "submission": submission,
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
            diagnostics = RoundDiagnostics(args.diagnostics_every)
            observed_type = diagnostics.instrument(factory("place"))
            original_factory = launcher_module.drone_class_for_mode
            launcher_module.drone_class_for_mode = lambda mode: observed_type if mode == "place" else factory(mode)
            try:
                result = launcher.one_round(
                    scenario, 1, headless=True, competition_mode=True)
            finally:
                launcher_module.drone_class_for_mode = original_factory
                # Save traces even if the official runner raises an exception.
                diagnostic_path = exploration_path.with_suffix(".diagnostics.json")
                diagnostic_path.parent.mkdir(parents=True, exist_ok=True)
                diagnostic_path.write_text(json.dumps(diagnostics.finish(), ensure_ascii=False, indent=2),
                                           encoding="utf-8")
            if result is None or len(result) != 15 or result[-1] is None:
                raise RuntimeError("Official round returned no red score")
            row = {
                "map": scenario.map_name,
                "seed": seed,
                "zones": scenario.zones_name_casual,
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
                "diagnostics": str(diagnostic_path),
                "submitters": [int(d.identifier) for d in diagnostics.drones
                               if d._exploration_map_submission is not None],
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
