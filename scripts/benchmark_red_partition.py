"""Real-engine radio partition experiment; never include in submission ZIPs.

The fixture adds an official NoComZone, without changing sensors, commands or
strategy state. Truth is used only for scoring after control has ended.
"""
import argparse
from dataclasses import asdict
import gc
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import platform
import random
import subprocess
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))

from swarm_rescue.simulation.elements.sensor_disablers import NoComZone
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.reporting.place_score_manager import PlaceScoreManager, walls_and_boundary_grid
from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.simulation.utils.constants import FRAME_RATE


def observed_grid(drone):
    """Render only stored observations, using the v04 cell renderer, off-policy."""
    w, h = map(int, drone.size_area)
    grid = np.zeros((h, w), dtype=np.uint8)
    half = drone.WALL_BIN // 2
    for col, row in drone._wall_set:
        x, y = col * drone.WALL_BIN, row * drone.WALL_BIN
        if 0 <= x < w and 0 <= y < h:
            grid[max(0, y-half):min(h, y-half+drone.WALL_BIN),
                 max(0, x-half):min(w, x-half+drone.WALL_BIN)] = 1
    cv2.rectangle(grid, (2, 2), (w-3, h-3), 1, 5)
    return grid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', choices=['Map01', 'Map03', 'Map04', 'Map07'], default='Map07')
    parser.add_argument('--partition', choices=['full', 'left', 'right'], default='full')
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--partition-step', type=int, default=0,
                        help='Simulation step at which the real radio-disabling zone is added')
    parser.add_argument('--strategy', type=Path, default=ROOT/'src/swarm_rescue/solutions/my_drone_place_baseline.py')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite output')
    if args.partition_step < 0:
        parser.error('partition-step must be nonnegative')
    engine = subprocess.check_output(['git', 'log', '-1', '--format=%H', '--',
                                      'src/swarm_rescue/simulation'], cwd=ROOT, text=True).strip()
    spec = importlib.util.spec_from_file_location('partition_strategy', args.strategy)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    map_class = getattr(importlib.import_module('swarm_rescue.maps.map_'+args.map[-2:]), args.map)
    random.seed(args.seed)
    np.random.seed(args.seed)
    gc.disable()
    the_map = map_class(drone_type=module.MyDronePlaceBaseline, number_drones=5)
    try:
        the_map.clear_bombs()
        for drone in the_map.drones:
            drone._misc_data.team_mode = TeamMode.PLACE
            drone.enable_bomb_deployer(the_map, 2)
        w, h = the_map.size_area
        zone_size = (w, h) if args.partition == 'full' else (w/2, h)
        zone_x = 0 if args.partition == 'full' else (-w/4 if args.partition == 'left' else w/4)
        def add_partition():
            the_map.playground.add(NoComZone(size=zone_size), ((zone_x, 0), 0), allow_overlapping=True)
        if args.partition_step == 0:
            add_partition()
        gui = GuiSR(the_map=the_map, headless=True, competition_mode=True)
        partition_added = args.partition_step == 0
        disabled_steps = {int(d.identifier): 0 for d in the_map.drones}
        while not gui._terminate:
            if not partition_added and gui.elapsed_timestep >= args.partition_step:
                add_partition()
                partition_added = True
            gui.on_update(FRAME_RATE)
            for drone in the_map.drones:
                if not drone.removed and drone.communicator_is_disabled():
                    disabled_steps[int(drone.identifier)] += 1
        submitters = [d for d in the_map.drones if d._exploration_map_submission is not None]
        assert partition_added, 'Round ended before partition injection'
        if args.partition == 'full' and args.partition_step == 0:
            assert all(not d._peer for d in the_map.drones), 'Full NoComZone leaked peer messages'
        prediction = submitters[0]._exploration_map_submission if submitters else None
        # No data below is supplied to strategy control.
        walls, boundary = walls_and_boundary_grid(the_map.playground,
            truth_grid=the_map.explored_map.truth_obstacle_grid())
        scorer = PlaceScoreManager()
        scores = scorer.compute_phase1_score(walls, prediction,
            elapsed_walltime=gui.elapsed_walltime, max_walltime_limit=the_map.max_walltime_limit,
            score_exploration_trajectory=the_map.explored_map.score()*100,
            placed_count=len(the_map.live_bombs), expected_count=10,
            bomb_positions=[tuple(b.true_position()) for b in the_map.live_bombs],
            elapsed_timestep=gui.elapsed_timestep, max_timestep_limit=the_map.max_timestep_limit,
            wall_grid=walls, boundary_grid=boundary)
        drones = []
        for drone in the_map.drones:
            grid = observed_grid(drone)
            map_result = scorer.compute_map_score(walls, grid,
                elapsed_walltime=gui.elapsed_walltime, max_walltime_limit=the_map.max_walltime_limit,
                has_crashed=False)
            drones.append({'id': int(drone.identifier), 'removed': drone.removed,
                'inventory': drone.carried_bombs_count(), 'wall_bins': len(drone._wall_set),
                'observed_free_fraction': float(np.mean(drone._free)),
                'peers_heard': len(drone._peer), 'known_completed': len(drone._completed),
                'radio_disabled_steps': disabled_steps[int(drone.identifier)],
                'last_peer_map_age': (drone._step - drone._last_peer_map_step
                                      if hasattr(drone, '_last_peer_map_step') else None),
                'raw_cell_map_score_at_end': map_result.score})
        report = {'python': platform.python_version(), 'engine_commit': engine,
            'strategy_sha256': hashlib.sha256(args.strategy.read_bytes()).hexdigest(),
            'map': args.map, 'partition': args.partition, 'seed': args.seed,
            'partition_step': args.partition_step,
            'competition_mode': True, 'steps': gui.elapsed_timestep,
            'wall_seconds': gui.elapsed_walltime, 'walltime_limit_reached': gui.is_max_walltime_limit_reached,
            'submitters': [int(d.identifier) for d in submitters], 'placed': len(the_map.live_bombs),
            'inventory': sum(d.carried_bombs_count() for d in the_map.drones),
            'destroyed_percent': gui.percent_drones_destroyed,
            'scores': asdict(scores), 'drones': drones}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print('PARTITION RESULT '+json.dumps(report), flush=True)
        return int(not submitters or gui.is_max_walltime_limit_reached)
    finally:
        the_map.playground.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
