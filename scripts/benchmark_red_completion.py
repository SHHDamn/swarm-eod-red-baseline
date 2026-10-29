"""Real-engine completion diagnostics; telemetry is never returned to control.

The official plan's declared zones and actual zone entities are recorded
separately. Optional fixture zones are explicit, not changes to official maps.
"""
import argparse
from dataclasses import asdict
import gc
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import random
import subprocess
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from swarm_rescue.launcher import Launcher
from swarm_rescue.simulation.elements.sensor_disablers import NoComZone, NoGpsZone
from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
from swarm_rescue.simulation.reporting.place_score_manager import walls_and_boundary_grid
from swarm_rescue.simulation.reporting.team_mode import TeamMode
from swarm_rescue.simulation.utils.constants import FRAME_RATE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/local_place_test_plan.yml')
    parser.add_argument('--map',default='Map03')
    parser.add_argument('--seed',type=int,default=71)
    parser.add_argument('--strategy',type=Path,default=ROOT/'src/swarm_rescue/solutions/my_drone_place_baseline.py')
    parser.add_argument('--inject-left-zones',action='store_true')
    parser.add_argument('--inject-zone-step',type=int,default=0)
    parser.add_argument('--destroy-id',type=int)
    parser.add_argument('--destroy-step',type=int,default=1000)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite output')
    if args.inject_zone_step < 0:
        parser.error('inject-zone-step must be nonnegative')
    if args.destroy_step < 0:
        parser.error('destroy-step must be nonnegative')
    os.environ['SWARM_RESCUE_RESULT_PATH'] = str(args.output.parent/'diagnostic-files')
    launcher = Launcher(config_path=str(args.config))
    scenario = next(ec for ec in launcher.eval_plan.list_eval_config if ec.map_name == args.map)
    engine = subprocess.check_output(['git','log','-1','--format=%H','--','src/swarm_rescue/simulation'],cwd=ROOT,text=True).strip()
    spec = importlib.util.spec_from_file_location('completion_strategy',args.strategy)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    cls = getattr(importlib.import_module('swarm_rescue.maps.map_'+args.map[-2:]),args.map)
    random.seed(args.seed)
    np.random.seed(args.seed)
    gc.disable()
    the_map = cls(drone_type=module.MyDronePlaceBaseline,zones_config=scenario.zones_config,
                  number_drones=scenario.number_drones)
    try:
        if args.destroy_id is not None and not 0 <= args.destroy_id < len(the_map.drones):
            parser.error('destroy-id is outside this round roster')
        launcher._configure_map_for_team(the_map,scenario,TeamMode.PLACE)
        declared_entities = {'NoComZone':sum(isinstance(e,NoComZone) for e in the_map.playground.elements),
                             'NoGpsZone':sum(isinstance(e,NoGpsZone) for e in the_map.playground.elements)}
        def add_fixture_zones():
            w,h = the_map.size_area
            for zone_type in (NoComZone,NoGpsZone):
                the_map.playground.add(zone_type(size=(w/2,h)),((-w/4,0),0),allow_overlapping=True)
        injected = args.inject_left_zones and args.inject_zone_step == 0
        if injected:
            add_fixture_zones()
        gui = GuiSR(the_map=the_map,draw_interactive=False,headless=True,competition_mode=True)
        the_map.explored_map.reset()
        counters = {int(d.identifier):{'radio_disabled':0,'gps_disabled':0,'first_empty':None}
                    for d in the_map.drones}
        history,previous = [],{}
        removed_step = None
        while not gui._terminate:
            if args.destroy_id is not None and removed_step is None and gui.elapsed_timestep >= args.destroy_step:
                victim = the_map.drones[args.destroy_id]
                assert not victim.removed, 'Victim died before requested injection'
                victim._drone_health = 1
                victim._last_collision_damage_timestep = None
                victim.collide_wall()
                assert victim.removed
                removed_step = victim._step
            if args.inject_left_zones and not injected and gui.elapsed_timestep >= args.inject_zone_step:
                add_fixture_zones()
                injected = True
            gui.on_update(FRAME_RATE)
            for drone in the_map.drones:
                if drone.removed:
                    continue
                identifier = int(drone.identifier)
                row = counters[identifier]
                row['radio_disabled'] += int(drone.communicator_is_disabled())
                row['gps_disabled'] += int(drone.gps_is_disabled())
                inventory = drone.carried_bombs_count()
                if inventory == 0 and row['first_empty'] is None:
                    row['first_empty'] = drone._step
                key = (inventory,tuple(sorted(drone._completed)))
                if previous.get(identifier) != key or drone._step%100 == 0:
                    history.append({'step':drone._step,'id':identifier,'inventory':inventory,
                        'completed':list(key[1]),'pos':drone._pos,'start':drone._start,
                        'radio_disabled':drone.communicator_is_disabled(),
                        'gps_disabled':drone.gps_is_disabled(),
                        'peers':{i:{'age':drone._step-t,'inv':v,'pos':p}
                                 for i,(t,v,p,_) in drone._peer.items()}})
                    previous[identifier] = key
        # Only the fixture sees truth, after all strategy calls have ended.
        assert not args.inject_left_zones or injected, 'Round ended before fixture-zone injection'
        assert args.destroy_id is None or removed_step is not None, 'Round ended before loss injection'
        actual_entities = {'NoComZone':sum(isinstance(e,NoComZone) for e in the_map.playground.elements),
                           'NoGpsZone':sum(isinstance(e,NoGpsZone) for e in the_map.playground.elements)}
        submitters = [d for d in the_map.drones if d._exploration_map_submission is not None]
        prediction = submitters[0]._exploration_map_submission if submitters else None
        walls,boundary = walls_and_boundary_grid(the_map.playground,truth_grid=the_map.explored_map.truth_obstacle_grid())
        bomb_positions = [tuple(b.true_position()) for b in the_map.live_bombs]
        scores = launcher.place_score_manager.compute_phase1_score(walls,prediction,
            elapsed_walltime=gui.elapsed_walltime,max_walltime_limit=the_map.max_walltime_limit,
            score_exploration_trajectory=the_map.explored_map.score()*100,
            placed_count=len(the_map.live_bombs),expected_count=scenario.initial_bombs_per_drone*the_map.number_drones,
            bomb_positions=bomb_positions,
            elapsed_timestep=gui.elapsed_timestep,max_timestep_limit=the_map.max_timestep_limit,
            wall_grid=walls,boundary_grid=boundary)
        report = {'python':platform.python_version(),'engine_commit':engine,
            'strategy_sha256':hashlib.sha256(args.strategy.read_bytes()).hexdigest(),
            'map':args.map,'seed':args.seed,'competition_mode':True,
            'zones_declared':[z.name for z in scenario.zones_config],
            'actual_zone_entities_before_fixture':declared_entities,
            'actual_zone_entities_after_fixture':actual_entities,
            'injected_left_zones':args.inject_left_zones,
            'fixture_zone_step':args.inject_zone_step if args.inject_left_zones else None,
            'destroyed_id':args.destroy_id,'destroy_elapsed_step':args.destroy_step if args.destroy_id is not None else None,
            'removed_strategy_step':removed_step,
            'steps':gui.elapsed_timestep,
            'walltime_limit_reached':gui.is_max_walltime_limit_reached,
            'submitters':[int(d.identifier) for d in submitters],
            'placed':len(the_map.live_bombs),'inventory':sum(d.carried_bombs_count() for d in the_map.drones),
            'destroyed_percent':gui.percent_drones_destroyed,'scores':asdict(scores),
            'bomb_positions':bomb_positions,
            'counters':counters,'history':history}
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('COMPLETION SUMMARY '+json.dumps({k:v for k,v in report.items() if k!='history'}),flush=True)
        return int(not submitters or gui.is_max_walltime_limit_reached)
    finally:
        the_map.playground.cleanup()


if __name__ == '__main__':
    raise SystemExit(main())
