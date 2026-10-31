"""Real Map01/GuiSR regressions; each scenario owns its OpenGL process.

Only the fixture injects damage. The submitted strategy, sensors, communication,
physics, competition guard and removal lifecycle run unchanged; no mocks.
Zero inventory isolates submission after task completion. Full ten-item rounds
are covered separately by scripts/benchmark_red.py.
"""
import json
import os
from pathlib import Path
import random
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))


def strategy_class():
    """可指定正式 ZIP，避免在实验分支中误测其他策略。"""
    archive = os.environ.get("SWARM_TEST_SUBMISSION_ZIP")
    if archive:
        from scripts.benchmark_red import load_submission
        factory, provenance = load_submission(archive)
        print("TEST IMPLEMENTATION " + json.dumps(provenance["implementation"]), flush=True)
        return factory("place")
    from swarm_rescue.solutions.my_drone_place_baseline import MyDronePlaceBaseline
    return MyDronePlaceBaseline


def run_scenario(scenario):
    import numpy as np
    from swarm_rescue.maps.map_01 import Map01
    from swarm_rescue.simulation.elements.sensor_disablers import NoComZone
    from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
    from swarm_rescue.simulation.reporting.team_mode import TeamMode
    from swarm_rescue.simulation.utils.constants import FRAME_RATE
    MyDronePlaceBaseline = strategy_class()

    random.seed(17)
    np.random.seed(17)
    the_map = Map01(drone_type=MyDronePlaceBaseline, number_drones=3)
    limit = 240
    the_map._max_timestep_limit = limit
    the_map.clear_bombs()
    for drone in the_map.drones:
        drone._misc_data.team_mode = TeamMode.PLACE
        drone._misc_data.max_timestep_limit = limit
        drone.enable_bomb_deployer(the_map, 0)
    if scenario == "isolated_survivor":
        # A real collision zone disables all radios, including at warm-up.
        the_map.playground.add(NoComZone(size=the_map.size_area),
                               ((0, 0), 0), allow_overlapping=True)
    removed_steps = {}
    try:
        gui = GuiSR(the_map=the_map, headless=True, competition_mode=True)
        while not gui._terminate:
            if gui.elapsed_timestep == 60 and scenario != "healthy":
                victims = the_map.drones[:2] if scenario == "isolated_survivor" else the_map.drones[:1]
                for drone in victims:
                    drone._drone_health = 1
                    drone._last_collision_damage_timestep = None
                    drone.collide_wall()
                    assert drone.removed and drone.drone_health == 0
                    removed_steps[drone.identifier] = drone._step
            if gui.elapsed_timestep == 143 and scenario == "stale_ready":
                # The survivor really received this announcement through radio;
                # the fixture does not write the strategy's message cache.
                assert the_map.drones[2]._peer_ready[1][1] is True
                drone = the_map.drones[1]
                drone._drone_health = 1
                drone._last_collision_damage_timestep = None
                drone.collide_wall()
                assert drone.removed
                removed_steps[drone.identifier] = drone._step
            gui.on_update(FRAME_RATE)
            assert gui.elapsed_timestep <= limit, "Round reached its limit without a submission"
        for identifier, last_step in removed_steps.items():
            assert the_map.drones[identifier]._step == last_step, "Removed drone was driven again"
        submitted = [d for d in the_map.drones if d._exploration_map_submission is not None]
        assert submitted, "No live drone submitted after the primary was removed"
        assert all(not d.removed for d in submitted)
        if scenario == "healthy":
            assert submitted[0].identifier == 0
        elif scenario == "isolated_survivor":
            assert [d.identifier for d in submitted] == [2]
            assert not the_map.drones[2]._peer, "Full-map NoComZone did not isolate the survivor"
        else:
            assert submitted[0].identifier != 0
        if scenario == "stale_ready":
            assert [d.identifier for d in submitted] == [2]
            heard, ready = submitted[0]._peer_ready[1]
            assert ready and submitted[0]._step - heard >= submitted[0].SUBMIT_PEER_TTL
        for drone in submitted:
            grid = drone._exploration_map_submission
            assert grid.shape == (800, 1250) and grid.dtype == np.uint8
            assert np.isin(grid, [0, 1]).all() and np.count_nonzero(grid) > 0
        print("SURVIVAL RESULT " + json.dumps({
            "scenario": scenario, "steps": gui.elapsed_timestep,
            "submitters": [int(d.identifier) for d in submitted],
            "removed": list(removed_steps), "competition_mode": True,
        }), flush=True)
    finally:
        the_map.playground.cleanup()


def run_partitioned_map_scenario():
    """Characterize the v04 risk, NOT a test asserting the risk is fixed.

    Actual radios exchange initial completion messages, then a real full-map
    NoComZone isolates them. Only the fixture changes inventory and physical
    observation poses; it never edits strategy maps, completed sets or inboxes.
    Controlled poses give the primary more wall observations than the backup.
    This is a synthetic real-engine fixture, not a platform replay.
    """
    import numpy as np
    from scripts.benchmark_red import RoundDiagnostics
    from swarm_rescue.maps.map_01 import Map01
    from swarm_rescue.simulation.elements.sensor_disablers import NoComZone
    from swarm_rescue.simulation.gui_map.gui_sr import GuiSR
    from swarm_rescue.simulation.reporting.team_mode import TeamMode
    from swarm_rescue.simulation.utils.constants import FRAME_RATE

    base = strategy_class()
    random.seed(17)
    np.random.seed(17)
    diagnostics = RoundDiagnostics(sample_period=40)
    the_map = Map01(drone_type=diagnostics.instrument(base), number_drones=2)
    limit = 400
    the_map._max_timestep_limit = limit
    the_map.clear_bombs()
    primary, backup = the_map.drones
    for drone in the_map.drones:
        drone._misc_data.team_mode = TeamMode.PLACE
        drone._misc_data.max_timestep_limit = limit
        drone.enable_bomb_deployer(the_map, int(drone.identifier == 1))
    before_partition = None
    try:
        gui = GuiSR(the_map=the_map, headless=True, competition_mode=True)
        while not gui._terminate:
            step = gui.elapsed_timestep
            if step == 40:
                assert 0 in backup._completed, "Initial completion was not received by real radio"
                assert 1 not in primary._completed, "Fixture completed before isolation"
                assert primary._peer and backup._peer
                the_map.playground.add(NoComZone(size=the_map.size_area),
                                       ((0, 0), 0), allow_overlapping=True)
            if step == 44:
                # Allow the zone to take effect and in-flight messages to drain.
                before_partition = {int(d.identifier): dict(d._peer) for d in the_map.drones}
                assert backup.carried_bombs_count() == 1
                backup.enable_bomb_deployer(the_map, 0)
            if step >= 40:
                # Fixture positions, not navigation commands or injected sensors.
                primary_pose = ((-180, 0), 0) if step < 100 else (
                    ((200, 280), 0) if step < 160 else ((500, -200), 0))
                # These wall-clear poses intentionally overlap the NoComZone.
                primary.move_to(primary_pose, allow_overlapping=True)
                backup.move_to(((-500, -200), 0), allow_overlapping=True)
            gui.on_update(FRAME_RATE)
            assert gui.elapsed_timestep <= limit, "No backup submission within deadline"

        observed = diagnostics.finish()
        assert before_partition is not None
        assert all(not d.removed for d in the_map.drones)
        assert observed["submitters"] == [1], observed["submitters"]
        assert 1 not in primary._completed and backup._completed == {0, 1}
        assert primary._peer == before_partition[0] and backup._peer == before_partition[1], (
            "Real NoComZone allowed further messages")
        assert len(backup._wall_set) < len(primary._wall_set), "Fixture did not create unequal map amounts"
        assert primary._step < int(0.82 * limit), "Backup did not precede the primary hard deadline"
        assert primary._exploration_map_submission is None
        grid = backup._exploration_map_submission
        assert grid.shape == (800, 1250) and grid.dtype == np.uint8
        assert np.isin(grid, [0, 1]).all()
        submission_samples = [s for s in observed["samples"] if s["submitted"]]
        assert submission_samples and submission_samples[0]["id"] == 1
        assert submission_samples[0]["missing_completed"] == []
        print("PARTITION RESULT " + json.dumps({
            "scenario": "partitioned_less_mapped_backup", "steps": gui.elapsed_timestep,
            "implementation": observed["implementation"], "submitters": observed["submitters"],
            "final_drones": observed["final_drones"],
            "union_wall_bins_at_end": observed["union_wall_bins_at_end"],
            "observer_seconds": observed["observer_seconds"],
        }), flush=True)
    finally:
        the_map.playground.cleanup()


class SubmissionSurvivalTests(unittest.TestCase):
    def run_real_round(self, scenario):
        result = subprocess.run(
            [sys.executable, "-B", str(Path(__file__).resolve()), "--scenario", scenario],
            cwd=ROOT, text=True, capture_output=True, timeout=90,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        print(result.stdout, end="")

    def test_healthy_primary(self):
        self.run_real_round("healthy")

    def test_destroyed_primary(self):
        self.run_real_round("destroyed_primary")

    def test_isolated_last_survivor(self):
        self.run_real_round("isolated_survivor")

    def test_ready_peer_dies_before_submitting(self):
        self.run_real_round("stale_ready")

    def test_partitioned_less_mapped_backup_can_submit_before_live_primary(self):
        self.run_real_round("partitioned_less_mapped_backup")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--scenario":
        if sys.argv[2] == "partitioned_less_mapped_backup":
            run_partitioned_map_scenario()
        else:
            run_scenario(sys.argv[2])
    else:
        unittest.main()
