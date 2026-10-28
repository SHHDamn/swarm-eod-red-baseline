"""Unit regressions on a real initialized drone and its actual planner.

Navigation state is explicit test input; no sensor, planner or method is mocked.
Full sensor/physics integration is exercised by benchmark_red.py and the
existing test_red_submission_survival suite.
"""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_rescue.maps.map_01 import Map01
from swarm_rescue.solutions.my_drone_place_baseline import MyDronePlaceBaseline


class FrontierProgressTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arena = Map01(drone_type=MyDronePlaceBaseline, number_drones=1)
        cls.arena.playground.window.set_visible(False)

    @classmethod
    def tearDownClass(cls):
        cls.arena.playground.cleanup()

    def setUp(self):
        self.drone = self.arena.drones[0]
        d = self.drone
        d._step = d._escape_until = d._goal_timeouts = 0
        d._progress_goal = None
        d._goal_best_distance = float("inf")
        d._goal_progress_step = 0
        d._blocked_until.clear()
        d._peer.clear()
        d._free[:] = False
        d._hits[:] = d._visits[:] = 0
        d._goal = (20, 20)
        d._waypoints = [d._goal]

    def test_moving_in_circles_does_not_count_as_progress(self):
        d = self.drone
        x, y = d._world(d._goal)
        for step in range(4, 200, 4):
            d._step = step
            angle = step / 20.0
            d._pos = (x + 100 * math.cos(angle), y + 100 * math.sin(angle))
            d._check_goal_progress()
            if d._goal_timeouts:
                break
        self.assertEqual(d._goal_timeouts, 1)
        self.assertIsNone(d._goal)
        self.assertFalse(d._waypoints)

    def test_small_target_jitter_cannot_restart_timeout(self):
        d = self.drone
        d._pos = d._world((20, 5))
        for step in range(4, 200, 4):
            d._step = step
            d._goal = (20, 20 + (step // 4) % 2)
            d._waypoints = [d._goal]
            d._check_goal_progress()
            if d._goal_timeouts:
                break
        self.assertEqual(d._goal_timeouts, 1)

    def test_detour_away_from_goal_with_shorter_path_is_progress(self):
        d = self.drone
        d._goal = (20, 20)
        goal = d._world(d._goal)
        initial_direct = None
        for step in range(4, 244, 4):
            d._step = step
            # First leg of a U-shaped detour: away from the final target,
            # towards the opening at row 45. Remaining path is decreasing.
            row = 21 + (step - 4) // 12
            d._pos = d._world((row, 19))
            d._waypoints = [(45, 19), (45, 20), d._goal]
            direct = math.dist(d._pos, goal)
            if initial_direct is None:
                initial_direct = direct
            d._check_goal_progress()
        self.assertGreater(direct, initial_direct + 100)
        self.assertEqual(d._goal_timeouts, 0)

    def test_new_distant_goal_gets_its_own_progress_window(self):
        d = self.drone
        d._pos = d._world((10, 10))
        d._step = 4
        d._check_goal_progress()
        d._step = 140
        d._goal = (30, 30)
        d._waypoints = [d._goal]
        d._check_goal_progress()
        d._step = 200
        d._check_goal_progress()
        self.assertEqual(d._goal_timeouts, 0)

    def test_real_planner_avoids_then_retries_expired_region(self):
        d = self.drone
        d._free[5:26, 5:26] = True
        d._pos = d._world((15, 15))
        d._step = 4
        d._plan()
        first_goal = d._goal
        self.assertIsNotNone(first_goal)
        d._check_goal_progress()
        d._step += d.GOAL_PROGRESS_TIMEOUT
        d._check_goal_progress()
        self.assertEqual(d._goal_timeouts, 1)
        d._plan()
        self.assertIsNotNone(d._goal)
        self.assertGreater(max(abs(a - b) for a, b in zip(d._goal, first_goal)), 2)
        d._step = max(d._blocked_until.values()) + 1
        d._plan()
        self.assertEqual(d._goal, first_goal)


if __name__ == "__main__":
    unittest.main()
