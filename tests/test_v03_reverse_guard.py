"""Real lidar geometry and device disable lifecycle for v03 reverse guarding."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_rescue.maps.map_01 import Map01
from swarm_rescue.solutions.my_drone_place_baseline import MyDronePlaceBaseline


class ReverseGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arena = Map01(drone_type=MyDronePlaceBaseline, number_drones=1)
        cls.arena.playground.window.set_visible(False)
        cls.drone = cls.arena.drones[0]

    @classmethod
    def tearDownClass(cls):
        cls.arena.playground.cleanup()

    def read_at(self, x):
        # Facing +x: left arena boundary is directly behind the drone.
        self.drone.move_to(((x, 0), 0), allow_overlapping=True)
        for _ in range(3):
            self.arena.playground.step()
        return self.drone.lidar_values(), self.drone.lidar_rays_angles()

    def apply_guard(self, lidar, angles):
        command = {"forward": -0.4, "lateral": 0.1, "rotation": 0.7}
        self.drone._avoid(command, lidar, angles)
        self.assertEqual(command["rotation"], 0.7)
        return command

    def test_close_rear_wall_stops_translation(self):
        lidar, angles = self.read_at(-600)
        command = self.apply_guard(lidar, angles)
        self.assertEqual(command["forward"], 0)
        self.assertEqual(command["lateral"], 0)

    def test_medium_rear_clearance_limits_reverse(self):
        lidar, angles = self.read_at(-580)
        self.assertEqual(self.apply_guard(lidar, angles)["forward"], -0.25)

    def test_clear_rear_keeps_reverse(self):
        lidar, angles = self.read_at(-530)
        self.assertEqual(self.apply_guard(lidar, angles)["forward"], -0.4)

    def test_disabled_real_lidar_stops_translation(self):
        self.read_at(-530)
        self.drone.lidar().disable()
        lidar = self.drone.lidar_values()
        self.assertIsNone(lidar)
        command = self.apply_guard(lidar, self.drone.lidar_rays_angles())
        self.assertEqual(command["forward"], 0)
        self.assertEqual(command["lateral"], 0)


if __name__ == "__main__":
    unittest.main()
