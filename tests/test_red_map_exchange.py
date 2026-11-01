"""Map exchange on real Map01 radios; explicit map fixtures, no sensor mocks."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from swarm_rescue.maps.map_01 import Map01
from swarm_rescue.solutions.my_drone_place_baseline import MyDronePlaceBaseline


class MapExchangeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arena = Map01(drone_type=MyDronePlaceBaseline, number_drones=2)
        cls.arena.playground.window.set_visible(False)

    @classmethod
    def tearDownClass(cls):
        cls.arena.playground.cleanup()

    def setUp(self):
        self.sender, self.receiver = self.arena.drones
        for d in self.arena.drones:
            d._wall_list.clear()
            d._wall_set.clear()
            d._send_cursor = d._priority_cursor = 0
            d._history_end = 0
        self.history = {(i % 180, i // 180) for i in range(1000)}
        for point in sorted(self.history):
            self.sender._add_wall(point)
        for _ in range(30):
            self.sender.define_message_for_all()
        self.fresh = {(10 + i, 100) for i in range(36)}
        for point in sorted(self.fresh):
            self.sender._add_wall(point)

    def transmit(self, near):
        self.sender.move_to(((-400, 0), 0), allow_overlapping=True)
        self.receiver.move_to((((-350, 0) if near else (450, 200)), 0), allow_overlapping=True)
        message = self.sender.define_message_for_all()
        self.assertLessEqual(len(message["walls"]), 72)
        self.assertEqual(len(message["walls"]), len(set(message["walls"])))
        self.arena.playground.step(all_messages={
            self.sender: {self.sender.communicator: (None, message)}})
        self.receiver._read_messages()
        return message

    def test_short_contact_receives_fresh_information(self):
        self.transmit(True)
        self.assertTrue(self.fresh <= self.receiver._wall_set)

    def test_disconnect_reconnect_recovers_history_and_duplicates(self):
        self.transmit(False)
        self.assertFalse(self.receiver._wall_set)
        for _ in range(32):
            self.transmit(True)
        expected = self.history | self.fresh
        self.assertEqual(self.receiver._wall_set, expected)
        self.assertEqual(len(self.receiver._wall_list), len(expected))

    def test_continuous_new_information_does_not_starve_history(self):
        for tick in range(30):
            for i in range(36):
                self.sender._add_wall((i, 20 + tick))
            self.transmit(True)
        self.assertTrue(self.history <= self.receiver._wall_set)


if __name__ == "__main__":
    unittest.main()
