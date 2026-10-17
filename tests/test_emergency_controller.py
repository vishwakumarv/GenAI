"""Lightweight tests that do not require SUMO to be installed."""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from emergency_controller import (  # noqa: E402
    ALL_RED_STATE,
    GLOBAL_CYCLE_SECONDS,
    GREEN_SECONDS,
    YELLOW_SECONDS,
    green_state,
    normal_signal_at,
    yellow_state,
)


class SignalScheduleTests(unittest.TestCase):
    def test_cycle_is_exactly_45_seconds(self):
        self.assertEqual(4 * (GREEN_SECONDS + YELLOW_SECONDS), GLOBAL_CYCLE_SECONDS)

    def test_phase_boundaries(self):
        self.assertEqual(normal_signal_at(0)[0:2], (0, green_state(0)))
        self.assertEqual(normal_signal_at(8.25)[0:2], (0, yellow_state(0)))
        self.assertEqual(normal_signal_at(11.25)[0:2], (1, green_state(1)))
        self.assertEqual(normal_signal_at(45)[0:2], (0, green_state(0)))
        self.assertEqual(normal_signal_at(90)[0:2], (0, green_state(0)))

    def test_each_phase_is_20_signal_characters_and_conflict_free(self):
        for phase in range(4):
            green = green_state(phase)
            yellow = yellow_state(phase)
            self.assertEqual(len(green), 20)
            self.assertEqual(len(yellow), 20)
            self.assertEqual(green.count("G"), 5)
            self.assertEqual(yellow.count("y"), 5)
            self.assertEqual(green.count("r"), 15)
            self.assertEqual(yellow.count("r"), 15)
        self.assertEqual(len(ALL_RED_STATE), 20)
        self.assertEqual(ALL_RED_STATE.count("r"), 20)


if __name__ == "__main__":
    unittest.main()
