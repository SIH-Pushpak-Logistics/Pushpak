import math
import unittest

from conversion import SurvivorTracker, confidence_percent, millimetres, quaternion_rpy_cdeg


class ConversionTest(unittest.TestCase):
    def test_units_and_orientation(self):
        self.assertEqual(millimetres(1.25), 1250)
        q = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
        self.assertEqual(quaternion_rpy_cdeg(q), (0, 0, 9000))
        self.assertEqual(confidence_percent(0.855), 86)

    def test_stable_survivor_ids(self):
        tracker = SurvivorTracker()
        self.assertEqual(tracker.observe((0, 0, 0)), (1, 1))
        self.assertEqual(tracker.observe((1, 0, 0)), (1, 2))
        self.assertEqual(tracker.observe((3, 0, 0)), (2, 1))

    def test_invalid_values(self):
        for value in (math.nan, math.inf):
            with self.assertRaises(ValueError):
                millimetres(value)
        with self.assertRaises(ValueError):
            confidence_percent(1.1)
        with self.assertRaises(ValueError):
            quaternion_rpy_cdeg((0, 0, 0, 0))


if __name__ == '__main__':
    unittest.main()
