"""Regression tests for stale odometry and coordinate-frame boundaries."""
import unittest
from types import SimpleNamespace as NS
from simulate_integration import install_fake_ros

install_fake_ros()
from ros_adapter import RosZenohAdapter


class AdapterContractTest(unittest.TestCase):
    def setUp(self):
        self.adapter = object.__new__(RosZenohAdapter)
        self.adapter.pose_source = 'odometry'
        self.adapter.max_odom_age_s = 0.5
        self.adapter.status_flags = 0
        self.adapter.get_clock = lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000))
        self.adapter.get_logger = lambda: NS(warning=lambda _: None)
        self.sent = []
        self.adapter.send = self.sent.append

    def odom(self, sec=10, frame='odom'):
        return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=0)),
                  pose=NS(pose=NS(position=NS(x=1., y=2., z=3.),
                                  orientation=NS(x=0., y=0., z=0., w=1.))))

    def test_stale_future_and_wrong_frame_are_not_transmitted(self):
        for odom in (self.odom(9), self.odom(11), self.odom(frame='map')):
            self.adapter.on_odometry(odom)
            self.adapter.send_keyframe()
        self.assertEqual(self.sent, [])
        self.adapter.on_odometry(self.odom())
        self.adapter.send_keyframe()
        self.assertEqual(self.sent[0]['pos_x_mm'], 1000)
        self.assertEqual(self.sent[0]['timestamp_ms'], 10000)

    def test_detection_with_wrong_frame_never_becomes_survivor(self):
        self.adapter.on_detection(NS(header=NS(frame_id='base_link')))
        self.assertEqual(self.sent, [])
