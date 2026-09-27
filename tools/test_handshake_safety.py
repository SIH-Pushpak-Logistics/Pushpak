"""Exercise handshake state logic without claiming a ROS/FCU hardware test."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch


def load_handshake():
    modules = {}
    for name, symbols in {
        'rclpy': [], 'rclpy.node': ['Node'],
        'rclpy.qos': ['DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy', 'qos_profile_sensor_data'],
        'geographic_msgs.msg': ['GeoPointStamped'],
        'geometry_msgs.msg': ['PoseStamped', 'TwistStamped'],
        'mavros_msgs.msg': ['HomePosition', 'State', 'StatusText'],
        'mavros_msgs.srv': ['CommandBool', 'CommandHome', 'CommandTOL', 'SetMode'],
        'sensor_msgs.msg': ['Range'], 'std_msgs.msg': ['Bool'],
    }.items():
        module = ModuleType(name)
        for symbol in symbols:
            setattr(module, symbol, type(symbol, (), {'Request': NS}))
        modules[name] = module
    path = Path(__file__).resolve().parents[1] / 'src/navigation_brain/navigation_brain/arm_takeoff_handshake.py'
    spec = importlib.util.spec_from_file_location('handshake_under_test', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module.ArmTakeoffHandshake


Handshake = load_handshake()


class HandshakeSafetyTest(unittest.TestCase):
    def test_old_or_invalid_ranges_cannot_establish_settled(self):
        node = object.__new__(Handshake)
        node.age = lambda timestamp: 10 - timestamp
        node.settle_window_s = 1.
        node.settle_band_m = .05
        node.tof_hist = [(8., 1.5)] * 10
        self.assertFalse(node.settled())
        node.tof_hist = [(9.5, 1.5)] * 10
        self.assertTrue(node.settled())
        node.tof_cb(NS(range=float('nan')))
        self.assertFalse(node.settled())
        self.assertIsNone(node.tof_latest)

    def test_land_retries_until_fcu_acknowledges(self):
        node = object.__new__(Handshake)
        node.phase = 'FLYING'
        node.stuck_warned = True
        node.last_cmd_time = 0
        node.age = lambda _: 2.
        node.fs4_timeout_s = 1.
        node.fcu = NS(armed=True, mode='GUIDED')
        node.get_logger = lambda: NS(error=lambda _: None)
        node.goto = lambda phase, _: setattr(node, 'phase', phase)
        sent = []
        node.request = lambda name, req: sent.append(req.custom_mode)
        node.due = lambda _: True
        node.tick()
        self.assertEqual(node.phase, 'LANDING')
        node.tick()  # Controller has not acknowledged: retry, do not abandon.
        self.assertEqual(sent, ['LAND', 'LAND'])
        node.fcu.mode = 'LAND'
        node.tick()
        self.assertEqual(node.phase, 'DONE')
        self.assertEqual(len(sent), 2)
