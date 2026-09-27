"""Exercise bounded callback writes and adapter exit when its Rust child dies."""

import queue
import subprocess
import sys
import time
import unittest
from types import SimpleNamespace

from simulate_integration import FakeNode, RIG_BINARY, install_fake_ros


def child_failure_case():
    install_fake_ros()
    import rclpy

    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    FakeNode.overrides = {'pose_source': 'odometry', 'rust_binary': str(RIG_BINARY)}

    def spin(adapter):
        for _ in range(20):
            adapter.send_heartbeat()
        adapter.peer.kill()
        adapter.peer.wait(timeout=5)
        deadline = time.monotonic() + 5
        while not adapter.writer_failed.is_set() and time.monotonic() < deadline:
            time.sleep(0.01)
        adapter.check_writer()

    rclpy.spin = spin
    from ros_adapter import main
    return main()


class AdapterFailureTest(unittest.TestCase):
    def test_pose_source_is_required(self):
        install_fake_ros()
        from ros_adapter import RosZenohAdapter
        FakeNode.overrides = {}
        with self.assertRaisesRegex(ValueError, 'pose_source is required'):
            RosZenohAdapter()

    def test_bounded_queue_does_not_block_callback(self):
        install_fake_ros()
        from ros_adapter import RosZenohAdapter
        adapter = object.__new__(RosZenohAdapter)
        adapter.outbox = queue.Queue(maxsize=2)
        adapter.writer_failed = SimpleNamespace(is_set=lambda: False)
        adapter.dropped_messages = 0
        warnings = []
        adapter.get_logger = lambda: SimpleNamespace(warning=warnings.append)
        start = time.monotonic()
        for _ in range(1000):
            adapter.send({'kind': 'heartbeat'})
        elapsed = time.monotonic() - start
        self.assertLess(elapsed, 1.0)
        self.assertEqual(adapter.dropped_messages, 998)
        self.assertIn('dropped 1 messages', warnings[0])
        print(f'PASS: 1000 callback sends completed in {elapsed:.3f}s; '
              f'{adapter.dropped_messages} dropped when queue was full')

    def test_child_killed_mid_stream_exits_nonzero_once(self):
        self.assertTrue(RIG_BINARY.is_file(), f'build Rust binary first: {RIG_BINARY}')
        result = subprocess.run([sys.executable, __file__, '--child-failure-case'],
                                capture_output=True, text=True, timeout=15)
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertEqual(result.stdout.count('Rust Zenoh peer stopped:'), 1,
                         result.stdout + result.stderr)
        self.assertNotIn('BrokenPipeError', result.stderr)
        print(result.stdout, end='')
        print('PASS: killing Rust mid-stream made adapter log once and exit 1')


if __name__ == '__main__':
    if '--child-failure-case' in sys.argv:
        sys.exit(child_failure_case())
    unittest.main(verbosity=2)
