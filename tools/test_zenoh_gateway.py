import unittest
from zenoh_gateway import GatewayState, MESSAGES, decode


class GatewayTest(unittest.TestCase):
    def test_identity_and_budget(self):
        packet = MESSAGES['swarm.telemetry.Heartbeat'](drone_id=2).SerializeToString()
        self.assertIsNotNone(decode('pushpak/heartbeat/2', packet))
        for key in ('pushpak/heartbeat/1', 'pushpak/heartbeat/2/extra', 'other/heartbeat/2'):
            self.assertIsNone(decode(key, packet))
        self.assertIsNone(decode('pushpak/heartbeat/2', b'\xff'))
        self.assertIsNone(decode('pushpak/heartbeat/2', bytes(51)))

    def test_units_timeout_and_survivor_updates(self):
        now = [10.]
        state = GatewayState(2, lambda: now[0])
        def send(kind, name, **fields):
            state.receive(f'pushpak/{kind}/2', MESSAGES['swarm.telemetry.' + name](drone_id=2, **fields).SerializeToString())
        send('heartbeat', 'Heartbeat')
        send('keyframe', 'SubMapKeyframe', pos_x_mm=-1500, yaw_cdeg=9000)
        send('survivor', 'SurvivorEvent', survivor_id=7, confidence_pct=87, hit_count=1)
        send('survivor', 'SurvivorEvent', survivor_id=7, confidence_pct=90, hit_count=2)
        data = state.snapshot()['data']
        self.assertEqual(data['pose']['x'], -1.5)
        self.assertEqual(data['pose']['yaw_deg'], 90)
        self.assertEqual(len(data['victims']), 1)
        self.assertEqual(data['victims'][0]['confidence'], .9)
        self.assertEqual(data['linkStatus']['state'], 'ONLINE')
        self.assertIsNone(data['linkStatus']['rssi_dbm'])
        now[0] += 1
        self.assertIsNone(state.snapshot()['data']['pose'])
        now[0] += 1
        self.assertEqual(state.snapshot()['data']['linkStatus']['state'], 'OFFLINE')
