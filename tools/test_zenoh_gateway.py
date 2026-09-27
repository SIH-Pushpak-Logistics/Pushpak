import unittest
from zenoh_gateway import FleetState, GatewayState, MESSAGES, decode


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

    def test_fleet_keeps_vehicle_origins_and_loss_separate(self):
        now = [10.]
        fleet = FleetState([1, 2], lambda: now[0])
        for drone_id, x in ((1, 1000), (2, -2000)):
            for kind, name, fields in (
                ('heartbeat', 'Heartbeat', {}),
                ('keyframe', 'SubMapKeyframe', {'pos_x_mm': x}),
            ):
                payload = MESSAGES['swarm.telemetry.' + name](drone_id=drone_id, **fields).SerializeToString()
                fleet.receive(f'pushpak/{kind}/{drone_id}', payload)
        drones = fleet.snapshot()['data']
        self.assertEqual(drones['1']['pose']['x'], 1)
        self.assertEqual(drones['2']['pose']['x'], -2)
        now[0] += 2
        packet = MESSAGES['swarm.telemetry.Heartbeat'](drone_id=2).SerializeToString()
        fleet.receive('pushpak/heartbeat/2', packet)
        drones = fleet.snapshot()['data']
        self.assertEqual(drones['1']['linkStatus']['state'], 'OFFLINE')
        self.assertEqual(drones['2']['linkStatus']['state'], 'ONLINE')
