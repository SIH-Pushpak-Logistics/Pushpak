import { useEffect, useState } from 'react';
import type { DashboardState, DronePose } from '../types';

type VehicleSnapshot = Omit<DashboardState, 'track' | 'connected'>;
type Fleet = Record<string, DashboardState>;

const emptyVehicle = (droneId: string, connected: boolean): DashboardState => ({
    pose: null, altitude: null, velocity: null, flowDebug: null, victims: [], track: [],
    linkStatus: {
        timestamp: 0, drone_id: droneId, state: 'OFFLINE', cached_packets: null,
        last_sync_sec: null, rssi_dbm: null,
    }, connected,
});

export function useTelemetry(wsUrl: string, selectedId: string) {
    const [fleet, setFleet] = useState<Fleet>({});
    const [connected, setConnected] = useState(false);

    useEffect(() => {
        let disposed = false;
        let reconnectTimer: number | undefined;
        let socket: WebSocket | undefined;

        function connect() {
            if (disposed) return;
            socket = new WebSocket(wsUrl);
            socket.onopen = () => { if (!disposed) setConnected(true); };
            socket.onmessage = (event) => {
                if (disposed) return;
                try {
                    const message = JSON.parse(event.data);
                    if (message.type !== 'fleet_snapshot' || typeof message.data !== 'object') return;
                    const snapshots = message.data as Record<string, VehicleSnapshot>;
                    setFleet(previous => {
                        const next: Fleet = {};
                        for (const [droneId, data] of Object.entries(snapshots)) {
                            if (!data.linkStatus || data.linkStatus.drone_id !== droneId) continue;
                            const before = previous[droneId];
                            const track = before?.track ?? [];
                            const pose = data.pose as DronePose | null;
                            const changed = pose && before?.pose?.timestamp !== pose.timestamp;
                            next[droneId] = {
                                ...data, connected: true,
                                track: changed ? [...track, pose].slice(-500) : track,
                            };
                        }
                        return next;
                    });
                } catch (error) {
                    console.error('Invalid telemetry snapshot', error);
                }
            };
            socket.onclose = () => {
                if (disposed) return;
                setConnected(false);
                setFleet(previous => Object.fromEntries(Object.entries(previous).map(([id, value]) =>
                    [id, { ...value, pose: null, connected: false,
                           linkStatus: { ...value.linkStatus!, state: 'OFFLINE' as const } }])));
                reconnectTimer = window.setTimeout(connect, 2000);
            };
            socket.onerror = () => socket?.close();
        }

        connect();
        return () => {
            disposed = true;
            window.clearTimeout(reconnectTimer);
            socket?.close();
        };
    }, [wsUrl]);

    return {
        state: fleet[selectedId] ?? emptyVehicle(selectedId, connected),
        fleet,
        connected,
    };
}
