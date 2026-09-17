import { useState, useEffect, useRef } from 'react';
import { type DashboardState, type DronePose, type DroneAltitude, type DroneVelocity, type DroneFlowDebug, type LinkStatus, type Victim } from '../types';

export function useTelemetry(wsUrl: string) {
    const [state, setState] = useState<DashboardState>({
        pose: null,
        altitude: null,
        velocity: null,
        flowDebug: null,
        linkStatus: null,
        victims: [],
        track: [],
        connected: false
    });

    const wsRef = useRef<WebSocket | null>(null);

    useEffect(() => {
        let reconnectTimer: number;

        function connect() {
            const ws = new WebSocket(wsUrl);
            wsRef.current = ws;

            ws.onopen = () => {
                setState(s => ({ ...s, connected: true }));
            };

            ws.onmessage = (event) => {
                try {
                    const message = JSON.parse(event.data);
                    const { type, data } = message;

                    setState(prev => {
                        const newState = { ...prev };
                        if (type === 'pose') {
                            newState.pose = data as DronePose;
                            // Add to track (max 500 points)
                            const track = [...prev.track, newState.pose];
                            if (track.length > 500) track.shift();
                            newState.track = track;
                        } else if (type === 'altitude') {
                            newState.altitude = data as DroneAltitude;
                        } else if (type === 'velocity') {
                            newState.velocity = data as DroneVelocity;
                        } else if (type === 'flow_debug') {
                            newState.flowDebug = data as DroneFlowDebug;
                        } else if (type === 'status') {
                            newState.linkStatus = data as LinkStatus;
                        } else if (type === 'victims') {
                            newState.victims = data as Victim[];
                        }
                        return newState;
                    });
                } catch (e) {
                    console.error('Failed to parse WebSocket message', e);
                }
            };

            ws.onclose = () => {
                setState(s => ({ ...s, connected: false }));
                reconnectTimer = setTimeout(connect, 2000);
            };

            ws.onerror = (err) => {
                console.error('WebSocket Error', err);
                ws.close();
            };
        }

        connect();

        return () => {
            clearTimeout(reconnectTimer);
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, [wsUrl]);

    return state;
}
