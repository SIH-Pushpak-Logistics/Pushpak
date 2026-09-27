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
        let disposed = false;

        function connect() {
            if (disposed) return;
            const ws = new WebSocket(wsUrl);
            wsRef.current = ws;

            ws.onopen = () => {
                if (disposed) return;
                setState(s => ({ ...s, connected: true }));
            };

            ws.onmessage = (event) => {
                if (disposed) return;
                try {
                    const message = JSON.parse(event.data);
                    const { type, data } = message;

                    setState(prev => {
                        const newState = { ...prev };
                        if (type === 'snapshot') {
                            const sameDrone = prev.linkStatus?.drone_id === data.linkStatus?.drone_id;
                            const track = sameDrone ? prev.track : [];
                            const changed = data.pose && (prev.pose?.timestamp !== data.pose.timestamp || !sameDrone);
                            return { ...prev, ...data, connected: true,
                                track: changed ? [...track, data.pose].slice(-500) : track };
                        } else if (type === 'pose') {
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
                if (disposed) return;
                setState(s => ({ ...s, connected: false, pose: null, linkStatus: null }));
                reconnectTimer = setTimeout(connect, 2000);
            };

            ws.onerror = (err) => {
                console.error('WebSocket Error', err);
                ws.close();
            };
        }

        connect();

        return () => {
            disposed = true;
            clearTimeout(reconnectTimer);
            if (wsRef.current) {
                wsRef.current.close();
            }
        };
    }, [wsUrl]);

    return state;
}
