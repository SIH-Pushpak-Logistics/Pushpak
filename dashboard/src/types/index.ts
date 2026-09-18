export interface DronePose {
    timestamp: number;
    drone_id: string;
    x: number;
    y: number;
    z: number;
    yaw_deg: number;
}

export interface DroneAltitude {
    timestamp: number;
    drone_id: string;
    z: number;
}

export interface DroneVelocity {
    timestamp: number;
    drone_id: string;
    linear_x: number;
    linear_y: number;
    linear_z: number;
    angular_z: number;
    is_valid: boolean;
    features: number;
}

export interface DroneFlowDebug {
    timestamp: number;
    drone_id: string;
    u_raw: number;
    v_raw: number;
    u_med: number;
    v_med: number;
    u_std: number;
    v_std: number;
    frame_diff: number;
    gyro_x: number;
    gyro_y: number;
    dt: number;
    altitude: number;
    features: number;
}

export interface Victim {
    victim_id: string;
    confidence: number;
    world_x: number;
    world_y: number;
    first_seen: number;
    last_seen: number;
    hit_count: number;
    image_path: string;
    acked: boolean;
}

export interface LinkStatus {
    timestamp: number;
    drone_id: string;
    state: 'ONLINE' | 'DEGRADED' | 'OFFLINE';
    cached_packets: number;
    last_sync_sec: number;
    rssi_dbm: number;
}

export interface DashboardState {
    pose: DronePose | null;
    altitude: DroneAltitude | null;
    velocity: DroneVelocity | null;
    flowDebug: DroneFlowDebug | null;
    linkStatus: LinkStatus | null;
    victims: Victim[];
    track: DronePose[];
    connected: boolean;
}
