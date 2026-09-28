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
    timestamp: number | null;
    drone_id: string;
    state: 'ONLINE' | 'DEGRADED' | 'OFFLINE';
    cached_packets: number;
    last_sync_sec: number | null;
    rssi_dbm: number | null;
}

export interface DashboardState {
    pose: DronePose | null;
    altitude: DroneAltitude | null;
    linkStatus: LinkStatus | null;
    victims: Victim[];
    track: DronePose[];
    connected: boolean;
}
