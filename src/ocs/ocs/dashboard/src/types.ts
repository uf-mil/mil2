export interface Heartbeat {
    sent_at: string;
    state: number;
    current_task: number;
    altitude_hae_m: number;
    spd_mps: number;
    position: {
        latitude: number;
        longitude: number;
    }
    heading_deg: number;
    flight_phase: number;
}

export interface DroneStatus {
    battery_percentage: number;
    battery_time: number;
    local_rssi_margin: number;
    remote_rssi_margin: number;
    geofence: LatLng[];
}

export interface LatLng {
    latitude: number;
    longitude: number;
}

export interface RobocommandStatus {
    connected: boolean;
    course_received: boolean;
    run_start_received: boolean;
}
