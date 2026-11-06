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
}

export interface DroneStatus {
    battery_percentage: number;
    battery_time: number;
}
