import type { Heartbeat, DroneStatus } from "./types";

export function Vehicle({ name, heartbeat, droneStatus }: { name: string; heartbeat?: Heartbeat; droneStatus?: DroneStatus }) {
    return (
        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">{name}</p>

          {heartbeat ? (
            <>
              <p className="text-center mb-2">
                {new Date(heartbeat.sent_at).toTimeString()}
              </p>
              <div className="text-xl grid grid-cols-2">
                <p><span className="font-semibold">State:</span> {stateToText(heartbeat.state)}</p>
                <p><span className="font-semibold">Task:</span> {taskToText(heartbeat.current_task)}</p>
                <p><span className="font-semibold">Altitude:</span> {heartbeat.altitude_hae_m} m</p>
                <p><span className="font-semibold">Speed:</span> {heartbeat.spd_mps} m/s</p>

                {droneStatus && (
                    <>
                        <p><span className="font-semibold">Battery:</span> {droneStatus.battery_percentage}%</p>
                        <p><span className="font-semibold">Remaining:</span> {droneStatus.battery_time} s</p>
                        <p><span className="font-semibold">Local RSSI Margin:</span> {droneStatus.local_rssi_margin}</p>
                        <p><span className="font-semibold">Remote RSSI Margin:</span> {droneStatus.remote_rssi_margin}</p>
                    </>
                )}
              </div>
            </>
          ) : (
            <p className="text-center text-xl">No heartbeats yet</p>
          )}
        </div>
    )
}

function stateToText(stateNum: number) {
  switch (stateNum) {
    case 1:
      return "Killed"
    case 2:
      return "Manual"
    case 3:
      return "Auto"
    default:
      return "Unknown"
  }
}

function taskToText(taskNum: number) {
  switch (taskNum) {
    case 1:
      return "None"
    case 2:
      return "1 (Safe Passage)"
    case 3:
      return "2 (Infra Survey Repair)"
    case 4:
      return "3 (Coordinated Logistics)"
    case 5:
      return "4 (Dynamic Incident Response)"
    default:
      return "Unknown"
  }
}
