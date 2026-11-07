import { useEffect, useState } from 'react'
import './App.css'
import 'leaflet/dist/leaflet.css';

import { Vehicle } from './Vehicle';
import { Map } from './Map';
import type { DroneStatus, Heartbeat, RobocommandStatus } from './types';

function App() {
  const [robocommandStatus, setRobocommandStatus] = useState<RobocommandStatus | undefined>(undefined);
  const [subjugatorHeartbeat, setSubjugatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [propagatorHeartbeat, setPropagatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [investigatorHeartbeat, setInvestigatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [droneStatus, setDroneStatus] = useState<DroneStatus | undefined>(undefined);
  const [alerts, setAlerts] = useState<string[]>([]);

  useEffect(() => {
    const interval = setInterval(async () => {
      const response = await fetch("/api/status").then(r => r.json());
      const drone_response = await fetch("/api/drone_status").then(r => r.json());

      setRobocommandStatus(response.robocommand);
      setSubjugatorHeartbeat(response.subjugatorHeartbeat);
      setPropagatorHeartbeat(response.propagatorHeartbeat);
      setInvestigatorHeartbeat(response.investigatorHeartbeat);
      setDroneStatus(drone_response);
    }, 1000);

    return () => {
      clearInterval(interval);
    }
  }, []);

  useEffect(refreshAlerts, [droneStatus, subjugatorHeartbeat, propagatorHeartbeat, investigatorHeartbeat]);
  useEffect(() => {
    const interval = setInterval(refreshAlerts, 1000);

    return () => {
        clearInterval(interval);
    }
  }, [droneStatus, subjugatorHeartbeat, propagatorHeartbeat, investigatorHeartbeat]);

  function refreshAlerts() {
    let newAlerts = [];

    if (droneStatus) {
      if (droneStatus.local_rssi_margin < 20) {
        newAlerts.push("Low local RSSI margin")
      }

      if (droneStatus.remote_rssi_margin < 20) {
        newAlerts.push("Low remote RSSI margin")
      }
    }

    const current_millis = new Date().valueOf();

    if (subjugatorHeartbeat && current_millis - new Date(subjugatorHeartbeat.sent_at).valueOf() > 2000) {
        newAlerts.push(">2 seconds since last sub heartbeat")
    }

    if (propagatorHeartbeat && current_millis - new Date(propagatorHeartbeat.sent_at).valueOf() > 2000) {
        newAlerts.push(">2 seconds since last boat heartbeat")
    }

    if (investigatorHeartbeat && current_millis - new Date(investigatorHeartbeat.sent_at).valueOf() > 2000) {
        newAlerts.push(">2 seconds since last drone heartbeat")
    }

    setAlerts(newAlerts);
  }

  function badGeofence() {
    fetch("/api/bad_drone_geofence", { method: "POST" });
  }

  function badHeight() {
    fetch("/api/bad_drone_height", { method: "POST" });
  }

  function goodDrone() {
    fetch("/api/good_drone", { method: "POST" });
  }

  return (
    <div className="flex flex-col p-5">
      <div className="flex flex-row gap-2 mb-2">
        <img src="/mil.svg" className="w-32" />
        <div className="grow px-5 py-2 min-h-full rounded-md border-2 border-black bg-white flex flex-col gap-2 justify-center items-center">
          <p className="text-2xl font-semibold">Alerts</p>
          <div className="flex flex-row justify-center items-center gap-10">
            {alerts.length == 0 ? <p className="text-xl">No alerts</p> : alerts.map(a => (
              <p className="text-xl">{a}</p>
            ))}
          </div>
        </div>

        <div className="flex flex-col gap-1">
          <button className="grow px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClick={badGeofence}>Send bad geofence RunDecl</button>
          <button className="grow px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClickCapture={badHeight}>Send bad height RunDecl</button>
          <button className="grow px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClickCapture={goodDrone}>Send good RunDecl</button>
        </div>
      </div>

      <div className="grid grid-cols-4 gap-5">
        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Robocommand</p>
          <p className="text-xl text-center">{robocommandStatus?.connected ? "Connected" : "Disconnected"}</p>
          <p className="text-xl text-center">{robocommandStatus?.course_received ? "Course received" : "No course"}</p>
          <p className="text-xl text-center">{robocommandStatus?.run_start_received ? "RunStart received" : "Waiting for RunStart"}</p>
        </div>

        <Vehicle name="SubjuGator" heartbeat={subjugatorHeartbeat} />
        <Vehicle name="PropaGator" heartbeat={propagatorHeartbeat} />
        <Vehicle name="InvestiGator" heartbeat={investigatorHeartbeat} droneStatus={droneStatus} />
      </div>

      <Map subHeartbeat={subjugatorHeartbeat} boatHeartbeat={propagatorHeartbeat} droneHeartbeat={investigatorHeartbeat} geofence={droneStatus?.geofence} />
    </div>
  )
}

export default App
