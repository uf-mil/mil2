import { useEffect, useState } from 'react'
import './App.css'
import 'leaflet/dist/leaflet.css';

import { Vehicle } from './Vehicle';
import { Map } from './Map';
import type { DroneStatus, Heartbeat } from './types';

function App() {
  const [robocommand, setRobocommand] = useState(false);
  const [subjugatorHeartbeat, setSubjugatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [propagatorHeartbeat, setPropagatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [investigatorHeartbeat, setInvestigatorHeartbeat] = useState<Heartbeat | undefined>(undefined);
  const [droneStatus, setDroneStatus] = useState<DroneStatus | undefined>(undefined);
  const [alerts, setAlerts] = useState<string[]>([]);

  useEffect(() => {
    const interval = setInterval(async () => {
      const response = await fetch("/api/status").then(r => r.json());
      const drone_response = await fetch("/api/drone_status").then(r => r.json());

      setRobocommand(response.robocommand);
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
    <div className="flex flex-col p-10">
      <header className="flex gap-5 w-full justify-center items-center mb-6">
        <img src="/mil.svg" />
        <p className="text-3xl text-bold">MIL RobotX 2026 Dashboard</p>
      </header>

      <div className="flex flex-row gap-5">
        <div className="grow p-5 rounded-md border-2 border-black bg-white flex flex-row gap-10 justify-center items-center mb-5">
          {alerts.length == 0 ? <p className="text-xl">No alerts</p> : alerts.map(a => (
            <p className="text-xl">{a}</p>
          ))}
        </div>

        <div className="flex flex-col gap-1">
          <button className="px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClick={badGeofence}>Bad geofence</button>
          <button className="px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClickCapture={badHeight}>Bad height</button>
          <button className="px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClickCapture={goodDrone}>Good</button>
        </div>
      </div>

      <div className="grid grid-cols-4 gap-5">
        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Robocommand</p>
          <p className="text-xl text-center">{robocommand ? "Connected" : "Disconnected"}</p>
        </div>

        <Vehicle name="SubjuGator" heartbeat={subjugatorHeartbeat} />
        <Vehicle name="PropaGator" heartbeat={propagatorHeartbeat} />
        <Vehicle name="InvestiGator" heartbeat={investigatorHeartbeat} droneStatus={droneStatus} />
      </div>

      <Map subHeartbeat={subjugatorHeartbeat} boatHeartbeat={propagatorHeartbeat} droneHeartbeat={investigatorHeartbeat} />
    </div>
  )
}

export default App
