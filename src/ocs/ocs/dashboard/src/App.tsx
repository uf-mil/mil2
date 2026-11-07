import { useEffect, useState } from 'react'
import './App.css'
import 'leaflet/dist/leaflet.css';

import { Vehicle } from './Vehicle';
import { Map } from './Map';

function App() {
  const [robocommand, setRobocommand] = useState(false);
  const [subjugatorHeartbeat, setSubjugatorHeartbeat] = useState<any>(null);
  const [propagatorHeartbeat, setPropagatorHeartbeat] = useState<any>(null);
  const [investigatorHeartbeat, setInvestigatorHeartbeat] = useState<any>(null);
  const [droneStatus, setDroneStatus] = useState<any>(null);

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

  function badGeofence() {
    fetch("/api/bad_drone_geofence", { method: "POST" });
  }

  function badHeight() {
    fetch("/api/bad_drone_height", { method: "POST" });
  }

  return (
    <div className="flex flex-col p-10">
      <header className="flex gap-5 w-full justify-center items-center mb-6">
        <img src="/mil.svg" />
        <p className="text-3xl text-bold">MIL RobotX 2026 Dashboard</p>
      </header>

      <div className="flex flex-row gap-5">
        <div className="grow p-5 rounded-md border-2 border-black bg-white flex flex-col justify-center items-center mb-5">
          <p className="text-xl">No alerts</p>
        </div>

        <div className="flex flex-col gap-1">
          <button className="px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClick={badGeofence}>Bad geofence</button>
          <button className="px-5 py-1 border-black border-2 bg-white rounded-md cursor-pointer" onClickCapture={badHeight}>Bad height</button>
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
