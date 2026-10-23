import { useEffect, useState } from 'react'
import './App.css'

function App() {
  const [robocommand, setRobocommand] = useState(false);
  const [subjugatorHeartbeat, setSubjugatorHeartbeat] = useState<any>(null);
  const [propagatorHeartbeat, setPropagatorHeartbeat] = useState<any>(null);
  const [investigatorHeartbeat, setInvestigatorHeartbeat] = useState<any>(null);

  useEffect(() => {
    const interval = setInterval(async () => {
      const response = await fetch("/api/status").then(r => r.json());

      setRobocommand(response.robocommand);
      setSubjugatorHeartbeat(response.subjugatorHeartbeat);
      setPropagatorHeartbeat(response.propagatorHeartbeat);
      setInvestigatorHeartbeat(response.investigatorHeartbeat);
    }, 1000);

    return () => {
      clearInterval(interval);
    }
  }, []);

  return (
    <div className="flex flex-col p-10">
      <header className="flex gap-5 w-full justify-center items-center mb-10">
        <img src="/mil.svg" />
        <p className="text-3xl text-bold">MIL RobotX 2026 Dashboard</p>
      </header>

      <div className="grid grid-cols-4 gap-5">
        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Robocommand</p>
          <p className="text-xl text-center">{robocommand ? "Connected" : "Disconnected"}</p>
        </div>

        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Subjugator</p>

          {subjugatorHeartbeat ? (
            <>
              <p className="text-center mb-2">
                {new Date(subjugatorHeartbeat.sent_at).toTimeString()}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">State:</span> {stateToText(subjugatorHeartbeat.state)}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">Task:</span> {taskToText(subjugatorHeartbeat.current_task)}
              </p>
            </>
          ) : (
            <p className="text-center text-xl">No heartbeats yet</p>
          )}
        </div>

        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Propagator</p>

          {propagatorHeartbeat ? (
            <>
              <p className="text-center mb-2">
                {new Date(propagatorHeartbeat.sent_at).toTimeString()}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">State:</span> {stateToText(propagatorHeartbeat.state)}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">Task:</span> {taskToText(propagatorHeartbeat.current_task)}
              </p>
            </>
          ) : (
            <p className="text-center text-xl">No heartbeats yet</p>
          )}
        </div>

        <div className="p-5 rounded-md border-2 border-black bg-white">
          <p className="font-bold text-2xl text-center">Investigator</p>

          {investigatorHeartbeat ? (
            <>
              <p className="text-center mb-2">
                {new Date(investigatorHeartbeat.sent_at).toTimeString()}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">State:</span> {stateToText(investigatorHeartbeat.state)}
              </p>
              <p className="text-xl text-center">
                <span className="font-semibold">Task:</span> {taskToText(investigatorHeartbeat.current_task)}
              </p>
            </>
          ) : (
            <p className="text-center text-xl">No heartbeats yet</p>
          )}
        </div>
      </div>
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

export default App
