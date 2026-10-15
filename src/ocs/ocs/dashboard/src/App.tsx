import { useEffect, useState } from 'react'
import './App.css'

function App() {
  const [robocommand, setRobocommand] = useState(false);

  useEffect(() => {
    const interval = setInterval(async () => {
      const response = await fetch("/api/status").then(r => r.json());

      setRobocommand(response.robocommand);
    }, 1000);

    return () => {
      clearInterval(interval);
    }
  }, []);

  return (
    <>
      <p>hello</p>
      <p>{robocommand ? "connected" : "disconnected"}</p>
    </>
  )
}

export default App
