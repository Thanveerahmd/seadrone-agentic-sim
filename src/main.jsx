import React, { useState } from 'react'
import ReactDOM from 'react-dom/client'
import ManualApp from '../seadrone_v6.jsx'
import AutopilotApp from '../seadrone_v5.jsx'

function AppSelector() {
  var [mode, setMode] = useState("manual")
  var btnBase = { border: "2px solid", cursor: "pointer", fontFamily: "monospace", fontSize: 12, fontWeight: 700, padding: "6px 20px", borderRadius: 6 }
  return (
    <div style={{ width: "100vw", height: "100vh", display: "flex", flexDirection: "column", overflow: "hidden" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, padding: "6px 14px", background: "#0a1520", borderBottom: "2px solid #1a3a4a", flexShrink: 0, zIndex: 100 }}>
        <span style={{ fontFamily: "monospace", fontSize: 13, fontWeight: 800, color: "#0a8a5a", letterSpacing: 2, marginRight: 10 }}>SEADRONE</span>
        <button onClick={function() { setMode("manual"); }} style={{
          ...btnBase,
          background: mode === "manual" ? "#c03030" : "transparent",
          borderColor: "#c03030",
          color: mode === "manual" ? "#fff" : "#c03030"
        }}>MANUAL</button>
        <button onClick={function() { setMode("autopilot"); }} style={{
          ...btnBase,
          background: mode === "autopilot" ? "#0a8a5a" : "transparent",
          borderColor: "#0a8a5a",
          color: mode === "autopilot" ? "#fff" : "#0a8a5a"
        }}>AUTOPILOT</button>
        <span style={{ fontFamily: "monospace", fontSize: 9, color: "#4a7a8a", marginLeft: 10 }}>
          {mode === "manual" ? "WASD to steer target — drones hunt autonomously — blast on triangulation" : "Pre-scripted scenarios — timeline playback — scenario & protocol selector"}
        </span>
      </div>
      <div style={{ flex: 1, overflow: "hidden", position: "relative" }}>
        {mode === "manual" ? <ManualApp /> : <AutopilotApp />}
      </div>
    </div>
  )
}

ReactDOM.createRoot(document.getElementById('root')).render(
  <React.StrictMode>
    <AppSelector />
  </React.StrictMode>
)
