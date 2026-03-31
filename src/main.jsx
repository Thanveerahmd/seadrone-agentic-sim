import React, { useState } from 'react'
import ReactDOM from 'react-dom/client'
import ManualApp from '../seadrone_v6.jsx'
import AutopilotApp from '../seadrone_v5.jsx'

function AppSelector() {
  var [mode, setMode] = useState("manual")
  return (
    <div style={{ width: "100vw", height: "100vh", display: "flex", flexDirection: "column", overflow: "hidden", background: "#080c12" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 0, padding: 0, background: "#0c1018", borderBottom: "1px solid #141e2a", flexShrink: 0, zIndex: 100, height: 36 }}>
        <div style={{ padding: "0 16px", display: "flex", alignItems: "center", gap: 8, height: "100%", borderRight: "1px solid #141e2a" }}>
          <div style={{ width: 6, height: 6, borderRadius: "50%", background: "#0a8a5a", boxShadow: "0 0 6px #0a8a5a80" }} />
          <span style={{ fontFamily: "'JetBrains Mono', monospace", fontSize: 11, fontWeight: 700, color: "#e0e8f0", letterSpacing: 2 }}>SEADRONE</span>
        </div>
        <button onClick={function() { setMode("manual"); }} style={{
          border: "none", cursor: "pointer", fontFamily: "'JetBrains Mono', monospace", fontSize: 10, fontWeight: 600,
          padding: "0 20px", height: "100%", letterSpacing: 1,
          background: mode === "manual" ? "#141e2a" : "transparent",
          color: mode === "manual" ? "#ff4444" : "#4a5a6a",
          borderBottom: mode === "manual" ? "2px solid #ff4444" : "2px solid transparent"
        }}>MANUAL</button>
        <button onClick={function() { setMode("autopilot"); }} style={{
          border: "none", cursor: "pointer", fontFamily: "'JetBrains Mono', monospace", fontSize: 10, fontWeight: 600,
          padding: "0 20px", height: "100%", letterSpacing: 1,
          background: mode === "autopilot" ? "#141e2a" : "transparent",
          color: mode === "autopilot" ? "#00cc88" : "#4a5a6a",
          borderBottom: mode === "autopilot" ? "2px solid #00cc88" : "2px solid transparent"
        }}>AUTOPILOT</button>
        <div style={{ flex: 1 }} />
        <span style={{ fontFamily: "'JetBrains Mono', monospace", fontSize: 8, color: "#2a3a4a", padding: "0 16px", letterSpacing: 0.5 }}>
          {mode === "manual" ? "WASD TARGET CONTROL · AUTONOMOUS DRONES · STRIKE ON TRIANGULATION" : "SCRIPTED SCENARIOS · TIMELINE PLAYBACK · PROTOCOL COMPARISON"}
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
