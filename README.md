# SeaDrone Multi-Agent Simulation

<div align="center">

<!-- System Overview Diagram -->
<svg xmlns="http://www.w3.org/2000/svg" width="700" height="200" viewBox="0 0 700 200">
  <defs>
    <linearGradient id="ocean" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#1a6090"/><stop offset="1" stop-color="#0a3050"/></linearGradient>
    <linearGradient id="sky" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="#3a7ec5"/><stop offset="1" stop-color="#87ceeb"/></linearGradient>
  </defs>
  <rect width="700" height="100" fill="url(#sky)" rx="8" />
  <rect y="100" width="700" height="100" fill="url(#ocean)" rx="0"/>
  <rect width="700" height="200" fill="none" stroke="#1a4a6a" stroke-width="2" rx="8"/>
  <!-- GS -->
  <rect x="330" y="150" width="40" height="30" rx="4" fill="#2a3a4a" stroke="#f0a030" stroke-width="2"/>
  <text x="350" y="170" text-anchor="middle" fill="#f0a030" font-size="10" font-weight="bold" font-family="monospace">GS</text>
  <line x1="350" y1="150" x2="350" y2="135" stroke="#888" stroke-width="2"/>
  <circle cx="350" cy="132" r="4" fill="#f0a030"/>
  <!-- Drone A -->
  <polygon points="140,115 155,105 170,115" fill="#0a8a5a" stroke="#fff" stroke-width="1.5"/>
  <text x="155" y="100" text-anchor="middle" fill="#0a8a5a" font-size="11" font-weight="bold" font-family="monospace">DRONE A</text>
  <!-- FOV A -->
  <path d="M155,115 L95,145 A65,65 0 0,1 215,145 Z" fill="#0a8a5a" fill-opacity="0.12" stroke="#0a8a5a" stroke-width="1" stroke-dasharray="4 3"/>
  <!-- Drone B -->
  <polygon points="490,115 505,105 520,115" fill="#4838d0" stroke="#fff" stroke-width="1.5"/>
  <text x="505" y="100" text-anchor="middle" fill="#4838d0" font-size="11" font-weight="bold" font-family="monospace">DRONE B</text>
  <!-- FOV B -->
  <path d="M505,115 L445,145 A65,65 0 0,1 565,145 Z" fill="#4838d0" fill-opacity="0.12" stroke="#4838d0" stroke-width="1" stroke-dasharray="4 3"/>
  <!-- Target -->
  <circle cx="350" cy="110" r="8" fill="#d03030" stroke="#fff" stroke-width="2"/>
  <text x="350" y="90" text-anchor="middle" fill="#d03030" font-size="10" font-weight="bold" font-family="monospace">TARGET</text>
  <!-- Bearing lines -->
  <line x1="155" y1="115" x2="350" y2="110" stroke="#c89020" stroke-width="1.5" stroke-dasharray="6 4" opacity="0.6"/>
  <line x1="505" y1="115" x2="350" y2="110" stroke="#c06020" stroke-width="1.5" stroke-dasharray="6 4" opacity="0.6"/>
  <!-- Comm beams -->
  <line x1="155" y1="118" x2="345" y2="150" stroke="#f0a030" stroke-width="1" stroke-dasharray="4 3" opacity="0.5"/>
  <line x1="505" y1="118" x2="355" y2="150" stroke="#f0a030" stroke-width="1" stroke-dasharray="4 3" opacity="0.5"/>
  <!-- Title -->
  <text x="350" y="25" text-anchor="middle" fill="#fff" font-size="18" font-weight="bold" font-family="Segoe UI, Arial, sans-serif">GPS-Denied Multi-Agent Triangulation</text>
  <text x="350" y="48" text-anchor="middle" fill="#c0d8e8" font-size="12" font-family="Segoe UI, Arial, sans-serif">Bearing-Only Target Localization via Autonomous Sea Drones</text>
</svg>

**Real-time 3D simulation of autonomous sea drone multi-agent coordination for maritime target tracking and triangulation.**

Built with **React** + **Three.js** + **Vite** | [Technical Proposal (PDF)](docs/TECHNICAL_PROPOSAL.pdf)

</div>

---

## System Architecture

<!-- Architecture Diagram -->
<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" width="700" height="320" viewBox="0 0 700 320">
  <rect width="700" height="320" fill="#f8fafb" rx="8" stroke="#d0e0e8" stroke-width="1"/>
  <!-- Drone A Box -->
  <rect x="20" y="20" width="200" height="180" rx="8" fill="#eaf6f0" stroke="#0a8a5a" stroke-width="2"/>
  <text x="120" y="42" text-anchor="middle" fill="#0a8a5a" font-size="12" font-weight="bold" font-family="monospace">DRONE A</text>
  <rect x="35" y="52" width="80" height="28" rx="4" fill="#fff" stroke="#0a8a5a" stroke-width="1"/>
  <text x="75" y="70" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">RouteCam</text>
  <rect x="125" y="52" width="80" height="28" rx="4" fill="#0a8a5a" stroke="#0a8a5a" stroke-width="1"/>
  <text x="165" y="70" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">Jetson</text>
  <rect x="35" y="90" width="80" height="28" rx="4" fill="#fff" stroke="#2a5a7a" stroke-width="1"/>
  <text x="75" y="108" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">YOLOv8</text>
  <rect x="125" y="90" width="80" height="28" rx="4" fill="#fff" stroke="#2a5a7a" stroke-width="1"/>
  <text x="165" y="108" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">SAMURAI</text>
  <rect x="35" y="128" width="170" height="28" rx="4" fill="#fff" stroke="#c89020" stroke-width="1"/>
  <text x="120" y="146" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">Pixhawk V6X (IMU/Compass)</text>
  <rect x="35" y="166" width="170" height="24" rx="4" fill="#e8f0f8" stroke="#2a5a7a" stroke-width="1"/>
  <text x="120" y="182" text-anchor="middle" fill="#2a5a7a" font-size="8" font-family="monospace">Radio Mesh | D-Link Switch</text>
  <!-- Drone B Box -->
  <rect x="480" y="20" width="200" height="180" rx="8" fill="#eef0f8" stroke="#4838d0" stroke-width="2"/>
  <text x="580" y="42" text-anchor="middle" fill="#4838d0" font-size="12" font-weight="bold" font-family="monospace">DRONE B</text>
  <rect x="495" y="52" width="80" height="28" rx="4" fill="#fff" stroke="#4838d0" stroke-width="1"/>
  <text x="535" y="70" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">RouteCam</text>
  <rect x="585" y="52" width="80" height="28" rx="4" fill="#4838d0" stroke="#4838d0" stroke-width="1"/>
  <text x="625" y="70" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">Jetson</text>
  <rect x="495" y="90" width="80" height="28" rx="4" fill="#fff" stroke="#2a5a7a" stroke-width="1"/>
  <text x="535" y="108" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">YOLOv8</text>
  <rect x="585" y="90" width="80" height="28" rx="4" fill="#fff" stroke="#2a5a7a" stroke-width="1"/>
  <text x="625" y="108" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">SAMURAI</text>
  <rect x="495" y="128" width="170" height="28" rx="4" fill="#fff" stroke="#c89020" stroke-width="1"/>
  <text x="580" y="146" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">Pixhawk V6X (IMU/Compass)</text>
  <rect x="495" y="166" width="170" height="24" rx="4" fill="#e8f0f8" stroke="#2a5a7a" stroke-width="1"/>
  <text x="580" y="182" text-anchor="middle" fill="#2a5a7a" font-size="8" font-family="monospace">Radio Mesh | D-Link Switch</text>
  <!-- GS Box -->
  <rect x="250" y="230" width="200" height="70" rx="8" fill="#fef8e8" stroke="#f0a030" stroke-width="2"/>
  <text x="350" y="255" text-anchor="middle" fill="#f0a030" font-size="12" font-weight="bold" font-family="monospace">GROUND STATION</text>
  <text x="350" y="272" text-anchor="middle" fill="#1a2a3a" font-size="9" font-family="monospace">Mac Workstation | Mission Planner</text>
  <text x="350" y="288" text-anchor="middle" fill="#6a8a9a" font-size="8" font-family="monospace">Orchestrator Agent | Radio Mesh</text>
  <!-- D2G Links (amber dashed) -->
  <line x1="120" y1="195" x2="290" y2="235" stroke="#f0a030" stroke-width="2" stroke-dasharray="6 4"/>
  <line x1="580" y1="195" x2="410" y2="235" stroke="#f0a030" stroke-width="2" stroke-dasharray="6 4"/>
  <text x="190" y="220" fill="#f0a030" font-size="9" font-weight="bold" font-family="monospace">D2G</text>
  <text x="490" y="220" fill="#f0a030" font-size="9" font-weight="bold" font-family="monospace">D2G</text>
  <!-- D2D Link (cyan solid) -->
  <line x1="220" y1="110" x2="480" y2="110" stroke="#20d090" stroke-width="2.5"/>
  <text x="350" y="105" text-anchor="middle" fill="#20d090" font-size="10" font-weight="bold" font-family="monospace">D2D DIRECT</text>
</svg>
</div>

---

## Features

### Scenarios (selectable)
- **Scenario 1: Detect & Triangulate** — A detects target, relays to B, both converge and triangulate
- **Scenario 2: Lose & Re-acquire** — A locks target, target escapes A's FOV, B re-acquires and relays back to A

### Communication Protocols (selectable)

<!-- Protocol Comparison Diagram -->
<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" width="660" height="120" viewBox="0 0 660 120">
  <!-- D2D -->
  <rect x="0" y="0" width="310" height="120" rx="8" fill="#f0faf6" stroke="#20d090" stroke-width="1.5"/>
  <text x="155" y="22" text-anchor="middle" fill="#20d090" font-size="12" font-weight="bold" font-family="monospace">D2D — Drone to Drone</text>
  <circle cx="55" cy="70" r="18" fill="#0a8a5a" stroke="#fff" stroke-width="1.5"/><text x="55" y="74" text-anchor="middle" fill="#fff" font-size="10" font-weight="bold" font-family="monospace">A</text>
  <circle cx="255" cy="70" r="18" fill="#4838d0" stroke="#fff" stroke-width="1.5"/><text x="255" y="74" text-anchor="middle" fill="#fff" font-size="10" font-weight="bold" font-family="monospace">B</text>
  <line x1="73" y1="70" x2="237" y2="70" stroke="#20d090" stroke-width="3"/>
  <text x="155" y="65" text-anchor="middle" fill="#20d090" font-size="9" font-family="monospace">DIRECT ~12ms</text>
  <text x="155" y="108" text-anchor="middle" fill="#6a8a9a" font-size="9" font-family="monospace">Low latency | 0.8 Mbps | Decentralized</text>
  <!-- D2G -->
  <rect x="340" y="0" width="320" height="120" rx="8" fill="#fef8f0" stroke="#f0a030" stroke-width="1.5"/>
  <text x="500" y="22" text-anchor="middle" fill="#f0a030" font-size="12" font-weight="bold" font-family="monospace">D2G — Drone-Ground-Drone</text>
  <circle cx="385" cy="70" r="18" fill="#0a8a5a" stroke="#fff" stroke-width="1.5"/><text x="385" y="74" text-anchor="middle" fill="#fff" font-size="10" font-weight="bold" font-family="monospace">A</text>
  <rect x="482" y="55" width="32" height="30" rx="4" fill="#2a3a4a" stroke="#f0a030" stroke-width="1.5"/><text x="498" y="74" text-anchor="middle" fill="#f0a030" font-size="9" font-weight="bold" font-family="monospace">GS</text>
  <circle cx="615" cy="70" r="18" fill="#4838d0" stroke="#fff" stroke-width="1.5"/><text x="615" y="74" text-anchor="middle" fill="#fff" font-size="10" font-weight="bold" font-family="monospace">B</text>
  <line x1="403" y1="70" x2="482" y2="70" stroke="#f0a030" stroke-width="2" stroke-dasharray="5 3"/>
  <line x1="514" y1="70" x2="597" y2="70" stroke="#f0a030" stroke-width="2" stroke-dasharray="5 3"/>
  <text x="500" y="50" text-anchor="middle" fill="#f0a030" font-size="9" font-family="monospace">RELAY ~45ms</text>
  <text x="500" y="108" text-anchor="middle" fill="#6a8a9a" font-size="9" font-family="monospace">Orchestrated | 2.4 Mbps | Central logging</text>
</svg>
</div>

### Mission Phases

<!-- Mission Phase Timeline -->
<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" width="700" height="80" viewBox="0 0 700 80">
  <rect x="0" y="20" width="70" height="35" rx="6" fill="#0a8a5a"/><text x="35" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">DEPLOY</text>
  <rect x="80" y="20" width="100" height="35" rx="6" fill="#1a80c0"/><text x="130" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">SEARCH</text>
  <rect x="190" y="20" width="110" height="35" rx="6" fill="#c89020"/><text x="245" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">DETECT/LOCK</text>
  <rect x="310" y="20" width="100" height="35" rx="6" fill="#c06020"/><text x="360" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">RELAY</text>
  <rect x="420" y="20" width="100" height="35" rx="6" fill="#c03030"/><text x="470" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">TRIANGULATE</text>
  <rect x="530" y="20" width="160" height="35" rx="6" fill="#0a8a5a"/><text x="610" y="42" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">TRACK / CONTAINED</text>
  <text x="350" y="72" text-anchor="middle" fill="#6a8a9a" font-size="9" font-family="monospace">Autonomous multi-phase mission with playback controls (0.5x — 3x speed)</text>
</svg>
</div>

### Dashboard Panels

<!-- Dashboard Layout -->
<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" width="700" height="250" viewBox="0 0 700 250">
  <rect width="700" height="250" fill="#1a2530" rx="8" stroke="#2a4a5a" stroke-width="1"/>
  <!-- 3D View area -->
  <rect x="10" y="10" width="420" height="230" rx="6" fill="#0a1a28" stroke="#2a4a5a" stroke-width="1"/>
  <text x="220" y="130" text-anchor="middle" fill="#3a5a6a" font-size="14" font-family="monospace">3D OCEAN VIEW</text>
  <!-- FPV A -->
  <rect x="18" y="18" width="110" height="75" rx="4" fill="#0a1520" stroke="#0a8a5a" stroke-width="1.5"/>
  <text x="73" y="32" text-anchor="middle" fill="#0a8a5a" font-size="7" font-weight="bold" font-family="monospace">DRONE A FPV</text>
  <line x1="63" y1="55" x2="83" y2="55" stroke="#0f0" stroke-width="0.7" opacity="0.6"/>
  <line x1="73" y1="45" x2="73" y2="65" stroke="#0f0" stroke-width="0.7" opacity="0.6"/>
  <!-- IMU A -->
  <rect x="18" y="96" width="110" height="55" rx="4" fill="#0a1520" stroke="#0a8a5a40" stroke-width="1"/>
  <text x="73" y="109" text-anchor="middle" fill="#0a8a5a" font-size="7" font-weight="bold" font-family="monospace">IMU A</text>
  <text x="30" y="122" fill="#6a8a9a" font-size="6" font-family="monospace">HDG 161.2°</text>
  <text x="30" y="132" fill="#6a8a9a" font-size="6" font-family="monospace">SPD 3.2 m/s</text>
  <text x="30" y="142" fill="#ffd700" font-size="6" font-family="monospace">BRG 161° LOCKED</text>
  <!-- FPV B -->
  <rect x="302" y="18" width="110" height="75" rx="4" fill="#0a1520" stroke="#4838d0" stroke-width="1.5"/>
  <text x="357" y="32" text-anchor="middle" fill="#4838d0" font-size="7" font-weight="bold" font-family="monospace">DRONE B FPV</text>
  <line x1="347" y1="55" x2="367" y2="55" stroke="#0f0" stroke-width="0.7" opacity="0.6"/>
  <line x1="357" y1="45" x2="357" y2="65" stroke="#0f0" stroke-width="0.7" opacity="0.6"/>
  <!-- Comm Log -->
  <rect x="18" y="158" width="200" height="70" rx="4" fill="#0a1520" stroke="#20d09040" stroke-width="1"/>
  <text x="28" y="172" fill="#20d090" font-size="7" font-weight="bold" font-family="monospace">INTER-DRONE COMM</text>
  <text x="28" y="185" fill="#f0a030" font-size="6" font-family="monospace">[A→GS] TARGET at (-17,17) BRG 291°</text>
  <text x="28" y="196" fill="#f0a030" font-size="6" font-family="monospace">[GS→B] RELAY: Intercept heading 245°</text>
  <text x="28" y="207" fill="#4838d0" font-size="6" font-family="monospace">[B→GS] ACK. Proceeding to zone.</text>
  <!-- Spatial Map -->
  <rect x="440" y="10" width="250" height="145" rx="6" fill="#f4f8fb" stroke="#d0e0e8" stroke-width="1"/>
  <text x="565" y="28" text-anchor="middle" fill="#1a80c0" font-size="9" font-weight="bold" font-family="monospace">SPATIAL MAP</text>
  <polygon points="490,70 496,60 502,70" fill="#0a8a5a"/>
  <polygon points="600,80 606,70 612,80" fill="#4838d0"/>
  <circle cx="550" cy="90" r="5" fill="#d03030" stroke="#fff" stroke-width="1"/>
  <path d="M496,65 L470,50 A30,30 0 0,1 522,50 Z" fill="#0a8a5a" fill-opacity="0.1" stroke="#0a8a5a" stroke-width="0.5" stroke-dasharray="3 2"/>
  <rect x="536" y="120" width="18" height="14" rx="2" fill="#2a3a4a" stroke="#f0a030" stroke-width="1"/><text x="545" y="131" text-anchor="middle" fill="#f0a030" font-size="6" font-weight="bold" font-family="monospace">GS</text>
  <!-- Comm Dashboard -->
  <rect x="440" y="162" width="120" height="78" rx="4" fill="#0a1520" stroke="#f0a03040" stroke-width="1"/>
  <text x="500" y="177" text-anchor="middle" fill="#f0a030" font-size="7" font-weight="bold" font-family="monospace">COMM DASHBOARD</text>
  <text x="450" y="191" fill="#6a8a9a" font-size="6" font-family="monospace">PROTO: D2G RELAY</text>
  <text x="450" y="202" fill="#6a8a9a" font-size="6" font-family="monospace">ROUTE: A→GS→B</text>
  <text x="450" y="213" fill="#20d090" font-size="6" font-family="monospace">LAT: 45ms | BW: 2.1Mbps</text>
  <!-- Triang OPT -->
  <rect x="568" y="162" width="120" height="78" rx="4" fill="#0a1520" stroke="#c0602040" stroke-width="1"/>
  <text x="628" y="177" text-anchor="middle" fill="#c06020" font-size="7" font-weight="bold" font-family="monospace">TRIANG OPT</text>
  <text x="578" y="191" fill="#6a8a9a" font-size="6" font-family="monospace">SEP: 180° / 180°</text>
  <text x="578" y="202" fill="#0a8a5a" font-size="6" font-family="monospace">GDOP: 1.00 OPTIMAL</text>
  <text x="578" y="213" fill="#6a8a9a" font-size="6" font-family="monospace">ORBIT: 22m / 22m</text>
</svg>
</div>

### Detection Pipeline

<!-- Detection Pipeline -->
<div align="center">
<svg xmlns="http://www.w3.org/2000/svg" width="700" height="70" viewBox="0 0 700 70">
  <rect width="700" height="70" fill="#f8fafb" rx="6" stroke="#d0e0e8" stroke-width="1"/>
  <rect x="10" y="15" width="100" height="40" rx="6" fill="#2a5a7a"/><text x="60" y="39" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">RouteCam</text>
  <text x="126" y="39" fill="#6a8a9a" font-size="14">→</text>
  <rect x="140" y="15" width="100" height="40" rx="6" fill="#c89020"/><text x="190" y="39" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">YOLOv8-nano</text>
  <text x="256" y="39" fill="#6a8a9a" font-size="14">→</text>
  <rect x="270" y="15" width="100" height="40" rx="6" fill="#0a6a5a"/><text x="320" y="39" text-anchor="middle" fill="#fff" font-size="9" font-weight="bold" font-family="monospace">SAMURAI</text>
  <text x="386" y="39" fill="#6a8a9a" font-size="14">→</text>
  <rect x="400" y="15" width="100" height="40" rx="6" fill="#2a5a7a"/><text x="450" y="33" text-anchor="middle" fill="#fff" font-size="8" font-weight="bold" font-family="monospace">Bearing</text><text x="450" y="45" text-anchor="middle" fill="#c0d8e8" font-size="7" font-family="monospace">Estimator</text>
  <text x="516" y="39" fill="#6a8a9a" font-size="14">→</text>
  <rect x="530" y="15" width="155" height="40" rx="6" fill="#0a4a6a"/><text x="607" y="33" text-anchor="middle" fill="#fff" font-size="8" font-weight="bold" font-family="monospace">Decision Engine</text><text x="607" y="45" text-anchor="middle" fill="#c0d8e8" font-size="7" font-family="monospace">PID → RC_OVERRIDE</text>
</svg>
</div>

---

## Prerequisites

- [Node.js](https://nodejs.org/) v18 or higher
- npm (comes with Node.js)

## Setup

```bash
# 1. Clone the repository
git clone https://github.com/Thanveerahmd/seadrone-agentic-sim.git
cd seadrone-agentic-sim

# 2. Install dependencies
npm install

# 3. Start the development server
npm run dev
```

Open **http://localhost:5173** in your browser.

## Build for Production

```bash
npm run build    # Build optimized static files to dist/
npm run preview  # Preview the production build locally
```

## Controls

| Action | Input |
|--------|-------|
| Orbit camera | Click + drag on 3D view |
| Zoom | Scroll wheel |
| Resize panels | Drag the vertical divider |
| Expand spatial map | Click "Expand Map" button |
| Switch scenario | Dropdown in header |
| Switch comm protocol | Dropdown in header (D2D / D2G) |
| Expand FPV camera | Click on drone FPV panel |
| Jump to phase | Click a phase label on the timeline |
| Scrub timeline | Drag the slider |

## Version History

| Version | Branch | Description |
|---------|--------|-------------|
| v1.0 | `main` | Initial simulation — basic 3D/2D, single scenario |
| v2.0 | `v2-dev` | Scenario selector, V-shaped FOV cones, lose & re-acquire scenario |
| v3.0 | `v3-dev` | 3D FOV alignment fix — dynamic vertex rebuild |
| v4.0 | `v4-dev` | FPV cameras, IMU telemetry, instruction sharing, path optimization |
| v5.0 | `v5-dev` | Ground station, D2D/D2G protocols, comm dashboard, technical docs |

## Documentation

| Document | Description |
|----------|-------------|
| [Technical Proposal (PDF)](docs/TECHNICAL_PROPOSAL.pdf) | Full technical proposal with 15 SVG diagrams |
| [GPS-Denied Triangulation](docs/GPS_DENIED_TRIANGULATION_TECHNICAL_DOC.md) | Sensor data analysis, bearing math, message protocol |
| [Path Planning Architecture](docs/AGENTIC_PATH_PLANNING_AND_COMM_ARCHITECTURE.md) | PBRI algorithm, pincer search, system integration |

## Project Structure

```
├── index.html                  # Entry point
├── seadrone_v5.jsx             # Main simulation (3D + 2D + logic, ~1700 lines)
├── src/
│   └── main.jsx                # React mount point
├── docs/
│   ├── TECHNICAL_PROPOSAL.pdf  # Full technical proposal with diagrams
│   ├── TECHNICAL_PROPOSAL.html # Source for PDF generation
│   ├── GPS_DENIED_TRIANGULATION_TECHNICAL_DOC.md
│   └── AGENTIC_PATH_PLANNING_AND_COMM_ARCHITECTURE.md
├── vite.config.js              # Vite configuration
├── package.json                # Dependencies and scripts
└── .gitignore
```

## Tech Stack

- **React 18** — UI and state management
- **Three.js** — 3D rendering (WebGL)
- **Vite 6** — Build tool and dev server
- **YOLOv8-nano** — Object detection (real platform)
- **SAMURAI** — SAM2-based persistent tracking (real platform)
- **Pixhawk V6X** — Flight controller with MAVLink (real platform)
- **NVIDIA Jetson** — Onboard AI compute (real platform)
