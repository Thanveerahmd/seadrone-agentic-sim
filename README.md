# SeaDrone Multi-Agent Simulation

A real-time 3D simulation of autonomous sea drone multi-agent coordination for maritime target tracking and triangulation.

Built with **React**, **Three.js**, and **Vite**.

## Features

### Scenarios (selectable)
- **Scenario 1: Detect & Triangulate** — A detects target, relays to B, both converge and triangulate
- **Scenario 2: Lose & Re-acquire** — A locks target, target escapes A's FOV, B re-acquires and relays back to A

### Communication Protocols (selectable)
- **D2D (Drone-to-Drone)** — Direct mesh link between drones. Cyan visuals, low latency (~12ms), fast packet animation
- **D2G (Drone-Ground-Drone)** — Relay via ground station orchestrator. Amber visuals, two-leg routing (Drone→GS→Drone), higher latency (~45ms)

### 3D Ocean View
- Realistic ocean with animated waves, sky dome, dynamic shadows, wake particles
- Detailed boat models with blinking antenna lights
- Ground station with rotating radar dish and beacon
- V-shaped FOV cones matching drone camera heading
- Protocol-aware comm beams (D2D direct vs D2G relay through GS)

### 2D Spatial Map
- SVG-based tactical overview with trails, bearing lines, FOV cones
- Ground station icon with comm routing visualization
- Lock-on crosshairs and "DUAL LOCK" indicator
- Expandable layout — toggle "Expand Map" for full-height view
- Draggable divider to resize 3D/map split

### Drone FPV Cameras
- First-person camera view from each drone's bow-mounted camera
- Clean feed — comm signals hidden from camera view
- Click to expand to 640x400 high-resolution feed
- HUD overlay: crosshair, heading compass, "TARGET LOCKED" indicator

### IMU Telemetry
- Per-drone panels: heading, speed, position, roll, pitch, bearing, distance to target
- Real-time updates derived from simulation tick data

### Communication Dashboard
- Active protocol indicator (D2D / D2G)
- Message routing path (A→B or A→GS→B)
- Link quality signal bars per connection
- Latency and bandwidth readouts
- Mini topology diagram

### Inter-Drone Comm Log
- Phase-aware instruction messages with protocol routing
- D2D: direct `[D2D] A→B` messages
- D2G: routed `A→GS` + `GS→B RELAY:` messages
- Color-coded by message type

### Triangulation Optimization
- Angular separation (vs 180° optimal)
- GDOP (Geometric Dilution of Precision) with quality rating
- Orbit radii for each drone
- Mini SVG diagram

### Multi-Phase Mission
- **Deploy** — Both drones launch from base
- **Search** — Drone A sweeps left, Drone B sweeps right
- **Detect/Lock** — Drone locks target with V-shaped FOV camera
- **Relay** — Comm signals sent via selected protocol (D2D or D2G)
- **Triangulate** — Both drones maintain 180° opposition
- **Track** — Coordinated tracking with anti-collision geometry

### Playback Controls
- Play/Pause, Reset, speed (0.5x–3x), timeline scrubbing, phase jump

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
# Build optimized static files
npm run build

# Preview the production build locally
npm run preview
```

The output will be in the `dist/` folder, ready to deploy to any static hosting.

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
| v5.0 | `v5-dev` | Ground station orchestrator, D2D/D2G protocol selector, comm dashboard |

## Project Structure

```
├── index.html          # Entry point
├── seadrone_v5.jsx     # Main simulation component (3D + 2D + logic)
├── src/
│   └── main.jsx        # React mount point
├── vite.config.js      # Vite configuration
├── package.json        # Dependencies and scripts
└── .gitignore
```

## Tech Stack

- **React 18** — UI and state management
- **Three.js** — 3D rendering (WebGL)
- **Vite 6** — Build tool and dev server
