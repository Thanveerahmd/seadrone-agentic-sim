# SeaDrone Multi-Agent Simulation

A real-time 3D simulation of autonomous sea drone multi-agent coordination for maritime target tracking and triangulation.

Built with **React**, **Three.js**, and **Vite**.

## Features

- **3D Ocean View** — Realistic ocean with animated waves, sky dome, dynamic shadows, wake particles, and detailed boat models with blinking antenna lights
- **2D Spatial Map** — SVG-based tactical overview with trails, bearing lines, comm links, and distance readouts
- **Resizable Panels** — Drag the divider between 3D and spatial map views to adjust layout
- **Multi-Phase Mission**:
  - **Deploy** — Both drones launch from base
  - **Search** — Drone A sweeps left, Drone B sweeps right
  - **Detect** — A locks target, begins pursuit as target attempts evasion
  - **Relay** — A sends comm signals to B (animated packets + beam), B continues searching until signal received
  - **Triangulate** — B accelerates toward target with realistic physics, both drones maintain 180° opposition
  - **Track** — Coordinated tracking with anti-collision geometry
- **Playback Controls** — Play/Pause, Reset, speed (0.5x–3x), timeline scrubbing, phase jump

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
| Jump to phase | Click a phase label on the timeline |
| Scrub timeline | Drag the slider |

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
