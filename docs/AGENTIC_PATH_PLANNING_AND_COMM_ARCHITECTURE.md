# Agentic Path Planning & Communication Architecture
## GPS-Denied Multi-Drone Sea Target Triangulation

---

## 1. Problem Statement

Two autonomous sea drones must:
1. **Find** a target vessel using onboard cameras (YOLOv8-nano + SAMURAI tracking)
2. **Triangulate** its position using bearing-only measurements from two different positions
3. **Recover** when a drone loses visual contact — efficiently redirect the swarm
4. **Do all of this without GPS** — using dead reckoning from a shared launch reference

**Core constraints (from real sea trial data):**
- Detection: 1.3 FPS, 118ms inference, 68% detection rate, confidence 0.45–0.81
- Platform: 0.6–3.6 m/s, aggressive turning (up to 15°/frame)
- FOV: ~60° horizontal camera cone
- DR drift: 3.4% over 800m (validated)
- Comm latency: 12ms (D2D) or 45ms (D2G via ground station)

---

## 1.1 Real Hardware Platform

### Per-Drone Hardware Stack

| Component | Model | Role | IP/Interface |
|-----------|-------|------|-------------|
| **Compute** | NVIDIA Jetson (SeaDrone Lite) | AI inference, decision engine, MAVLink control | 192.168.1.x, SYSID=254 |
| **Camera** | RouteCam (RTSP) | Video stream to Jetson | 192.168.1.10 |
| **Flight Controller** | Pixhawk V6X | Autopilot, IMU, compass, GPS, motor control | 192.168.1.41, UART4 serial to Jetson |
| **Radio** | Radio Mesh (RF Bridge) | IP bridge between drone and ground station | 192.168.1.x subnet |
| **Network** | D-Link Switch | Onboard LAN connecting camera, Jetson, radio | 192.168.1.x |
| **ESC + Motors** | Standard marine ESC | PWM-driven from Pixhawk | Direct wire |

### Ground Station Hardware

| Component | Role | IP |
|-----------|------|-----|
| **Mac workstation** | GS orchestrator, monitoring, operator UI | 192.168.1.160 |
| **Mission Planner** | Telemetry monitoring (NO CONTROL authority) | SYSID=255 |
| **Radio Mesh A** | Ground-side RF bridge | Via D-Link Switch |
| **RC Controller** | FAILSAFE ONLY (2.4GHz RF) | Direct to Pixhawk |

### Control Authority Model

```
┌────────────────────────────────────────────────────────┐
│                 CONTROL PRIORITY                        │
│                                                        │
│  1. Jetson (SYSID=254) — FULL CONTROL                 │
│     └─ UART4 Serial → Pixhawk → MAVLink RC_OVERRIDE   │
│                                                        │
│  2. Mission Planner (SYSID=255) — NO CONTROL           │
│     └─ UDP via Mesh → Monitoring only                  │
│                                                        │
│  3. RC Controller — FAILSAFE ONLY                      │
│     └─ 2.4GHz RF → Direct to Pixhawk                  │
│        Only activates if Jetson link is lost            │
└────────────────────────────────────────────────────────┘
```

**Key insight:** The Jetson has FULL autonomous control via MAVLink RC_OVERRIDE. This means the agentic decision engine runs on the Jetson and directly commands steering + throttle. No human in the loop required for normal operation.

### Current Single-Drone Data Flow (Baseline)

```
RouteCam ──RTSP──→ Jetson ──UART4──→ Pixhawk ──PWM──→ Motors
  (RTSP)         ┌─────────────────┐   (V6X)
  Video          │ YOLOv8-nano     │
  Frames         │    ↓            │
                 │ Bounding Boxes  │
                 │    ↓            │
                 │ SAMURAI Tracker │
                 │ (IoU Lock)      │
                 │    ↓            │
                 │ Target Position │
                 │    ↓            │
                 │ PID Controller  │
                 │    ↓            │
                 │ Steering/       │
                 │ Throttle        │
                 │    ↓            │
                 │ MAVLink         │
                 │ RC_OVERRIDE     │
                 └─────────────────┘
                        │
          Operator ─────┘ (Click to Lock / Arm / Launch / Stop)
```

### Network Topology (Current Single-Drone)

```
GROUND (192.168.1.x)                    BOAT (192.168.1.x)
┌─────────────────┐                     ┌──────────────────────────┐
│                 │                     │                          │
│  Mac            │    RF Bridge        │  RouteCam                │
│  192.168.1.160  │                     │  192.168.1.10            │
│       │         │                     │       │                  │
│  D-Link Switch──┼──Radio Mesh A ═══ Radio Mesh B──D-Link Switch │
│       │         │                     │       │                  │
│  Mission        │                     │  Jetson ──UART4── Pixhawk│
│  Planner        │                     │  192.168.1.x    .41     │
│                 │                     │                          │
└─────────────────┘                     └──────────────────────────┘
```

### Multi-Drone Network Extension (Proposed)

For two-drone triangulation, extend the network:

```
GROUND STATION (192.168.1.x)
┌──────────────────────────────────┐
│  Mac 192.168.1.160               │
│  ├─ GS Orchestrator Agent        │
│  ├─ Mission Planner (monitor)    │
│  └─ Web Dashboard (sim viewer)   │
│       │                          │
│  D-Link Switch                   │
│       │                          │
│  Radio Mesh GS                   │
└───────┼──────────────────────────┘
        │ RF Bridge (shared mesh network)
    ┌───┴──────────────────┐
    │                      │
DRONE A (192.168.2.x)   DRONE B (192.168.3.x)
┌──────────────────┐     ┌──────────────────┐
│ Radio Mesh A     │     │ Radio Mesh B     │
│      │           │     │      │           │
│ D-Link Switch    │     │ D-Link Switch    │
│   │        │     │     │   │        │     │
│ RouteCam Jetson  │     │ RouteCam Jetson  │
│  .10     .20     │     │  .10     .20     │
│          │       │     │          │       │
│       Pixhawk    │     │       Pixhawk    │
│        .41       │     │        .41       │
└──────────────────┘     └──────────────────┘
          │                        │
          └────── D2D Direct ──────┘
          (Radio Mesh A ↔ Radio Mesh B
           on shared mesh frequency)
```

**Subnet separation:** Each drone gets its own subnet (192.168.2.x / 192.168.3.x) to prevent IP conflicts. The mesh radios handle routing between subnets.

**D2D capability:** The RF mesh radios can communicate directly between drones WITHOUT going through the ground station — enabling D2D protocol mode. The ground station is simply another node on the mesh.

---

## 2. Coordinate System — No GPS

### 2.1 Shared Local Frame

All positions expressed in **meters East/North from the ground station (launch point)**:

```
Origin: Ground Station position (0, 0)
X-axis: East (positive = right on map)
Y-axis: North (positive = up on map)

Drone A launch: (-3, 0) meters from GS
Drone B launch: (+3, 0) meters from GS
```

**No latitude/longitude anywhere in the protocol.** Every drone computes its own (x, y) position via dead reckoning:

```
At each timestep dt:
  x += speed * cos(heading) * dt
  y += speed * sin(heading) * dt
```

### 2.2 Position Quality Tag

Every position message includes a quality tag:

```
position_quality: {
  dr_distance_m: 200.0,        // total distance traveled on DR since last known fix
  estimated_error_m: 6.8,       // dr_distance * 0.034
  age_sec: 45.0                 // time since last absolute calibration
}
```

---

## 3. The Three Interception Approaches

### 3.1 Approach Comparison

| Approach | Speed | Accuracy | When to Use |
|----------|-------|----------|-------------|
| **Straight-Line Pursuit** | Slow — always chasing where target WAS | Poor — always behind | Never (baseline only) |
| **Proportional Navigation** | Fast — leads the target | Good — converges | Standard interception |
| **Predictive Intercept (Recommended)** | Fastest — races to where target WILL BE | Best — uses bearing rate history | When bearing data has 3+ samples |

### 3.2 Recommended: Predictive Bearing-Rate Intercept (PBRI)

This is the approach to implement. It combines bearing-rate extrapolation with energy-optimal path planning.

```
Algorithm: PREDICTIVE_BEARING_RATE_INTERCEPT

Inputs:
  - Tracking drone (A) sends N bearing reports over time window T
  - Intercepting drone (B) knows its own position (DR)
  - B knows A's position (from A's messages)

Step 1: ESTIMATE TARGET MOTION FROM BEARING HISTORY
  ────────────────────────────────────────────────
  From A's bearing reports at times [t1, t2, ... tN]:
    bearings = [θ1, θ2, ... θN]
    positions_A = [(x1,y1), (x2,y2), ... (xN,yN)]  // A's DR positions at each report

  Compute bearing rate: ω = dθ/dt  (deg/sec, from linear regression)

  Classify target motion:
    |ω| < 0.5°/s  → NEARLY_STATIONARY
    |ω| 0.5-3°/s  → SLOW_MOVER
    |ω| 3-8°/s    → MEDIUM_MOVER
    |ω| > 8°/s    → FAST_MOVER or TURNING

  Estimate target speed class from bearing rate + bbox size change:
    If bbox growing:  target approaching (reduce intercept aggressiveness)
    If bbox shrinking: target receding (increase intercept aggressiveness)
    If bbox constant:  target crossing (lead intercept)

Step 2: COMPUTE PREDICTED TARGET ZONE
  ─────────────────────────────────────
  Using A's latest bearing θ_now from position P_A:

  Bearing line: target is somewhere along ray from P_A at angle θ_now

  Range estimate (coarse): R_est = K / sqrt(bbox_area)
    where K is calibrated from known target class size

  Predicted target position at time T_future:
    target_bearing_at_T = θ_now + ω * (T_future - T_now)
    target_pos_estimate = P_A + R_est * (cos(target_bearing_at_T), sin(target_bearing_at_T))

  Uncertainty cone:
    angular_uncertainty = ±(5° + 2° * seconds_since_last_detection)
    range_uncertainty = ±(R_est * 0.3)  // 30% range error from bbox

  This defines an ELLIPTICAL PREDICTED ZONE the target will likely be in.

Step 3: COMPUTE OPTIMAL INTERCEPT POINT FOR DRONE B
  ──────────────────────────────────────────────────
  Requirements for the intercept point:
    a) B can reach it before the target leaves the predicted zone
    b) B's approach angle creates >60° angular separation from A
    c) B's FOV (60°) will cover the predicted zone on arrival
    d) B arrives with enough speed margin to track

  For each candidate intercept time T in [T+5s, T+10s, ... T+60s]:
    1. Predict target zone center at time T
    2. Compute: can B reach a good observation point by time T?
       - B_distance = speed_B * (T - T_now)
       - Observation point = target_predicted + offset for triangulation angle
    3. Score the candidate:

       score = w1 * triangulation_quality(angular_separation)
             + w2 * arrival_margin(time_slack)
             + w3 * fov_coverage(zone_size vs B's FOV at distance)
             - w4 * energy_cost(distance / battery_remaining)

  Select the T with highest score → compute waypoint for B.

Step 4: GENERATE INTERCEPT PATH
  ────────────────────────────────
  NOT a straight line. Use a curved approach that:
    a) Starts with B's current heading (smooth transition, no sudden turn)
    b) Gradually curves toward the intercept point
    c) Arrives with B FACING the predicted target zone (so FOV covers it)

  Path type: Dubins path or clothoid curve
    - Accounts for minimum turn radius of the sea drone
    - Ensures B doesn't overshoot and lose the target behind it

  Final approach heading:
    approach_heading = bearing_from_intercept_point_to_predicted_target
    This ensures B's 60° FOV cone is centered on where the target should be.
```

---

## 4. Target Loss Recovery Strategy

This is the critical scenario: Drone A had the target, then lost it. How do we efficiently recover?

### 4.1 Loss Classification

When a tracking drone loses the target, the **reason** determines the recovery strategy:

```
LOSS CLASSIFICATION:

1. TARGET_LEFT_FOV_SIDE
   - Last bearing rate was high (>5°/s)
   - Target moved out of the side of the camera frame
   - Recovery: Turn toward the last bearing rate direction

2. TARGET_LEFT_FOV_AHEAD
   - Target was ahead but moved beyond detection range
   - Bbox was shrinking before loss
   - Recovery: Speed up along last known bearing

3. TARGET_OCCLUDED
   - Lost_count is low, confidence dropped gradually
   - Target may be temporarily behind waves or in glare
   - Recovery: Maintain course for 5-10 seconds, likely to reappear

4. TARGET_EVADED
   - Bearing rate changed sign before loss (target turned)
   - Recovery: This is the hardest case — need swarm coordination
```

### 4.2 Recovery Strategy: Expanding Pincer Search

When the tracking drone (A) loses the target, the system executes a coordinated recovery:

```
Phase 1: IMMEDIATE (0-5 seconds after loss)
  ──────────────────────────────────────────
  Agent A:
    - Broadcast TARGET_LOST with last_bearing, bearing_rate, loss_reason
    - Execute IMMEDIATE TURN toward predicted escape direction:
      turn_direction = sign(last_bearing_rate)  // turn the way target was going
    - Begin expanding spiral search from last known bearing

  Agent B (or GS):
    - Receive TARGET_LOST
    - Compute PREDICTED ESCAPE ZONE from:
      * Last known bearing from A
      * Bearing rate at moment of loss (tells us which way target went)
      * Time since loss (target keeps moving)
      * Target speed class (from previous tracking data)

Phase 2: COORDINATED SEARCH (5-30 seconds)
  ──────────────────────────────────────────
  GS computes optimal search allocation:

  The predicted escape zone is a WEDGE-SHAPED area:
    origin: last known target position (bearing from A)
    direction: last bearing rate direction
    spread: widens with time (uncertainty grows)

  PINCER STRATEGY:
    Agent A: Search the LEFT half of the escape wedge
    Agent B: Search the RIGHT half of the escape wedge

    This ensures:
    a) The entire probable escape zone is covered
    b) Whichever drone re-acquires, the other is nearby for triangulation
    c) No wasted overlap

  Waypoints for each drone:
    A_search_waypoint = escape_zone_center + offset_LEFT * zone_radius
    B_search_waypoint = escape_zone_center + offset_RIGHT * zone_radius

    Both drones approach from OPPOSITE SIDES of the predicted zone.

Phase 3: RE-ACQUISITION (when either drone detects target)
  ─────────────────────────────────────────────────────────
  If Agent B finds the target:
    B sends TARGET_BEARING_REPORT to A (via GS or D2D)
    A navigates to triangulation position using PBRI algorithm

  If Agent A re-finds the target:
    A sends updated TARGET_BEARING_REPORT
    B adjusts course to maintain triangulation geometry

  In BOTH cases:
    The non-detecting drone uses the BEARING REPORT to compute
    an approach that creates optimal angular separation (>90°)
```

### 4.3 Escape Zone Prediction Model

```
At time T_loss, Agent A reports:
  - last_bearing: θ (degrees, absolute)
  - bearing_rate: ω (deg/sec)
  - estimated_range: R (meters, from bbox)
  - own_position: P_A (x, y in local frame)

Target estimated position at T_loss:
  T_pos = P_A + R * (cos(θ), sin(θ))

Target estimated velocity direction:
  If ω > 0: target moving clockwise (rightward) relative to A
  If ω < 0: target moving counterclockwise (leftward)

  target_heading_estimate = θ + 90° * sign(ω)  // perpendicular to bearing line
  // (if target is crossing A's view, it's moving perpendicular to the bearing)

Escape zone at time T_loss + Δt:
  center = T_pos + target_speed * Δt * (cos(target_heading_estimate), sin(target_heading_estimate))
  radius = target_speed * Δt * 0.5  // uncertainty grows with time

  The zone is a CIRCLE centered on the predicted position,
  growing at half the target's estimated speed.
```

---

## 5. Communication Protocol — GPS-Free

### 5.1 Message Types (Revised for No GPS)

All positions in **local frame meters (x_east, y_north) from ground station origin**.

#### Message 1: BEARING_REPORT (1 Hz when tracking)

**This is the single most important message in the entire system.**

```
BEARING_REPORT — 32 bytes compact format
─────────────────────────────────────────
Field                      Type      Bytes  Description
─────────────────────────────────────────
msg_type                   uint8     1      = 0x01
sequence                   uint16    2      rolling counter
source_agent               uint8     1      0=A, 1=B
timestamp_ms               uint32    4      ms since mission start
source_x_m                 float32   4      own position, meters east from GS
source_y_m                 float32   4      own position, meters north from GS
source_heading_deg         float16   2      compass heading
source_speed_mps           float16   2      ground speed
target_bearing_abs_deg     float16   2      absolute bearing to target
target_bearing_rate_dps    float16   2      bearing rate, deg/sec
detection_confidence       uint8     1      0-100 mapped from 0.0-1.0
bbox_area_px               uint16    2      bounding box area (for range estimate)
bbox_cx_px                 uint16    2      bbox center x in frame
dr_quality                 uint8     1      0-100 (100 = fresh calibration, 0 = max drift)
track_lost_count           uint8     1      0 = solid lock, >0 = degraded
padding                    uint8     1
─────────────────────────────────────────
TOTAL: 32 bytes
```

**Why 32 bytes matters:** At low-bandwidth D2D links (e.g., 900 MHz LoRa at 300 bps), a 32-byte packet takes ~0.85 seconds. At 2.4 GHz (9600 bps), it takes ~27ms. This packet can be sent at 1 Hz on any practical radio link.

#### Message 2: TARGET_LOST (event-triggered)

```
TARGET_LOST — 24 bytes
─────────────────────────────
msg_type                   uint8     1      = 0x02
source_agent               uint8     1
timestamp_ms               uint32    4
last_bearing_deg           float16   2
bearing_rate_at_loss_dps   float16   2
estimated_range_m          float16   2
loss_reason                uint8     1      0=LEFT_SIDE, 1=LEFT_AHEAD, 2=OCCLUDED, 3=EVADED
source_x_m                 float32   4
source_y_m                 float32   4
source_heading_deg         float16   2
padding                    uint8     1
─────────────────────────────
TOTAL: 24 bytes
```

#### Message 3: INTERCEPT_COMMAND (GS → Drone, or Drone → Drone)

```
INTERCEPT_COMMAND — 28 bytes
──────────────────────────────
msg_type                   uint8     1      = 0x03
target_agent               uint8     1
timestamp_ms               uint32    4
waypoint_x_m               float32   4      intercept point, meters east
waypoint_y_m               float32   4      intercept point, meters north
approach_heading_deg       float16   2      heading to arrive with
approach_speed_mps         float16   2      recommended speed
predicted_target_x_m       float32   4      where target will be
predicted_target_y_m       float32   4      where target will be
urgency                    uint8     1      0=LOW, 1=MEDIUM, 2=HIGH, 3=CRITICAL
padding                    uint8     1
──────────────────────────────
TOTAL: 28 bytes
```

#### Message 4: TRIANGULATION_FIX (GS → Both, when both have bearings)

```
TRIANGULATION_FIX — 20 bytes
────────────────────────────
msg_type                   uint8     1      = 0x04
timestamp_ms               uint32    4
target_x_m                 float32   4      triangulated target position
target_y_m                 float32   4      triangulated target position
accuracy_m                 float16   2      estimated circle of error
angular_separation_deg     uint8     1      geometry quality
gdop                       uint8     1      *10 (e.g., 12 = GDOP 1.2)
target_heading_deg         float16   2      estimated from bearing pair history
padding                    uint8     1
────────────────────────────
TOTAL: 20 bytes
```

#### Message 5: SEARCH_ASSIGN (GS → Drone, after target loss)

```
SEARCH_ASSIGN — 20 bytes
──────────────────────────
msg_type                   uint8     1      = 0x05
target_agent               uint8     1
timestamp_ms               uint32    4
search_center_x_m          float32   4      center of search zone
search_center_y_m          float32   4      center of search zone
search_radius_m            float16   2      zone radius
search_heading_deg         float16   2      initial search heading
pattern                    uint8     1      0=SPIRAL, 1=SECTOR_SWEEP, 2=PINCER_LEFT, 3=PINCER_RIGHT
padding                    uint8     1
──────────────────────────
TOTAL: 20 bytes
```

### 5.2 Communication Flow Diagram

```
                    MISSION TIMELINE
    ════════════════════════════════════════════

    SEARCH PHASE
    ┌─────┐                          ┌─────┐
    │  A  │──AGENT_STATUS (2Hz)─────→│ GS  │←──AGENT_STATUS (2Hz)──│  B  │
    │     │  "searching, no contact" │     │  "searching, no contact│     │
    └─────┘                          └─────┘                       └─────┘

    A DETECTS TARGET
    ┌─────┐                          ┌─────┐                       ┌─────┐
    │  A  │──BEARING_REPORT (1Hz)───→│ GS  │──INTERCEPT_CMD──────→│  B  │
    │     │  bearing=161°, rate=-2°/s│     │  "go to (45, 80),     │     │
    │     │  conf=0.78, range~50m    │     │   approach from 245°" │     │
    └─────┘                          └─────┘                       └─────┘

    A LOSES TARGET
    ┌─────┐                          ┌─────┐                       ┌─────┐
    │  A  │──TARGET_LOST────────────→│ GS  │──SEARCH_ASSIGN──────→│  B  │
    │     │  last_brg=175°, rate=-5° │     │  "PINCER_RIGHT,       │     │
    │     │  loss_reason=LEFT_SIDE   │     │   search (30,65) r=20"│     │
    │     │                          │     │                       │     │
    │     │←─SEARCH_ASSIGN───────────│     │  "PINCER_LEFT,        │     │
    │     │  "search (10,65) r=20"   │     │   search (30,65) r=20"│     │
    └─────┘                          └─────┘                       └─────┘

    B RE-ACQUIRES TARGET
    ┌─────┐                          ┌─────┐                       ┌─────┐
    │  A  │←─INTERCEPT_CMD───────────│ GS  │←──BEARING_REPORT─────│  B  │
    │     │  "go to (35, 70),        │     │  bearing=72°, rate=1° │     │
    │     │   approach from 320°"    │     │  conf=0.71            │     │
    └─────┘                          └─────┘                       └─────┘

    BOTH HAVE BEARINGS → TRIANGULATION
    ┌─────┐                          ┌─────┐                       ┌─────┐
    │  A  │──BEARING_REPORT─────────→│ GS  │←──BEARING_REPORT─────│  B  │
    │     │                          │     │                       │     │
    │     │←─TRIANGULATION_FIX───────│     │──TRIANGULATION_FIX──→│     │
    │     │  target at (32, 68)      │     │  target at (32, 68)   │     │
    │     │  accuracy: 5.2m          │     │  maintain 180° sep    │     │
    │     │  orbit at 22m            │     │  orbit at 22m         │     │
    └─────┘                          └─────┘                       └─────┘
```

### 5.3 D2D vs D2G Protocol Differences

When operating in **D2D mode** (no ground station relay):

```
D2D MODE — Drone A acts as temporary orchestrator:
───────────────────────────────────────────────────

The DETECTING drone becomes the coordinator:

  Agent A (has target):
    1. Computes intercept point for B (same algorithm GS would use)
    2. Sends INTERCEPT_CMD directly to B
    3. Continues sending BEARING_REPORT to B at 1 Hz

  Agent B (intercepting):
    1. Receives INTERCEPT_CMD from A
    2. Navigates to intercept point
    3. When B also detects: sends BEARING_REPORT to A
    4. A computes triangulation locally

  Advantages:
    - Lower latency (12ms vs 45ms)
    - No single point of failure (GS goes down = still works)
    - Works beyond GS range

  Disadvantages:
    - Detecting drone has computational burden (tracking + computing + transmitting)
    - No central mission oversight
    - No data logging at GS for post-mission analysis

When target is LOST in D2D mode:
  - A broadcasts TARGET_LOST to B
  - A and B each compute their own search sector:
    A takes the LEFT half of escape zone
    B takes the RIGHT half
  - Simple rule: lower-ID drone takes LEFT, higher-ID takes RIGHT
  - No GS coordination needed
```

---

## 6. Integrated System Architecture

### 6.1 Onboard Processing Pipeline (Each Drone — Runs on Jetson)

```
RouteCam (RTSP, 960x540, ~1.3 FPS)
       │
       ▼
┌──────────────────┐
│  YOLOv8-nano     │  118ms inference on Jetson GPU
│  Detection       │  → bbox + confidence + class label
└──────┬───────────┘
       │ if new detection
       ▼
┌──────────────────┐
│  SAMURAI Tracker  │  SAM2-based persistent tracking
│  (IoU Lock)      │  → track_id, velocity_px/s, lost_count
│                  │  Operator can "Click to Lock" initial target
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│  Bearing          │  bbox centroid → camera angle → absolute bearing
│  Estimator       │  + roll/pitch compensation from Pixhawk IMU
│                  │  (IMU data via MAVLink over UART4)
└──────┬───────────┘
       │
       ▼
┌──────────────────┐
│  Dead Reckoning  │  Pixhawk compass heading + speed → (x, y)
│  Navigator       │  Local frame from GS launch point
│                  │  Updated every MAVLink heartbeat (~1 Hz)
└──────┬───────────┘
       │
       ▼
┌──────────────────────────────────────────────────────┐
│  AGENTIC DECISION ENGINE (new for multi-drone)       │
│                                                      │
│  Current (single-drone): PID Controller → RC_OVERRIDE│
│                                                      │
│  Proposed (multi-drone): State Machine that decides: │
│    SEARCH → TRACK → INTERCEPT → LOST_RECOVERY       │
│    Outputs:                                          │
│      ├─→ PID targets (heading + throttle)            │
│      ├─→ MAVLink RC_OVERRIDE → UART4 → Pixhawk      │
│      ├─→ COMM TX via Radio Mesh (UDP packets)        │
│      └─→ COMM RX from GS or other drone             │
└──────────────────────────────────────────────────────┘
       │
       ▼
┌──────────────────┐
│  MAVLink         │  Steering/Throttle PWM values
│  RC_OVERRIDE     │  → UART4 → Pixhawk V6X → ESC → Motors
└──────────────────┘
```

### 6.1.1 What Changes for Multi-Drone (Current → Proposed)

The current single-drone system has a simple pipeline: detect → track → PID follow → RC_OVERRIDE. For multi-drone GPS-denied triangulation, the **Agentic Decision Engine** replaces the simple PID controller as the decision-maker:

```
CURRENT (Single-Drone):
  SAMURAI Target Position → PID Controller → Steering/Throttle → RC_OVERRIDE

PROPOSED (Multi-Drone Agent):
  SAMURAI Target Position ─┐
  Bearing Estimate ────────┤
  DR Position ─────────────┼──→ DECISION ENGINE ──→ PID Targets ──→ RC_OVERRIDE
  COMM RX (from GS/B) ────┤      │
  Battery/Sensor State ────┘      │
                                  ├──→ COMM TX (BEARING_REPORT, TARGET_LOST, etc.)
                                  └──→ State transitions (SEARCH↔TRACK↔INTERCEPT)
```

**The PID controller is preserved** — it still converts a target heading + speed into steering/throttle PWM. But instead of always pointing at the visual target, the Decision Engine can command the PID to:
- Follow a search pattern (no target in FOV)
- Navigate to an intercept waypoint (target reported by other drone)
- Execute a pincer search (target lost, coordinated recovery)
- Orbit at standoff distance (triangulation maintenance)

### 6.1.2 MAVLink Integration

The Jetson communicates with Pixhawk via MAVLink over UART4:

```
JETSON → PIXHAWK (control):
  MAV_CMD: RC_OVERRIDE (ch1=steering, ch3=throttle)
  Rate: 10-20 Hz (PID output rate)

PIXHAWK → JETSON (telemetry):
  ATTITUDE: roll, pitch, yaw (IMU data for bearing correction)
  VFR_HUD: heading, groundspeed (for dead reckoning)
  GPS_RAW_INT: lat, lon, satellites (when available, for DR calibration)
  SYS_STATUS: battery voltage, remaining
  Rate: 1-10 Hz depending on message type

JETSON → MESH RADIO (inter-agent, UDP):
  BEARING_REPORT: 32 bytes at 1 Hz
  TARGET_LOST: 24 bytes on event
  AGENT_STATUS: 16 bytes at 0.5 Hz
  Destination: GS (192.168.1.160) or other drone (192.168.3.x)
```

### 6.2 Decision Engine State Machine

```
                    ┌──────────┐
         ┌─────────│  DEPLOY   │─────────┐
         │         └──────────┘          │
         ▼                               ▼
    ┌──────────┐                   ┌──────────┐
    │ SEARCH_A │                   │ SEARCH_B │
    │ (sweep)  │                   │ (sweep)  │
    └────┬─────┘                   └────┬─────┘
         │ target in FOV                │ recv INTERCEPT_CMD
         ▼                              ▼
    ┌──────────┐                   ┌────────────┐
    │ TRACK    │───BEARING_RPT───→│ NAVIGATE    │
    │ (follow) │   to B/GS        │ (intercept) │
    └────┬─────┘                   └─────┬──────┘
         │ target lost                    │ target in FOV
         ▼                               ▼
    ┌──────────┐                   ┌──────────┐
    │ LOST     │──TARGET_LOST────→│ TRACK_B   │
    │ RECOVERY │   to B/GS        │ (B leads) │
    └────┬─────┘                   └─────┬─────┘
         │ recv SEARCH_ASSIGN            │ A also has bearing
         ▼                               ▼
    ┌──────────────┐              ┌───────────────┐
    │ PINCER_SEARCH│              │ TRIANGULATE    │
    │ (coordinated)│              │ (dual bearing) │
    └──────┬───────┘              └───────┬───────┘
           │ re-acquire                   │ converged
           ▼                              ▼
    ┌──────────────┐              ┌───────────────┐
    │  TRIANGULATE │              │ ORBIT_TRACK    │
    │              │              │ (maintain sep) │
    └──────────────┘              └───────────────┘
```

### 6.3 Timing Budget

```
Total latency from target movement to drone response:

Camera exposure:              ~10ms
YOLOv8 inference:            ~118ms
SAMURAI tracking:             ~15ms
Bearing computation:           ~2ms
DR position update:            ~1ms
Message construction:          ~1ms
────────────────────────────────────
Subtotal (onboard):          ~147ms

Communication:
  D2D:                        ~12ms
  D2G (A→GS→B):              ~45ms

GS computation:
  Intercept planning:         ~20ms
  Triangulation:               ~5ms
────────────────────────────────────
Total D2D pipeline:          ~180ms  (5.5 Hz theoretical update rate)
Total D2G pipeline:          ~215ms  (4.7 Hz theoretical update rate)

At target speed 2 m/s:
  D2D: target moves 0.36m between detection and drone response
  D2G: target moves 0.43m between detection and drone response

This is well within acceptable limits for maritime tracking.
```

---

## 7. Path Optimization Techniques

### 7.1 Intercept Geometry

The intercepting drone should NOT fly straight at the target. It should fly to where the target WILL BE, arriving at an angle that maximizes triangulation quality.

```
OPTIMAL INTERCEPT GEOMETRY:

                    Target predicted path
                    ─ ─ ─ ─ ─ ─ ─►
                         ╱
                        ╱ ← B approaches from here
                       ╱     (90-180° from A's bearing)
               Target ●
                     ╲
                      ╲ ← A's bearing line
                       ╲
                        A

Angular separation goal: 90° ≤ θ ≤ 180°
At θ = 90°:  GDOP = 1.41 (good)
At θ = 180°: GDOP = 1.00 (optimal)
```

### 7.2 Energy-Optimal Speed Profile

Don't run at max speed the whole way — use a trapezoidal speed profile:

```
Speed
  ▲
  │    ┌──────────────┐
  │   ╱                ╲
  │  ╱                  ╲
  │ ╱                    ╲  ← Slow down for final approach
  │╱                      ╲   (need to be stable for camera)
  └────────────────────────── Time

  Phase 1: Accelerate (20% of trip)
  Phase 2: Cruise at optimal speed (60% of trip)
  Phase 3: Decelerate to tracking speed (20% of trip)

  Cruise speed selection:
    - Short intercept (<50m): 2.5 m/s (accuracy > speed)
    - Medium intercept (50-100m): 3.0 m/s (balanced)
    - Long intercept (>100m): 3.5 m/s (speed priority)
    - Critical (target escaping): max speed 3.6 m/s
```

### 7.3 Arrival Heading Optimization

The drone must arrive FACING the predicted target zone so its 60° FOV covers the area:

```
ARRIVAL HEADING COMPUTATION:

  predicted_target_at_arrival = target_pos + target_vel * time_to_arrive

  arrival_heading = atan2(
    predicted_target_at_arrival.y - intercept_point.y,
    predicted_target_at_arrival.x - intercept_point.x
  )

  FOV coverage check:
    zone_angular_width = 2 * atan(zone_radius / distance_to_zone)

    if zone_angular_width < FOV_ANGLE (60°):
      ✓ FOV covers the predicted zone — proceed
    else:
      Approach closer before committing to tracking mode
```

---

## 8. Integration Summary

### 8.1 What Each Component Does

```
┌─────────────────────────────────────────────────────────────┐
│                    GROUND STATION                            │
│                                                             │
│  Receives: BEARING_REPORT, TARGET_LOST, AGENT_STATUS        │
│  Computes: Intercept points, triangulation, search zones    │
│  Sends:    INTERCEPT_CMD, SEARCH_ASSIGN, TRIANGULATION_FIX  │
│                                                             │
│  Key algorithms:                                            │
│    - Predictive Bearing-Rate Intercept (PBRI)               │
│    - Bearing intersection (triangulation)                    │
│    - Escape zone prediction                                 │
│    - Pincer search allocation                               │
└──────────────────────────┬──────────────────────────────────┘
                           │ D2G protocol (amber, 45ms)
                           │ or
                           │ D2D protocol (cyan, 12ms) — GS not in loop
            ┌──────────────┴──────────────┐
            ▼                             ▼
┌───────────────────┐         ┌───────────────────┐
│    DRONE A         │         │    DRONE B         │
│                   │         │                   │
│  Camera → YOLO →  │         │  Camera → YOLO →  │
│  SAMURAI tracker  │         │  SAMURAI tracker  │
│  Bearing estimator│         │  Bearing estimator│
│  Dead reckoning   │         │  Dead reckoning   │
│  Decision engine  │         │  Decision engine  │
│  Autopilot        │         │  Autopilot        │
│                   │◄───────►│                   │
│                   │  D2D    │                   │
└───────────────────┘ direct  └───────────────────┘
```

### 8.2 Data That Must Be Communicated

| Data | From | To | When | Why |
|------|------|-----|------|-----|
| Own DR position (x, y) | Each drone | GS/other | 1 Hz always | Bearing line origin |
| Target bearing (deg) | Detecting drone | GS/other | 1 Hz when tracking | Triangulation input |
| Bearing rate (deg/s) | Detecting drone | GS/other | 1 Hz when tracking | Target motion prediction |
| Detection confidence | Detecting drone | GS/other | 1 Hz when tracking | Bearing weighting |
| Bbox area (px) | Detecting drone | GS/other | 1 Hz when tracking | Range estimation |
| Loss event + reason | Losing drone | GS/other | Immediate | Trigger recovery |
| Bearing at loss + rate | Losing drone | GS/other | Immediate | Escape prediction |
| Intercept waypoint | GS (or detecting drone) | Intercepting drone | On detection, updated 0.5 Hz | Navigation target |
| Search sector assignment | GS (or by convention) | Both drones | On target loss | Coordinated recovery |
| Triangulated position | GS (or detecting drone) | Both drones | When dual bearings available | Confirmed target position |

### 8.3 What Is NOT Communicated (Kept Local)

| Data | Stays On | Reason |
|------|----------|--------|
| Raw camera frames | Each drone | Too much bandwidth |
| YOLO inference results (full) | Each drone | Only bearing matters |
| IMU raw data | Each drone | Only heading/speed needed |
| Autopilot commands | Each drone | Local control loop |
| SAMURAI track state | Each drone | Only lock/lost status matters |
| Servo PWM values | Each drone | Low-level actuator detail |
| Full bbox coordinates | Each drone | Only center + area communicated |

---

## 9. Performance Targets

| Metric | Target | Basis |
|--------|--------|-------|
| Time from detection to B intercept | < 60 seconds | At 3 m/s, B can cover 180m |
| Triangulation accuracy | < 10m CEP | GDOP 1.4 at 90° sep, 50m range, 5° bearing error |
| Target loss recovery time | < 30 seconds | Pincer search covers 40m radius zone |
| Communication bandwidth | < 64 bytes/sec per drone | 32-byte packet at 1 Hz + overhead |
| DR position accuracy | < 10m after 5 min | 3.4% drift validated |
| Bearing accuracy | < 5° | Camera 960px, 60° FOV, SAMURAI centroid |
| System latency (detect → response) | < 250ms | Pipeline analysis Section 6.3 |
