# GPS-Denied Multi-Agentic Sea Drone Triangulation — Technical Documentation

## 1. Executive Summary

This document defines the inter-agent communication protocol and agentic path planning strategy for GPS-denied multi-drone maritime target triangulation. Two autonomous sea drones (Agent A, Agent B) must collaboratively locate, track, and triangulate a target vessel **without relying on GPS for target positioning** — using only onboard sensors (camera, IMU, compass) and inter-agent communication via a ground station orchestrator or direct mesh link.

---

## 2. Real Sensor Data Analysis

Based on `sea_drone_data_v1.json` — 394 frames from a real sea drone tracking session:

### 2.1 Platform Capabilities (Measured)

| Parameter | Value | Source |
|-----------|-------|--------|
| Frame rate | ~1.3 FPS (394 frames / ~300s) | Timestamps |
| Detection model | YOLOv8 | `processing_info.model_name` |
| Inference time | ~118ms per frame | `processing_info.inference_time_ms` |
| Camera resolution | 960 x 540 | `frame_resolution` |
| Ground speed | 0.6 — 3.6 m/s | `velocity.ground_speed_mps` |
| IMU data | Roll, Pitch, Yaw (deg) | `telemetry_info.imu` |
| Compass heading | 0 — 360 deg | `telemetry_info.compass` |
| GPS (drone only) | Lat/Lon, altitude, fix type, satellites | `telemetry_info.gps` |
| Battery | 64 — 67% over 5 min session | `telemetry_info.battery` |
| Detection confidence | 0.45 — 0.81 | `detected_object.confidence` |
| Tracking velocity | -274 to +90 px/s (X), -44 to +61 px/s (Y) | `tracking.velocity_px_per_sec` |
| Max track lost count | 349 frames | `tracking.lost_count` |
| Steering | PWM 1580 (fixed — manual mode) | `servo_outputs.channels` |

### 2.2 Tracking Model

Target tracking uses **SAMURAI** (Segment Anything Model for Unified Robust Identification and Tracking) — a SAM2-based visual tracker that maintains target identity across frames even with partial occlusion, scale changes, and camera motion. SAMURAI provides:
- Persistent track ID across detection gaps
- Sub-pixel bounding box estimation
- Motion-compensated tracking during drone maneuvers

Detection is performed by **YOLOv8** for initial target acquisition, then handed off to SAMURAI for continuous tracking.

### 2.3 Key Observations

1. **GPS is available for the DRONE, not the TARGET.** The drone knows where it is; it does NOT know where the target is in world coordinates.
2. **Target position is only known as a bounding box** in the camera frame (pixel coordinates).
3. **Detection is intermittent** — target is detected in 269/394 frames with multiple lock/lost transitions.
4. **The drone covered 796m of path over ~128m straight-line distance** — significant maneuvering.
5. **Heading changes up to 14.8°/frame** — the platform can turn aggressively.
6. **Tracking velocity in pixels** provides target motion direction within the camera frame.

---

## 3. Dead Reckoning Feasibility — Field Validation

### 3.1 Test Results (Single-Drone Sea Trial)

A GPS-free feasibility test was conducted using real sea drone data, validating two critical capabilities:

**Test 1: Dead Reckoning Self-Position (Compass + Speed Only)**

| Metric | Value |
|--------|-------|
| Method | Compass heading + ground speed integration |
| Path length | ~800m |
| Duration | 5 minutes |
| Cumulative drift | **3.4%** (27m over 800m) |
| GPS comparison | Dead reckoning path closely matches GPS ground truth |

**Implication:** A drone can reliably estimate its own position using only compass + speed sensor for at least 5 minutes of operation. This means **even if GPS is denied/jammed**, the drone can still provide a reasonably accurate "bearing line origin" for triangulation — with known and bounded error.

**Test 2: Target Position Estimation (Camera-Only, No Target GPS)**

| Metric | Value |
|--------|-------|
| Method | YOLOv8/SAMURAI bbox centroid + drone heading → bearing → range estimate |
| Target bearing accuracy | **Directionally accurate** — target trajectory shape matches reality |
| Target range accuracy | **Approximate** — requires known target physical size for exact scale |
| Raw estimates | Noisy (scattered dots) but correct trend |
| Smoothed path | Clean trajectory closely matching target's actual movement |

**Implication:** From a single drone's camera, we can reconstruct the target's approximate trajectory in world coordinates. The **direction** is reliable; the **distance** is the primary uncertainty. This is exactly why **two-drone triangulation** is needed — two bearings from different positions resolve the range ambiguity.

### 3.2 Impact on Multi-Agent Triangulation

These field results fundamentally validate the approach:

```
Single Drone Can Provide:
  ✓ Own position via dead reckoning (3.4% drift over 800m)
  ✓ Target BEARING (direction) — accurate
  ✗ Target RANGE (distance) — approximate, needs calibration

Two Drones Together Provide:
  ✓ Own positions (each via DR, 3.4% drift each)
  ✓ Two independent target BEARINGS from different positions
  ✓ Target POSITION via bearing intersection (triangulation)
  ✓ Target RANGE — resolved geometrically, no calibration needed
```

### 3.3 Dead Reckoning Error Model for Triangulation

Since dead reckoning drifts at ~3.4% of distance traveled, bearing line origin uncertainty grows over time:

```
DR position error after distance D:
  σ_position = 0.034 * D

For a drone that has traveled 200m since last GPS fix:
  σ_position = 6.8m

This position error translates to bearing error:
  σ_bearing ≈ arctan(σ_position / range_to_target)

At 50m range to target, 6.8m position error → ~7.7° bearing error
At 100m range to target, 6.8m position error → ~3.9° bearing error
```

**Mitigation strategies:**
1. **Periodic GPS recalibration** — when GPS is available, reset DR accumulation
2. **Shorter DR windows** — relay GPS coordinates before denial, then DR for the denial period
3. **DR cross-validation** — if both drones have GPS intermittently, compare DR predictions with GPS fixes to calibrate drift rate
4. **Weighted triangulation** — weight each drone's bearing by inverse of its DR age (fresher DR = higher weight)

### 3.4 Range Estimation from Bbox (Single-Drone Fallback)

The right-side plot shows that target distance estimation from bbox size is approximate but usable as a fallback. The relationship:

```
estimated_range = (K * known_target_width) / bbox_width_pixels

Where:
  K = focal_length_pixels (camera calibration constant)
  known_target_width = estimated real-world width of target class (e.g., boat = 3-5m)
  bbox_width_pixels = detected bounding box width
```

From the real data:
- Bbox width ranges from 17px (far) to 136px (close)
- With camera FOV ~60° and 960px width: focal_length ≈ 831px
- A 4m boat at 50m range would appear as: 831 * 4 / 50 = 66px (matches observed data)

**This provides a coarse range estimate** (±30-50% accuracy) usable for initial intercept planning before triangulation is established.

---

## 4. The GPS-Denied Problem

### 3.1 What We Know vs. What We Don't

| Known (per drone) | Unknown |
|---|---|
| Own GPS position (lat, lon) | Target GPS position |
| Own heading (compass + IMU yaw) | Target heading |
| Own speed (ground speed m/s) | Target speed |
| Target bearing in camera frame (bbox center) | Target range/distance |
| Target pixel velocity (px/s) | Target world velocity |
| Target detection confidence | Target identity confirmation |
| Own roll, pitch (wave state) | Target size (real-world) |

### 3.2 Core Insight

A single drone can determine the **bearing** to the target (angle from drone heading) but **NOT the range** (distance). With two drones observing the same target from different positions, **triangulation** resolves the target's position using the intersection of two bearing lines.

---

## 5. Bearing Estimation from Camera Data

### 4.1 Camera-to-Bearing Conversion

The target's bearing relative to the drone is derived from its bounding box center position in the camera frame:

```
Target bbox center: (cx, cy) in pixels
Camera FOV horizontal: FOV_H (measured or calibrated, e.g., 60°)
Frame width: W = 960 pixels

Pixel offset from center: dx = cx - (W / 2)
Bearing offset (radians): bearing_offset = (dx / (W / 2)) * (FOV_H / 2)
Absolute bearing: target_bearing = drone_heading + bearing_offset
```

**Example from real data:**
- Frame 36: bbox center x = 626, frame width = 960
- dx = 626 - 480 = +146 pixels (target is right of center)
- With 60° FOV: bearing_offset = (146/480) * 30° = +9.1°
- If drone heading = 152°: target_bearing = 161.1°

### 4.2 Bearing Confidence

Bearing accuracy depends on:
- **Detection confidence** (0.45–0.81) — low confidence = noisy bbox = noisy bearing
- **Bbox width** — small bbox (far target) = less precise centroid
- **Roll/pitch** — wave-induced camera tilt distorts the bearing

**Compensation formula:**
```
corrected_bearing = raw_bearing - roll_component * sin(pitch)
bearing_confidence = detection_confidence * (1 - abs(roll) / 30°)
```

### 4.3 Bearing Rate (Target Motion)

The tracking velocity in pixels (`velocity_px_per_sec.x`) indicates how fast the target is moving across the camera frame:

```
bearing_rate = (velocity_px_per_sec.x / (W / 2)) * (FOV_H / 2)  // deg/sec
```

This tells us whether the target is moving left or right relative to the drone — critical for **predicting where the target will be** when the second drone arrives.

---

## 6. Inter-Agent Communication Protocol

### 10.1 Message Types

#### 6.1.1 TARGET_BEARING_REPORT (Drone → GS or Drone → Drone)

The **primary message** for triangulation. Sent when a drone has a locked target.

```json
{
  "msg_type": "TARGET_BEARING_REPORT",
  "source_agent": "DRONE_A",
  "timestamp": "2026-03-24T11:48:15.123Z",
  "sequence_id": 42,

  "source_position": {
    "latitude": 1.2231877,
    "longitude": 103.8496582,
    "position_accuracy_m": 2.5,
    "source": "GPS|DR|GPS+DR",
    "dr_drift_percent": 3.4,
    "dr_distance_since_fix_m": 200.0,
    "last_gps_fix_age_sec": 45.0
  },

  "source_heading_deg": 152.1,
  "source_speed_mps": 3.2,

  "target_bearing": {
    "absolute_deg": 161.2,
    "relative_deg": 9.1,
    "confidence": 0.78,
    "method": "CAMERA_BBOX_CENTROID"
  },

  "target_motion": {
    "bearing_rate_deg_per_sec": -2.3,
    "bbox_velocity_px_per_sec": { "x": 53.8, "y": -3.4 },
    "motion_direction": "LEFT_RELATIVE",
    "estimated_target_speed_class": "MEDIUM"
  },

  "detection_quality": {
    "detector": "YOLOv8",
    "tracker": "SAMURAI",
    "confidence": 0.78,
    "bbox_area_px": 1254,
    "bbox_aspect_ratio": 3.47,
    "track_id": "track_224",
    "track_age_frames": 15,
    "lost_count": 0
  },

  "environmental": {
    "roll_deg": 5.6,
    "pitch_deg": 7.1,
    "sea_state_estimate": "MODERATE",
    "visibility": "GOOD"
  }
}
```

**Why each field matters:**

| Field | Purpose for Receiving Agent |
|-------|----------------------------|
| `source_position` | Bearing line origin — GPS, dead reckoning (DR), or fused. Field-validated: DR drifts only 3.4% over 800m |
| `source_position.dr_distance_since_fix_m` | How far drone has traveled on DR alone — higher = more position uncertainty |
| `source_position.last_gps_fix_age_sec` | Staleness of last GPS calibration — used to weight bearing confidence |
| `source_heading_deg` | Validates bearing calculation |
| `target_bearing.absolute_deg` | **THE critical value** — bearing line from source to target |
| `target_bearing.confidence` | Weight in triangulation calculation |
| `target_motion.bearing_rate_deg_per_sec` | Predict where target will be when agent B arrives |
| `target_motion.estimated_target_speed_class` | Path planning: how fast to intercept |
| `detection_quality.confidence` | Trust level — low confidence = wider uncertainty cone |
| `detection_quality.lost_count` | If > 0, bearing may be stale |
| `environmental.roll_deg` | Bearing error estimation from wave motion |

#### 6.1.2 NAVIGATE_TO_INTERCEPT (GS → Drone)

Ground station computes the intercept waypoint and sends navigation command:

```json
{
  "msg_type": "NAVIGATE_TO_INTERCEPT",
  "target_agent": "DRONE_B",
  "timestamp": "2026-03-24T11:48:15.500Z",

  "waypoint": {
    "latitude": 1.2230500,
    "longitude": 103.8492000,
    "approach_bearing_deg": 245.0,
    "approach_speed_mps": 3.0,
    "urgency": "HIGH"
  },

  "predicted_target": {
    "estimated_position": { "lat": 1.2230100, "lon": 103.8491500 },
    "estimated_heading_deg": 210.0,
    "estimated_speed_mps": 2.0,
    "prediction_age_sec": 0.5,
    "confidence": 0.65,
    "method": "BEARING_RATE_EXTRAPOLATION"
  },

  "triangulation_geometry": {
    "required_approach_angle_deg": 245.0,
    "optimal_standoff_distance_m": 22.0,
    "min_angular_separation_deg": 60.0,
    "current_angular_separation_deg": 0.0
  },

  "mission_context": {
    "phase": "INTERCEPT",
    "agent_a_status": "TRACKING",
    "time_to_intercept_est_sec": 45.0
  }
}
```

**Key field explanations:**

| Field | Why It's Needed |
|-------|----------------|
| `waypoint` | Where drone B should go — computed from target prediction |
| `approach_bearing_deg` | The direction B should face when arriving (to see target) |
| `predicted_target` | B needs to know where the target WILL BE, not where it WAS |
| `triangulation_geometry` | Optimal angle for B to maximize triangulation accuracy |
| `optimal_standoff_distance_m` | How far B should stay — too close = target fills FOV, too far = no detection |

#### 6.1.3 TRIANGULATION_RESULT (GS → Both Drones)

Once both drones have bearings, the GS computes and broadcasts the target position:

```json
{
  "msg_type": "TRIANGULATION_RESULT",
  "target_agents": ["DRONE_A", "DRONE_B"],
  "timestamp": "2026-03-24T11:49:30.000Z",

  "target_position": {
    "latitude": 1.2230100,
    "longitude": 103.8491500,
    "accuracy_m": 5.2,
    "method": "DUAL_BEARING_INTERSECTION"
  },

  "bearing_inputs": {
    "drone_a": {
      "position": { "lat": 1.2231877, "lon": 103.8496582 },
      "bearing_deg": 161.2,
      "confidence": 0.78,
      "age_sec": 0.5
    },
    "drone_b": {
      "position": { "lat": 1.2228500, "lon": 103.8488000 },
      "bearing_deg": 72.5,
      "confidence": 0.71,
      "age_sec": 0.3
    }
  },

  "geometry": {
    "angular_separation_deg": 88.7,
    "gdop": 1.13,
    "quality": "GOOD",
    "intersection_distance_a_m": 58.2,
    "intersection_distance_b_m": 44.1
  },

  "tracking_update": {
    "target_heading_deg": 210.0,
    "target_speed_mps": 2.1,
    "prediction_method": "BEARING_RATE_FUSION"
  },

  "commands": {
    "drone_a": { "action": "MAINTAIN_TRACK", "orbit_radius_m": 22 },
    "drone_b": { "action": "MAINTAIN_TRACK", "orbit_radius_m": 22 }
  }
}
```

#### 6.1.4 AGENT_STATUS (Drone → GS, periodic)

Heartbeat with drone state for the orchestrator's situational awareness:

```json
{
  "msg_type": "AGENT_STATUS",
  "source_agent": "DRONE_A",
  "timestamp": "2026-03-24T11:48:10.000Z",

  "position": { "latitude": 1.2231877, "longitude": 103.8496582 },
  "heading_deg": 152.1,
  "speed_mps": 3.2,
  "battery_percent": 66,
  "battery_voltage_v": 11.9,

  "sensor_status": {
    "camera": "ACTIVE",
    "imu": "NOMINAL",
    "gps": "FIX_3D",
    "gps_satellites": 28,
    "compass": "CALIBRATED"
  },

  "tracking_status": {
    "has_target": true,
    "track_id": "track_224",
    "locked": true,
    "lost_count": 0,
    "track_age_frames": 15
  },

  "mission_phase": "TRACKING",
  "estimated_endurance_min": 18.5
}
```

#### 6.1.5 TARGET_LOST (Drone → GS)

Critical event — triggers re-search or handoff:

```json
{
  "msg_type": "TARGET_LOST",
  "source_agent": "DRONE_A",
  "timestamp": "2026-03-24T11:49:00.000Z",

  "last_known": {
    "bearing_deg": 175.3,
    "confidence": 0.52,
    "drone_position": { "lat": 1.2229500, "lon": 103.8493000 },
    "drone_heading_deg": 168.0,
    "target_motion_at_loss": {
      "bearing_rate_deg_per_sec": -5.1,
      "estimated_direction": "LEFT_FAST"
    }
  },

  "lost_reason": "TARGET_LEFT_FOV",
  "lost_duration_frames": 5,
  "recommended_search_heading_deg": 160.0,
  "confidence_decay_rate": 0.1
}
```

---

## 7. Triangulation Algorithm

### 10.1 Bearing-Only Triangulation

Given two bearing lines from known positions, the target is at their intersection:

```
Agent A at (Ax, Az) with bearing θa
Agent B at (Bx, Bz) with bearing θb

Bearing line A: (x, z) = (Ax, Az) + t * (cos(θa), sin(θa))
Bearing line B: (x, z) = (Bx, Bz) + s * (cos(θb), sin(θb))

Solving the intersection:
t = ((Bx - Ax) * sin(θb) - (Bz - Az) * cos(θb)) / (cos(θa) * sin(θb) - sin(θa) * cos(θb))

Target position:
Tx = Ax + t * cos(θa)
Tz = Az + t * sin(θa)
```

### 10.2 Accuracy (GDOP)

Geometric Dilution of Precision depends on the angular separation between bearing lines:

```
GDOP = 1 / sin(angular_separation / 2)
```

| Angular Separation | GDOP | Quality |
|---|---|---|
| 180° | 1.00 | OPTIMAL |
| 120° | 1.15 | EXCELLENT |
| 90° | 1.41 | GOOD |
| 60° | 2.00 | FAIR |
| 30° | 3.86 | POOR |
| 10° | 11.47 | UNUSABLE |

**Minimum viable separation: 60°. Optimal: 90–180°.**

### 10.3 Weighted Triangulation

When bearing confidences differ, use weighted least squares:

```
weight_a = confidence_a * (1 / bearing_age_a)
weight_b = confidence_b * (1 / bearing_age_b)

// Apply Gauss-Newton iteration with weighted residuals
```

---

## 8. Agentic Path Planning (GPS-Denied Interception)

### 10.1 The Challenge

When Agent A detects the target and sends a bearing report to Agent B, Agent B must navigate to a position where:
1. The target will be **within B's camera FOV** on arrival
2. The approach angle creates **good triangulation geometry** (>60° separation from A)
3. B arrives **before the target moves out of the predicted zone**

### 10.2 Target Motion Prediction

From Agent A's bearing reports over time, the GS can estimate target motion:

```
Input: Sequence of TARGET_BEARING_REPORT messages from Agent A
  - bearing_1 at time t1 from position P1
  - bearing_2 at time t2 from position P2
  - ...

Step 1: Bearing Rate → Angular Velocity
  ω = Δbearing / Δtime  (deg/s)

Step 2: Classify Target Motion
  if |ω| < 1 deg/s  → STATIONARY or MOVING_AWAY
  if |ω| 1-5 deg/s  → SLOW_CROSSING
  if |ω| > 5 deg/s  → FAST_CROSSING or TURNING

Step 3: Extrapolate Target Position
  Using bearing from A + estimated range (from bbox size heuristic):
  estimated_range = K / sqrt(bbox_area)   // K = calibration constant

  Predicted target position at time t_future:
  target_future = target_now + target_velocity * (t_future - t_now)
```

### 10.3 Optimal Intercept Point Computation

The GS computes where to send Agent B:

```
Algorithm: COMPUTE_INTERCEPT_WAYPOINT

Inputs:
  - Agent B current position (GPS)
  - Agent B max speed (3.5 m/s from data)
  - Target predicted position at time T
  - Target predicted velocity vector
  - Agent A position (for triangulation geometry)

1. For each candidate time T_arrive in [T+5s, T+10s, ... T+60s]:
   a. Predict target position at T_arrive
   b. Compute intercept point: where B can arrive at T_arrive
      such that target is within B's FOV
   c. Compute angular separation from A's bearing
   d. Score = angular_separation_quality * arrival_probability

2. Select T_arrive with highest score

3. Compute approach heading:
   - B should approach from the OPPOSITE side of A
   - approach_heading = bearing_from_target_to_A + 180° (ideal)
   - Adjust for target motion: lead the target

4. Output: waypoint (lat, lon), approach_bearing, ETA
```

### 10.4 Proportional Navigation (Lead Pursuit)

Instead of pointing B directly at the predicted target position, use proportional navigation to account for target motion:

```
Algorithm: PROPORTIONAL_NAVIGATION

bearing_to_target = atan2(target.z - B.z, target.x - B.x)
bearing_rate = (bearing_to_target - prev_bearing_to_target) / dt

// Navigation constant N (typically 3-5 for sea vehicles)
N = 4.0

// Commanded heading
commanded_heading = bearing_to_target + N * bearing_rate * time_to_intercept

// This naturally leads the target — B steers to where the target WILL BE
```

### 9.5 Path Planning State Machine

```
┌─────────┐    target bearing    ┌─────────────┐
│  SEARCH  │───from Agent A─────→│  COMPUTE     │
│  (sweep) │                     │  INTERCEPT   │
└─────────┘                      └──────┬───────┘
                                        │
                              waypoint  │
                                        ▼
                                 ┌──────────────┐
                                 │  NAVIGATE     │
                                 │  (prop-nav)   │
                                 └──────┬───────┘
                                        │
                              target     │
                              in FOV     ▼
                                 ┌──────────────┐    bearing    ┌───────────────┐
                                 │  ACQUIRE      │───reports───→│  TRIANGULATE   │
                                 │  (lock target)│    to GS     │  (dual bearing)│
                                 └──────────────┘              └───────────────┘
                                                                       │
                                                                       ▼
                                                               ┌───────────────┐
                                                               │  ORBIT/TRACK   │
                                                               │  (maintain sep)│
                                                               └───────────────┘
```

---

## 9. Information Flow by Mission Phase

### 10.1 SEARCH Phase

```
Agent A: Sweep pattern → camera frames → YOLOv8 inference → no detection
Agent B: Sweep pattern → camera frames → YOLOv8 inference → no detection

Both agents send periodic AGENT_STATUS to GS (every 2s)
GS monitors: positions, battery, sensor health
```

**Data communicated:** Own position, heading, speed, battery, "no contact"

### 10.2 DETECTION Phase (A Locks Target)

```
Agent A: Detection! → bbox → compute bearing → TARGET_BEARING_REPORT → GS

GS receives bearing report from A
GS computes: target estimated position (bearing + range heuristic)
GS computes: optimal intercept point for B
GS sends: NAVIGATE_TO_INTERCEPT → Agent B

Agent B: Receives waypoint → begins navigation → uses proportional nav
```

**Critical data communicated A→GS:**
- Own GPS position (bearing line origin)
- Target absolute bearing (bearing line direction)
- Target bearing rate (for prediction)
- Detection confidence (for weighting)
- Bbox area (for range estimation heuristic)

**Critical data communicated GS→B:**
- Intercept waypoint (lat, lon)
- Approach bearing (so B's FOV will see the target)
- Predicted target position + velocity
- Required angular separation for triangulation

### 10.3 TARGET LOST Phase

```
Agent A: Target leaves FOV → TARGET_LOST → GS

GS computes: last known bearing + bearing rate → predicted escape zone
GS sends: updated search waypoints to both A and B
  - A: search near last known position
  - B: continue to predicted intercept zone (target likely heading there)

Key: bearing_rate_at_loss tells us which direction target went
```

**Critical data: `target_motion_at_loss.bearing_rate_deg_per_sec`**
- Positive rate: target was moving RIGHT in camera → fled to the left of drone's heading
- Negative rate: target was moving LEFT in camera → fled to the right

### 10.4 RE-ACQUISITION Phase (B Finds Target)

```
Agent B: Detection! → TARGET_BEARING_REPORT → GS
GS: Now has bearing from B's position
GS → Agent A: NAVIGATE_TO_INTERCEPT (come from opposite side)
```

### 9.5 TRIANGULATION Phase

```
Both agents: Send TARGET_BEARING_REPORT at ~1 Hz

GS: Receives bearing from A + bearing from B
GS: Computes intersection → TRIANGULATION_RESULT → both agents
GS: Monitors angular separation, GDOP
GS: Adjusts orbit commands to maintain optimal geometry

Both agents: Orbit the target maintaining 180° separation at standoff distance
```

**Data required for continuous triangulation:**
- Both agents' GPS positions (updated at 1 Hz)
- Both agents' target bearings (updated at frame rate)
- Bearing confidences (for weighted solution)
- Target bearing rates (for interpolation between updates)

---

## 10. Handling Real-World Challenges

### 10.1 Latency Compensation

From real data, detection runs at ~1.3 FPS with 118ms inference time.

With D2G protocol (45ms latency), a bearing report arrives at GS ~163ms after the frame was captured. At target speed 2 m/s, the target has moved ~0.33m. Over the communication round-trip (A→GS→B), the total delay is ~326ms = 0.65m of target motion.

**Mitigation:** Each message includes a timestamp. The GS extrapolates bearings to a common time reference before computing intersection:

```
bearing_at_reference_time = bearing_at_report_time + bearing_rate * (ref_time - report_time)
```

### 10.2 Wave-Induced Bearing Noise

Real data shows roll up to 5.6° and pitch up to 8.6°. This tilts the camera, introducing systematic bearing error.

**Mitigation:**
1. Apply IMU-based correction to each bearing measurement
2. Increase uncertainty cone when roll/pitch is high
3. Use multiple bearing measurements (Kalman filter) to smooth noise

### 10.3 Intermittent Detection

Real data shows detection gaps (269/394 frames detected). During gaps:
1. Continue using last known bearing + bearing rate for extrapolation
2. Increase uncertainty proportional to `lost_count`
3. If `lost_count` > threshold (e.g., 30 frames = ~23s), declare TARGET_LOST

### 10.4 Battery Constraints

Battery drops 3% over 5 minutes. With ~67% at start:
- Estimated endurance: ~112 minutes at current consumption
- **Include battery in intercept planning** — don't send the low-battery drone on a long intercept

---

## 11. Minimum Viable Data Packet for Triangulation

If bandwidth is severely constrained (e.g., acoustic underwater link), the absolute minimum data needed from each drone for triangulation:

```
MINIMAL_BEARING_PACKET (24 bytes):
  - latitude:      float32 (4 bytes)
  - longitude:     float32 (4 bytes)
  - heading:       float16 (2 bytes)
  - target_bearing: float16 (2 bytes)
  - bearing_rate:  float16 (2 bytes)
  - confidence:    uint8   (1 byte)
  - bbox_cx:       uint16  (2 bytes)  // pixel x of target center
  - bbox_area:     uint16  (2 bytes)  // for range estimation
  - lost_count:    uint8   (1 byte)
  - timestamp_ms:  uint32  (4 bytes)  // ms since epoch, truncated
```

This is **everything the other agent needs** to compute an intercept course and triangulation.

---

## 12. Summary: What Each Agent Must Know to Triangulate

### Agent A (First Detector) Must Send:
1. **Where I am** — GPS lat/lon (bearing line origin)
2. **What direction the target is** — absolute bearing in degrees
3. **How fast the bearing is changing** — bearing rate (deg/s) for prediction
4. **How confident I am** — detection confidence + track quality
5. **How the target is moving in my frame** — pixel velocity

### Agent B (Interceptor) Must Receive:
1. **Where to go** — intercept waypoint from GS (or computed from A's data)
2. **What direction to face** — approach bearing to have target in FOV
3. **Where the target will be** — predicted position at arrival time
4. **What angular separation to maintain** — for good triangulation geometry

### Ground Station Must Compute:
1. **Target estimated position** — from bearing + range heuristic
2. **Target predicted trajectory** — from bearing rate sequence
3. **Optimal intercept point for B** — balancing time, geometry, energy
4. **Triangulation solution** — when both bearings are available
5. **Orbit commands** — maintain 180° separation during tracking

---

## 13. Mapping to V5 Simulation

| Real System | V5 Simulation Equivalent |
|---|---|
| YOLOv8/SAMURAI detection + bbox | `aInRange` / `bInRange` (FOV cone check) |
| Bearing from bbox centroid | `aHeading` / `bHeading` + FOV geometry |
| Dead reckoning (compass + speed) | Drone position in posAt() — validated 3.4% drift |
| TARGET_BEARING_REPORT | `commAB` / `commBA` flags + instruction log |
| NAVIGATE_TO_INTERCEPT | B's approach Bezier curve in posAt() |
| D2D protocol | D2D mode — direct beam visualization |
| D2G protocol | D2G mode — relay through ground station |
| Ground Station orchestrator | GS icon + CommDashboard + routing topology |
| TRIANGULATION_RESULT | PathOptPanel (angular separation, GDOP) |
| Bearing rate (px/s → deg/s) | Target pixel velocity mapped to target evasion |
| Range estimation (bbox area) | FOV cone SENSE_R parameter |
| TARGET_LOST | Scenario 2, phase A_LOST |
| RE-ACQUISITION | Scenario 2, phase B_LOCK |
| Proportional navigation | B's acceleration curve toward predicted intercept |

## 14. Field-Validated Assumptions

| Assumption | Validation | Source |
|---|---|---|
| Drone can self-locate without GPS | Dead reckoning: 3.4% drift over 800m/5min | DR feasibility test |
| Camera can provide target bearing | Bbox centroid → bearing: directionally accurate | YOLO/SAMURAI tracking test |
| Target trajectory can be estimated | Smoothed bbox path matches actual target movement | Right-side DR plot |
| Single drone cannot determine range | Range from bbox is ±30-50% — needs triangulation | Bbox size analysis |
| Two bearings resolve target position | Geometric intersection — validated by DR cross-plot | Triangulation theory + DR data |
| SAMURAI maintains track through gaps | 269/394 frames tracked across multiple lock/lost | sea_drone_data_v1.json |
| IMU provides reliable heading | Compass heading + yaw: stable, supports DR | IMU data analysis |
