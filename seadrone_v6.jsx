import { useState, useEffect, useRef, useCallback } from "react";
import * as THREE from "three";

const OR = 22;
const SENSE_R = 45; // detection range in meters
const FOV_ANGLE = Math.PI * 0.55; // 100° FOV — wider camera for better detection
const FOV_HALF = FOV_ANGLE / 2;
var GS_POS = { x: 0, z: -55 };
var PROTOCOLS = {
  D2D: { id: "D2D", name: "Drone-to-Drone (D2D)", color: "#20d090", latency: 12, bw: 0.8, desc: "Direct mesh link between drones" },
  D2G: { id: "D2G", name: "Drone-Ground-Drone (D2G)", color: "#f0a030", latency: 45, bw: 2.4, desc: "Relay via ground station orchestrator" }
};

// ═══ V6: REAL-TIME SIMULATION CONSTANTS ═══
var DRONE_MAX_SPEED = 3.5;
var DRONE_TURN_RATE = 1.2; // rad/s
var DRONE_ACCEL = 3.0;
var TARGET_MAX_SPEED = 5.0;
var TARGET_TURN_RATE = 1.8;
var TARGET_ACCEL = 6.0;
var DETECT_CONFIRM_TIME = 0.3; // seconds to confirm lock
var LOST_TIMEOUT = 3.0; // seconds before declaring lost
var RELAY_DELAY = 1.5; // seconds before partner reacts
var TRAIL_INTERVAL = 0.15;
var TRAIL_MAX = 800;

// Playground boundary (meters from GS origin)
var BOUNDS = { xMin: -100, xMax: 100, zMin: -70, zMax: 100 };

function lr(a, b, t) { return a + (b - a) * Math.max(0, Math.min(1, t)); }
function d3(a, b) { return Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2); }
function normAng(a) { return ((a + Math.PI * 3) % (Math.PI * 2)) - Math.PI; }

function commState(aP, bP, commAB, commBA, proto) {
  var dAB = d3(aP, bP), dAG = d3(aP, GS_POS), dBG = d3(bP, GS_POS);
  var hasComm = commAB || commBA;
  var sigAB = Math.max(0, 1 - dAB / 80);
  var sigAG = Math.max(0, 1 - dAG / 150);
  var sigBG = Math.max(0, 1 - dBG / 150);
  var latency = 0, bw = 0;
  if (hasComm) {
    if (proto === "D2D") { latency = PROTOCOLS.D2D.latency + (1 - sigAB) * 25; bw = PROTOCOLS.D2D.bw * Math.max(sigAB, 0.1); }
    else { latency = PROTOCOLS.D2G.latency + (1 - sigAG) * 30 + (1 - sigBG) * 30; bw = PROTOCOLS.D2G.bw * Math.min(Math.max(sigAG, 0.1), Math.max(sigBG, 0.1)); }
  }
  var route = !hasComm ? "---" : proto === "D2D" ? (commAB ? "A→B" : "B→A") : (commAB ? "A→GS→B" : "B→GS→A");
  return { proto: proto, dAB: dAB, dAG: dAG, dBG: dBG, sigAB: sigAB, sigAG: sigAG, sigBG: sigBG, latency: latency, bw: bw, route: route, hasComm: hasComm };
}

function inFOV(dronePos, droneHeading, targetPos) {
  var dx = targetPos.x - dronePos.x, dz = targetPos.z - dronePos.z;
  var dist = Math.sqrt(dx * dx + dz * dz);
  if (dist > SENSE_R || dist < 0.5) return false;
  var diff = normAng(Math.atan2(dz, dx) - droneHeading);
  return Math.abs(diff) <= FOV_HALF;
}

function fovTriangle(pos, heading, range) {
  return {
    tip: pos,
    left: { x: pos.x + range * Math.cos(heading - FOV_HALF), z: pos.z + range * Math.sin(heading - FOV_HALF) },
    right: { x: pos.x + range * Math.cos(heading + FOV_HALF), z: pos.z + range * Math.sin(heading + FOV_HALF) }
  };
}

// ════════════════════════════════════════════════════
// V6: AUTONOMOUS DRONE AUTOPILOT
// ════════════════════════════════════════════════════

// Generate expanding spiral search from center outward, biased to assigned side
function genSearchWaypoints(side) {
  var wps = [];
  var cx = side === "left" ? -30 : 30;
  var cz = 20; // start from upper-center area (likely target zone)
  var spacing = 18;
  // Expanding square spiral: center → out
  for (var ring = 0; ring < 6; ring++) {
    var r = spacing * (ring + 1);
    wps.push({ x: cx + r, z: cz + r });
    wps.push({ x: cx - r, z: cz + r });
    wps.push({ x: cx - r, z: cz - r });
    wps.push({ x: cx + r, z: cz - r });
  }
  // Clamp to bounds
  wps = wps.map(function(p) {
    return { x: Math.max(BOUNDS.xMin + 5, Math.min(BOUNDS.xMax - 5, p.x)), z: Math.max(BOUNDS.zMin + 5, Math.min(BOUNDS.zMax - 5, p.z)) };
  });
  return wps;
}

function initialDrone(id, x, z, side) {
  return {
    id: id, x: x, z: z, heading: Math.PI / 2, speed: 0,
    state: "SEARCH", stateTime: 0,
    searchWps: genSearchWaypoints(side), searchIdx: 0,
    targetLastSeen: null, targetBearing: 0, targetDist: 999,
    lostTimer: 0, detectTimer: 0, relayTimer: 0,
    partnerTarget: null
  };
}

function initialSim() {
  return {
    target: { x: 0, z: 40, heading: Math.PI / 2, speed: 0 },
    droneA: initialDrone("A", -3, -30, "left"),
    droneB: initialDrone("B", 3, -30, "right"),
    elapsed: 0, mode: "manual",
    contained: false, containedTime: 0,
    blasted: false, blastTime: 0,
    commAB: false, commBA: false,
    aTrail: [], bTrail: [], tTrail: [], trailTimer: 0
  };
}

function steerToward(drone, targetAngle, dt) {
  var diff = normAng(targetAngle - drone.heading);
  drone.heading += Math.sign(diff) * Math.min(Math.abs(diff), DRONE_TURN_RATE * dt);
}

function stepDrone(drone, target, partner, sim, dt) {
  var tPos = { x: target.x, z: target.z };
  var canSee = inFOV({ x: drone.x, z: drone.z }, drone.heading, tPos);
  var dist = d3({ x: drone.x, z: drone.z }, tPos);
  drone.targetDist = dist;
  drone.stateTime += dt;

  switch (drone.state) {
    case "SEARCH": {
      // Navigate through search waypoints with scanning head movement
      var wps = drone.searchWps || genSearchWaypoints(drone.id === "A" ? "left" : "right");
      if (!drone.searchWps) { drone.searchWps = wps; drone.searchIdx = 0; }
      if (wps.length > 0) {
        var wp = wps[drone.searchIdx % wps.length];
        var angToWp = Math.atan2(wp.z - drone.z, wp.x - drone.x);
        // Add scanning sweep: oscillate heading ±40° around travel direction
        var scanOffset = Math.sin(drone.stateTime * 1.8) * 0.7; // ~40° sweep
        steerToward(drone, angToWp + scanOffset, dt);
        drone.speed = Math.min(drone.speed + DRONE_ACCEL * dt, DRONE_MAX_SPEED * 0.7);
        drone.x += Math.cos(drone.heading) * drone.speed * dt;
        drone.z += Math.sin(drone.heading) * drone.speed * dt;
        if (d3({ x: drone.x, z: drone.z }, wp) < 8) drone.searchIdx++;
      }
      // Check detection — instant lock when target is clearly in FOV
      if (canSee) { drone.detectTimer += dt; } else { drone.detectTimer = Math.max(0, drone.detectTimer - dt * 0.3); }
      if (drone.detectTimer > DETECT_CONFIRM_TIME) { drone.state = "CHASE"; drone.stateTime = 0; drone.lostTimer = 0; }
      // Check if partner relayed target position
      if (drone.partnerTarget) { drone.state = "INTERCEPT"; drone.stateTime = 0; }
      break;
    }
    case "CHASE": {
      // LOCKED ON TARGET — orbit at OR distance, always face target
      // The drone KNOWS the target position — it doesn't rely on FOV to track
      var angToTarget = Math.atan2(tPos.z - drone.z, tPos.x - drone.x);
      drone.targetBearing = angToTarget;
      drone.targetLastSeen = { x: tPos.x, z: tPos.z };

      if (dist > OR + 5) {
        // Too far — close in toward target
        steerToward(drone, angToTarget, dt);
        drone.speed = lr(drone.speed, DRONE_MAX_SPEED, dt * 3);
      } else if (dist < OR - 5) {
        // Too close — back off (steer away)
        steerToward(drone, angToTarget + Math.PI, dt);
        drone.speed = lr(drone.speed, DRONE_MAX_SPEED * 0.5, dt * 3);
      } else {
        // At orbit distance — orbit around target, always facing it
        var orbitDir = angToTarget + Math.PI / 2; // perpendicular = orbit
        steerToward(drone, orbitDir, dt);
        // Keep facing target even while orbiting
        drone.heading = lr(drone.heading, angToTarget, dt * 2);
        drone.speed = lr(drone.speed, target.speed > 0.5 ? DRONE_MAX_SPEED * 0.6 : 0.5, dt * 3);
      }
      drone.x += Math.cos(drone.heading) * drone.speed * dt;
      drone.z += Math.sin(drone.heading) * drone.speed * dt;

      // ALWAYS relay to partner — continuously update target position
      if (drone.stateTime > RELAY_DELAY) {
        partner.partnerTarget = { x: tPos.x, z: tPos.z };
        if (drone.id === "A") sim.commAB = true; else sim.commBA = true;
        if (partner.state === "SEARCH") { partner.state = "INTERCEPT"; partner.stateTime = 0; }
      }

      // Transition to triangulate when partner is also chasing/triangulating
      if (partner.state === "CHASE" || partner.state === "TRIANGULATE") {
        drone.state = "TRIANGULATE"; drone.stateTime = 0;
      }
      break;
    }
    case "INTERCEPT": {
      // Navigate toward relayed target position, then toward target directly when close
      var wp2 = drone.partnerTarget;
      if (!wp2) { drone.state = "SEARCH"; drone.stateTime = 0; break; }
      // If close to target, steer at target directly instead of waypoint
      var steerTarget = dist < SENSE_R * 1.5 ? tPos : wp2;
      var angToWp2 = Math.atan2(steerTarget.z - drone.z, steerTarget.x - drone.x);
      steerToward(drone, angToWp2, dt);
      drone.speed = Math.min(drone.speed + DRONE_ACCEL * dt, DRONE_MAX_SPEED);
      drone.x += Math.cos(drone.heading) * drone.speed * dt;
      drone.z += Math.sin(drone.heading) * drone.speed * dt;
      // Check if we can see the target — fast transition to CHASE
      if (canSee) { drone.detectTimer += dt; } else { drone.detectTimer = Math.max(0, drone.detectTimer - dt); }
      if (drone.detectTimer > DETECT_CONFIRM_TIME * 0.3) {
        drone.state = "CHASE"; drone.stateTime = 0; drone.lostTimer = 0; drone.partnerTarget = null;
      }
      // Update waypoint from partner's ongoing reports
      if ((partner.state === "CHASE" || partner.state === "TRIANGULATE") && partner.targetLastSeen) {
        drone.partnerTarget = { x: partner.targetLastSeen.x, z: partner.targetLastSeen.z };
      }
      // If partner is already triangulating and we can see, jump straight there
      if (canSee && partner.state === "TRIANGULATE") {
        drone.state = "TRIANGULATE"; drone.stateTime = 0;
      }
      break;
    }
    case "TRIANGULATE": {
      // ORBIT the target at OR distance — continuous circular orbit, always face target
      var angFromTarget = Math.atan2(drone.z - tPos.z, drone.x - tPos.x);
      var angToTgt = angFromTarget + Math.PI;

      // Always face the target (for FOV)
      drone.heading = angToTgt;

      // Radial correction: push toward OR distance — fast approach, gentle hold
      var radialSpeed = 0;
      if (dist > OR + 5) radialSpeed = -DRONE_MAX_SPEED * 0.8; // far away — approach fast
      else if (dist > OR + 2) radialSpeed = -DRONE_MAX_SPEED * 0.3; // close — gentle approach
      else if (dist < OR - 2) radialSpeed = DRONE_MAX_SPEED * 0.3; // too close — push out

      // Tangential orbit: continuous circular motion (counterclockwise)
      var orbitSpeed = DRONE_MAX_SPEED * 0.5; // steady orbit speed
      var tangentAng = angFromTarget + Math.PI / 2; // perpendicular = orbit direction

      // Apply both radial correction and tangential orbit
      var moveX = Math.cos(tangentAng) * orbitSpeed * dt + Math.cos(angFromTarget) * radialSpeed * dt;
      var moveZ = Math.sin(tangentAng) * orbitSpeed * dt + Math.sin(angFromTarget) * radialSpeed * dt;
      drone.x += moveX;
      drone.z += moveZ;
      drone.speed = orbitSpeed;

      drone.targetBearing = angToTgt;
      drone.lostTimer = 0;
      break;
    }
    case "STRIKE": {
      // Full speed collision course toward target
      var angToTgt2 = Math.atan2(tPos.z - drone.z, tPos.x - drone.x);
      drone.heading = angToTgt2;
      drone.speed = DRONE_MAX_SPEED * 1.5; // overdrive
      drone.x += Math.cos(drone.heading) * drone.speed * dt;
      drone.z += Math.sin(drone.heading) * drone.speed * dt;
      break;
    }
    case "BLAST": {
      // Dead — no movement
      drone.speed = 0;
      break;
    }
  }
  drone.x = Math.max(BOUNDS.xMin, Math.min(BOUNDS.xMax, drone.x));
  drone.z = Math.max(BOUNDS.zMin, Math.min(BOUNDS.zMax, drone.z));
}

function stepTargetManual(target, keys, dt) {
  var dx = 0, dz = 0;
  if (keys.up) dz += 1;
  if (keys.down) dz -= 1;
  if (keys.left) dx -= 1;
  if (keys.right) dx += 1;
  if (dx !== 0 || dz !== 0) {
    var desired = Math.atan2(dz, dx);
    var diff = ((desired - target.heading + Math.PI * 3) % (Math.PI * 2)) - Math.PI;
    target.heading += Math.sign(diff) * Math.min(Math.abs(diff), TARGET_TURN_RATE * dt);
    target.speed = Math.min(target.speed + TARGET_ACCEL * dt, TARGET_MAX_SPEED);
  } else {
    target.speed *= 0.97;
  }
  if (target.speed < 0.05) target.speed = 0;
  target.x += Math.cos(target.heading) * target.speed * dt;
  target.z += Math.sin(target.heading) * target.speed * dt;
  target.x = Math.max(BOUNDS.xMin + 3, Math.min(BOUNDS.xMax - 3, target.x));
  target.z = Math.max(BOUNDS.zMin + 3, Math.min(BOUNDS.zMax - 3, target.z));
}

function stepTargetAutopilot(target, droneA, droneB, elapsed, dt) {
  // AI target: cruise around, evade when drones get close
  var dA = d3({ x: target.x, z: target.z }, { x: droneA.x, z: droneA.z });
  var dB = d3({ x: target.x, z: target.z }, { x: droneB.x, z: droneB.z });
  var closestDist = Math.min(dA, dB);
  var closestDrone = dA < dB ? droneA : droneB;

  if (closestDist < 50) {
    // Evade! Flee away from closest drone
    var fleeAng = Math.atan2(target.z - closestDrone.z, target.x - closestDrone.x);
    // Add zigzag to make evasion harder
    fleeAng += Math.sin(elapsed * 2) * 0.6;
    var diff = ((fleeAng - target.heading + Math.PI * 3) % (Math.PI * 2)) - Math.PI;
    target.heading += Math.sign(diff) * Math.min(Math.abs(diff), TARGET_TURN_RATE * 1.2 * dt);
    target.speed = Math.min(target.speed + TARGET_ACCEL * dt, TARGET_MAX_SPEED * 0.85);
  } else {
    // Cruise: gentle S-curve patrol
    target.heading += Math.sin(elapsed * 0.3) * 0.3 * dt;
    target.speed = lr(target.speed, 2.0, dt * 2);
  }

  // Bounce off boundaries
  var margin = 15;
  if (target.x < BOUNDS.xMin + margin) target.heading = lr(target.heading, 0, dt * 3);
  if (target.x > BOUNDS.xMax - margin) target.heading = lr(target.heading, Math.PI, dt * 3);
  if (target.z < BOUNDS.zMin + margin) target.heading = lr(target.heading, Math.PI / 2, dt * 3);
  if (target.z > BOUNDS.zMax - margin) target.heading = lr(target.heading, -Math.PI / 2, dt * 3);

  target.x += Math.cos(target.heading) * target.speed * dt;
  target.z += Math.sin(target.heading) * target.speed * dt;
  target.x = Math.max(BOUNDS.xMin + 3, Math.min(BOUNDS.xMax - 3, target.x));
  target.z = Math.max(BOUNDS.zMin + 3, Math.min(BOUNDS.zMax - 3, target.z));
}

// Debug logger
var debugLog = [];
var debugLastSnapshot = 0;
var debugPrevStateA = "", debugPrevStateB = "";

function debugCapture(sim, reason) {
  var tP = { x: sim.target.x, z: sim.target.z };
  var aCanSee = inFOV({ x: sim.droneA.x, z: sim.droneA.z }, sim.droneA.heading, tP);
  var bCanSee = inFOV({ x: sim.droneB.x, z: sim.droneB.z }, sim.droneB.heading, tP);
  debugLog.push({
    t: Math.round(sim.elapsed * 100) / 100,
    reason: reason,
    target: { x: +sim.target.x.toFixed(1), z: +sim.target.z.toFixed(1), hdg: +(sim.target.heading * 180 / Math.PI).toFixed(1), spd: +sim.target.speed.toFixed(2) },
    droneA: {
      x: +sim.droneA.x.toFixed(1), z: +sim.droneA.z.toFixed(1),
      hdg: +(sim.droneA.heading * 180 / Math.PI).toFixed(1), spd: +sim.droneA.speed.toFixed(2),
      state: sim.droneA.state, stateTime: +sim.droneA.stateTime.toFixed(2),
      canSee: aCanSee, dist: +d3({ x: sim.droneA.x, z: sim.droneA.z }, tP).toFixed(1),
      detectTimer: +sim.droneA.detectTimer.toFixed(2), lostTimer: +sim.droneA.lostTimer.toFixed(2),
      partnerTarget: sim.droneA.partnerTarget ? { x: +sim.droneA.partnerTarget.x.toFixed(1), z: +sim.droneA.partnerTarget.z.toFixed(1) } : null,
      searchIdx: sim.droneA.searchIdx
    },
    droneB: {
      x: +sim.droneB.x.toFixed(1), z: +sim.droneB.z.toFixed(1),
      hdg: +(sim.droneB.heading * 180 / Math.PI).toFixed(1), spd: +sim.droneB.speed.toFixed(2),
      state: sim.droneB.state, stateTime: +sim.droneB.stateTime.toFixed(2),
      canSee: bCanSee, dist: +d3({ x: sim.droneB.x, z: sim.droneB.z }, tP).toFixed(1),
      detectTimer: +sim.droneB.detectTimer.toFixed(2), lostTimer: +sim.droneB.lostTimer.toFixed(2),
      partnerTarget: sim.droneB.partnerTarget ? { x: +sim.droneB.partnerTarget.x.toFixed(1), z: +sim.droneB.partnerTarget.z.toFixed(1) } : null,
      searchIdx: sim.droneB.searchIdx
    },
    commAB: sim.commAB, commBA: sim.commBA, contained: sim.contained
  });
}

function stepSim(sim, keys, dt) {
  sim.elapsed += dt;

  // Debug: capture on state transitions
  if (sim.droneA.state !== debugPrevStateA || sim.droneB.state !== debugPrevStateB) {
    debugCapture(sim, "STATE_CHANGE: A=" + debugPrevStateA + "→" + sim.droneA.state + " B=" + debugPrevStateB + "→" + sim.droneB.state);
    debugPrevStateA = sim.droneA.state;
    debugPrevStateB = sim.droneB.state;
  }
  // Debug: periodic snapshot every 0.5s
  if (sim.elapsed - debugLastSnapshot > 0.5) {
    debugCapture(sim, "PERIODIC");
    debugLastSnapshot = sim.elapsed;
  }
  // Triangulation → contained → strike → blast
  if (sim.droneA.state === "TRIANGULATE" && sim.droneB.state === "TRIANGULATE" && !sim.contained) {
    sim.contained = true;
    sim.containedTime = 0;
  }
  if (sim.contained && !sim.blasted) {
    // Only count orbiting time when BOTH drones are near orbit distance
    var aOrbitDist = d3({ x: sim.droneA.x, z: sim.droneA.z }, { x: sim.target.x, z: sim.target.z });
    var bOrbitDist = d3({ x: sim.droneB.x, z: sim.droneB.z }, { x: sim.target.x, z: sim.target.z });
    var bothInOrbit = aOrbitDist < OR + 8 && bOrbitDist < OR + 8;
    if (bothInOrbit) {
      sim.containedTime = (sim.containedTime || 0) + dt;
    }
    // After 6 seconds of BOTH orbiting → STRIKE
    if (sim.containedTime > 6 && sim.droneA.state !== "STRIKE" && sim.droneA.state !== "BLAST") {
      sim.droneA.state = "STRIKE"; sim.droneA.stateTime = 0;
      sim.droneB.state = "STRIKE"; sim.droneB.stateTime = 0;
    }
  }
  // Check blast — both drones reach target
  var aDist = d3({ x: sim.droneA.x, z: sim.droneA.z }, { x: sim.target.x, z: sim.target.z });
  var bDist = d3({ x: sim.droneB.x, z: sim.droneB.z }, { x: sim.target.x, z: sim.target.z });
  if (sim.droneA.state === "STRIKE" && sim.droneB.state === "STRIKE" && aDist < 4 && bDist < 4 && !sim.blasted) {
    sim.blasted = true;
    sim.blastTime = sim.elapsed;
    sim.droneA.state = "BLAST"; sim.droneB.state = "BLAST";
  }

  // Freeze target when contained, otherwise step based on mode
  if (!sim.contained) {
    if (sim.mode === "autopilot") {
      stepTargetAutopilot(sim.target, sim.droneA, sim.droneB, sim.elapsed, dt);
    } else {
      stepTargetManual(sim.target, keys, dt);
    }
  } else {
    sim.target.speed *= 0.95;
    if (sim.target.speed < 0.05) sim.target.speed = 0;
    sim.target.x += Math.cos(sim.target.heading) * sim.target.speed * dt;
    sim.target.z += Math.sin(sim.target.heading) * sim.target.speed * dt;
  }
  stepDrone(sim.droneA, sim.target, sim.droneB, sim, dt);
  stepDrone(sim.droneB, sim.target, sim.droneA, sim, dt);

  // Post-step: FORCE triangulation if both drones can see the target — no exceptions
  var tP = { x: sim.target.x, z: sim.target.z };
  var aCanSee = inFOV({ x: sim.droneA.x, z: sim.droneA.z }, sim.droneA.heading, tP);
  var bCanSee = inFOV({ x: sim.droneB.x, z: sim.droneB.z }, sim.droneB.heading, tP);
  if (aCanSee && bCanSee) {
    // Both have visual — triangulate immediately regardless of current state
    if (sim.droneA.state !== "TRIANGULATE") { sim.droneA.state = "TRIANGULATE"; sim.droneA.stateTime = 0; sim.droneA.lostTimer = 0; }
    if (sim.droneB.state !== "TRIANGULATE") { sim.droneB.state = "TRIANGULATE"; sim.droneB.stateTime = 0; sim.droneB.lostTimer = 0; }
    sim.commAB = true; sim.commBA = true;
  }

  // Also: if one drone is chasing and close to target, and the other can see → triangulate
  var aDist = d3({ x: sim.droneA.x, z: sim.droneA.z }, tP);
  var bDist = d3({ x: sim.droneB.x, z: sim.droneB.z }, tP);
  if (aCanSee && bDist < SENSE_R && sim.droneB.state !== "SEARCH") {
    sim.droneA.state = "TRIANGULATE"; sim.droneA.stateTime = 0;
    sim.droneB.state = "TRIANGULATE"; sim.droneB.stateTime = 0;
    sim.commAB = true; sim.commBA = true;
  }
  if (bCanSee && aDist < SENSE_R && sim.droneA.state !== "SEARCH") {
    sim.droneA.state = "TRIANGULATE"; sim.droneA.stateTime = 0;
    sim.droneB.state = "TRIANGULATE"; sim.droneB.stateTime = 0;
    sim.commAB = true; sim.commBA = true;
  }

  if (sim.droneA.state === "SEARCH" && sim.droneB.state === "SEARCH") { sim.commAB = false; sim.commBA = false; }
  // Accumulate trails
  sim.trailTimer += dt;
  if (sim.trailTimer > TRAIL_INTERVAL) {
    sim.trailTimer = 0;
    if (sim.aTrail.length < TRAIL_MAX) sim.aTrail.push({ x: sim.droneA.x, z: sim.droneA.z });
    if (sim.bTrail.length < TRAIL_MAX) sim.bTrail.push({ x: sim.droneB.x, z: sim.droneB.z });
    if (sim.tTrail.length < TRAIL_MAX) sim.tTrail.push({ x: sim.target.x, z: sim.target.z });
  }
}

function simSnapshot(sim) {
  var aP = { x: sim.droneA.x, z: sim.droneA.z };
  var bP = { x: sim.droneB.x, z: sim.droneB.z };
  var tg = { x: sim.target.x, z: sim.target.z };
  var aH = sim.droneA.heading, bH = sim.droneB.heading;
  var aInR = inFOV(aP, aH, tg), bInR = inFOV(bP, bH, tg);
  var aD = sim.droneA.state !== "SEARCH";
  var bD = sim.droneB.state !== "SEARCH";
  var cm = sim.commAB || sim.commBA;
  var pi = 1;
  if (sim.blasted) pi = 8;
  else if (sim.droneA.state === "STRIKE") pi = 7;
  else if (sim.contained) pi = 6;
  else if (sim.droneA.state === "TRIANGULATE" && sim.droneB.state === "TRIANGULATE") pi = 5;
  else if (aD && bD) pi = 4;
  else if (cm) pi = 3;
  else if (aD || bD) pi = 2;
  return { aP: aP, bP: bP, tg: tg, aD: aD, bD: bD, cm: cm, commAB: sim.commAB, commBA: sim.commBA, aInRange: aInR, bInRange: bInR, aHeading: aH, bHeading: bH, pi: pi, lt: 0 };
}

function simStatusText(sim) {
  var a = sim.droneA.state, b = sim.droneB.state;
  if (sim.blasted) return { text: "TARGET DESTROYED", color: "#c03030", icon: "💥" };
  if (a === "STRIKE") return { text: "STRIKE INBOUND", color: "#c03030", icon: "⚡" };
  var orbTime = sim.containedTime || 0;
  if (sim.contained) return { text: orbTime > 0 ? "ORBITING — STRIKE IN " + Math.max(0, 6 - orbTime).toFixed(0) + "s" : "CONTAINED — WAITING FOR ORBIT", color: "#c06020", icon: "●" };
  if (a === "TRIANGULATE" && b === "TRIANGULATE") return { text: "TRIANGULATING", color: "#0a6a5a", icon: "◉" };
  if (a === "CHASE" && b === "CHASE") return { text: "DUAL CHASE", color: "#c06020", icon: "◉" };
  if (sim.commBA) return { text: "B→A RELAY · A INTERCEPTING", color: "#6050e0", icon: "◈" };
  if (sim.commAB) return { text: "A→B RELAY · B INTERCEPTING", color: "#20d090", icon: "◈" };
  if (a === "CHASE") return { text: "A CHASING · B " + b, color: "#c89020", icon: "◐" };
  if (b === "CHASE") return { text: "B CHASING · A " + a, color: "#c89020", icon: "◐" };
  if (a === "INTERCEPT" || b === "INTERCEPT") return { text: "INTERCEPTING", color: "#c06020", icon: "◎" };
  return { text: "SEARCHING", color: "#1a80c0", icon: "○" };
}

// Dummy for compat — not used in v6 but some components reference it
var SCENARIOS = {}; // empty, no scripted scenarios in v6
function Map2D({ state, trail, proto, elapsed, blasted }) {
  var s = state;
  var ts = s.pi >= 5;
  var SC = 4.5;
  var W = 600, H = 600;
  var cx = function(x) { return W / 2 + x * SC; };
  var cy = function(z) { return H * 0.45 - z * SC; };

  var aTrail = trail.a, bTrail = trail.b, tTrail = trail.t;

  var pD = function(arr) {
    if (arr.length < 2) return "";
    return arr.map(function(p, i) { return (i ? "L" : "M") + cx(p.x).toFixed(1) + "," + cy(p.z).toFixed(1); }).join(" ");
  };

  var dA = s.aD ? d3(s.aP, s.tg).toFixed(0) : null;
  var dB = s.bD ? d3(s.bP, s.tg).toFixed(0) : null;

  return (
    <svg viewBox={"0 0 " + W + " " + H} style={{ display: "block", background: "#f4f8fb", borderRadius: 6, border: "1px solid #e0e8f0", width: "100%", flex: 1 }} preserveAspectRatio="xMidYMid meet">
      {Array.from({ length: 21 }, function(_, i) { return <line key={"v" + i} x1={i * 30} y1="0" x2={i * 30} y2={H} stroke="#e8eff5" strokeWidth=".5" />; })}
      {Array.from({ length: 21 }, function(_, i) { return <line key={"h" + i} x1="0" y1={i * 30} x2={W} y2={i * 30} stroke="#e8eff5" strokeWidth=".5" />; })}

      {/* Playground boundary */}
      <rect x={cx(BOUNDS.xMin)} y={cy(BOUNDS.zMax)} width={(BOUNDS.xMax - BOUNDS.xMin) * SC} height={(BOUNDS.zMax - BOUNDS.zMin) * SC} fill="none" stroke="#c03030" strokeWidth="1.5" strokeDasharray="8 4" opacity=".3" rx="4" />
      <text x={cx(BOUNDS.xMax) - 4} y={cy(BOUNDS.zMax) + 12} textAnchor="end" style={{ fontSize: "8px", fill: "#c03030", fontFamily: "monospace", opacity: 0.5 }}>BOUNDARY</text>

      {aTrail.length > 1 && <path d={pD(aTrail)} fill="none" stroke="#0a8a5a" strokeWidth="2" opacity=".25" />}
      {bTrail.length > 1 && <path d={pD(bTrail)} fill="none" stroke="#4838d0" strokeWidth="2" opacity=".25" />}
      {tTrail.length > 1 && <path d={pD(tTrail)} fill="none" stroke="#c03030" strokeWidth="1.5" opacity=".2" strokeDasharray="4 5" />}

      {/* ── FOV Vision Cones (V-shaped) ── */}
      {(function() {
        var fovA = s.aHeading != null ? fovTriangle(s.aP, s.aHeading, SENSE_R) : null;
        var fovB = s.bHeading != null ? fovTriangle(s.bP, s.bHeading, SENSE_R) : null;
        var aCol = s.aInRange ? "#0a8a5a" : "#90a8b8";
        var bCol = s.bInRange ? "#4838d0" : "#90a8b8";
        // Build arc path for rounded FOV cone
        function fovPath(fov, heading) {
          var r = SENSE_R * SC;
          // SVG arc: from left edge, arc to right edge, line back to tip
          var lx = cx(fov.left.x), ly = cy(fov.left.z);
          var rx = cx(fov.right.x), ry = cy(fov.right.z);
          var tx = cx(fov.tip.x), ty = cy(fov.tip.z);
          return "M" + tx.toFixed(1) + "," + ty.toFixed(1) +
                 " L" + lx.toFixed(1) + "," + ly.toFixed(1) +
                 " A" + r.toFixed(1) + "," + r.toFixed(1) + " 0 0,1 " + rx.toFixed(1) + "," + ry.toFixed(1) +
                 " Z";
        }
        return <g>
          {/* A FOV cone */}
          {fovA && <path d={fovPath(fovA, s.aHeading)} fill={aCol} fillOpacity={s.aInRange ? 0.12 : 0.04} stroke={aCol} strokeWidth={s.aInRange ? "1.5" : "0.8"} opacity={s.aInRange ? ".5" : ".25"} strokeDasharray={s.aInRange ? "none" : "5 3"} />}
          {/* B FOV cone */}
          {fovB && <path d={fovPath(fovB, s.bHeading)} fill={bCol} fillOpacity={s.bInRange ? 0.12 : 0.04} stroke={bCol} strokeWidth={s.bInRange ? "1.5" : "0.8"} opacity={s.bInRange ? ".5" : ".25"} strokeDasharray={s.bInRange ? "none" : "5 3"} />}

          {/* A lock-on crosshair */}
          {s.aInRange && <g>
            <line x1={cx(s.tg.x) - 12} y1={cy(s.tg.z)} x2={cx(s.tg.x) - 5} y2={cy(s.tg.z)} stroke="#0a8a5a" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x) + 5} y1={cy(s.tg.z)} x2={cx(s.tg.x) + 12} y2={cy(s.tg.z)} stroke="#0a8a5a" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x)} y1={cy(s.tg.z) - 12} x2={cx(s.tg.x)} y2={cy(s.tg.z) - 5} stroke="#0a8a5a" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x)} y1={cy(s.tg.z) + 5} x2={cx(s.tg.x)} y2={cy(s.tg.z) + 12} stroke="#0a8a5a" strokeWidth="1.5" opacity=".6" />
            <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="14" fill="none" stroke="#0a8a5a" strokeWidth="1" opacity=".35" strokeDasharray="3 3">
              <animateTransform attributeName="transform" type="rotate" from={"0 " + cx(s.tg.x) + " " + cy(s.tg.z)} to={"360 " + cx(s.tg.x) + " " + cy(s.tg.z)} dur="4s" repeatCount="indefinite" />
            </circle>
            <text x={cx(s.aP.x)} y={cy(s.aP.z) + 18} textAnchor="middle" style={{ fontSize: "10px", fill: "#0a8a5a", fontFamily: "monospace", fontWeight: 700 }}>A LOCKED</text>
          </g>}
          {/* B lock-on crosshair */}
          {s.bInRange && <g>
            <line x1={cx(s.tg.x) - 12} y1={cy(s.tg.z)} x2={cx(s.tg.x) - 5} y2={cy(s.tg.z)} stroke="#4838d0" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x) + 5} y1={cy(s.tg.z)} x2={cx(s.tg.x) + 12} y2={cy(s.tg.z)} stroke="#4838d0" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x)} y1={cy(s.tg.z) - 12} x2={cx(s.tg.x)} y2={cy(s.tg.z) - 5} stroke="#4838d0" strokeWidth="1.5" opacity=".6" />
            <line x1={cx(s.tg.x)} y1={cy(s.tg.z) + 5} x2={cx(s.tg.x)} y2={cy(s.tg.z) + 12} stroke="#4838d0" strokeWidth="1.5" opacity=".6" />
            <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="14" fill="none" stroke="#4838d0" strokeWidth="1" opacity=".35" strokeDasharray="3 3">
              <animateTransform attributeName="transform" type="rotate" from={"0 " + cx(s.tg.x) + " " + cy(s.tg.z)} to={"-360 " + cx(s.tg.x) + " " + cy(s.tg.z)} dur="4s" repeatCount="indefinite" />
            </circle>
            <text x={cx(s.bP.x)} y={cy(s.bP.z) + 18} textAnchor="middle" style={{ fontSize: "10px", fill: "#4838d0", fontFamily: "monospace", fontWeight: 700 }}>B LOCKED</text>
          </g>}
          {/* Dual lock */}
          {s.aInRange && s.bInRange && <g>
            <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="20" fill="none" stroke="#c06020" strokeWidth="1.5" opacity=".4" strokeDasharray="4 2">
              <animateTransform attributeName="transform" type="rotate" from={"0 " + cx(s.tg.x) + " " + cy(s.tg.z)} to={"360 " + cx(s.tg.x) + " " + cy(s.tg.z)} dur="3s" repeatCount="indefinite" />
            </circle>
            <text x={cx(s.tg.x)} y={cy(s.tg.z) - 26} textAnchor="middle" style={{ fontSize: "10px", fill: "#c06020", fontFamily: "monospace", fontWeight: 700 }}>DUAL LOCK</text>
          </g>}
        </g>;
      })()}

      {/* Ground Station icon */}
      <rect x={cx(GS_POS.x) - 9} y={cy(GS_POS.z) - 9} width="18" height="18" rx="3" fill="#1a2a3a" stroke="#f0a030" strokeWidth="1.5" />
      <text x={cx(GS_POS.x)} y={cy(GS_POS.z) + 4} textAnchor="middle" style={{ fontSize: "9px", fill: "#f0a030", fontFamily: "monospace", fontWeight: 700 }}>GS</text>
      <text x={cx(GS_POS.x)} y={cy(GS_POS.z) + 24} textAnchor="middle" style={{ fontSize: "7px", fill: "#8a6a2a", fontFamily: "monospace" }}>GROUND STN</text>

      {/* Protocol-aware comm links */}
      {(function() {
        var hasComm = s.commAB || s.commBA;
        if (!hasComm) return null;
        var txP = s.commAB ? s.aP : s.bP;
        var rxP = s.commAB ? s.bP : s.aP;
        var txLabel = s.commAB ? "A" : "B";
        var rxLabel = s.commAB ? "B" : "A";

        if (proto === "D2D") {
          return <g>
            <line x1={cx(txP.x)} y1={cy(txP.z)} x2={cx(rxP.x)} y2={cy(rxP.z)} stroke="#20d090" strokeWidth="2" opacity=".35" />
            {[0, .25, .5, .75].map(function(off, ci) {
              var ct = ((elapsed * 1.5 + off) % 1);
              return <circle key={"d2d" + ci} cx={lr(cx(txP.x), cx(rxP.x), ct)} cy={lr(cy(txP.z), cy(rxP.z), ct)} r={3 + 2 * Math.sin(ct * Math.PI)} fill="#20d090" opacity={.8 * Math.sin(ct * Math.PI)} />;
            })}
            <text x={(cx(txP.x) + cx(rxP.x)) / 2} y={(cy(txP.z) + cy(rxP.z)) / 2 - 10} textAnchor="middle" style={{ fontSize: "9px", fill: "#20d090", fontFamily: "monospace", fontWeight: 700 }}>D2D {txLabel}→{rxLabel}</text>
          </g>;
        } else {
          return <g>
            {/* TX → GS */}
            <line x1={cx(txP.x)} y1={cy(txP.z)} x2={cx(GS_POS.x)} y2={cy(GS_POS.z)} stroke="#f0a030" strokeWidth="1.5" opacity=".3" strokeDasharray="5 4" />
            {[0, .33, .66].map(function(off, ci) {
              var ct = ((elapsed * 0.8 + off) % 1);
              return <circle key={"d2g1" + ci} cx={lr(cx(txP.x), cx(GS_POS.x), ct)} cy={lr(cy(txP.z), cy(GS_POS.z), ct)} r={2.5 + 1.5 * Math.sin(ct * Math.PI)} fill="#f0a030" opacity={.7 * Math.sin(ct * Math.PI)} />;
            })}
            {/* GS → RX */}
            <line x1={cx(GS_POS.x)} y1={cy(GS_POS.z)} x2={cx(rxP.x)} y2={cy(rxP.z)} stroke="#f0a030" strokeWidth="1.5" opacity=".3" strokeDasharray="5 4" />
            {[0, .33, .66].map(function(off, ci) {
              var ct = ((elapsed * 0.8 + off + .15) % 1);
              return <circle key={"d2g2" + ci} cx={lr(cx(GS_POS.x), cx(rxP.x), ct)} cy={lr(cy(GS_POS.z), cy(rxP.z), ct)} r={2.5 + 1.5 * Math.sin(ct * Math.PI)} fill="#f0a030" opacity={.7 * Math.sin(ct * Math.PI)} />;
            })}
            <text x={cx(GS_POS.x)} y={cy(GS_POS.z) - 16} textAnchor="middle" style={{ fontSize: "9px", fill: "#f0a030", fontFamily: "monospace", fontWeight: 700 }}>RELAY {txLabel}→GS→{rxLabel}</text>
          </g>;
        }
      })()}

      {/* Triangulation orbit + link */}
      {s.pi >= 5 && <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r={OR * SC} fill="none" stroke="#0a8a5a" strokeWidth="1" opacity=".15" strokeDasharray="6 6" />}
      {s.pi >= 5 && <line x1={cx(s.aP.x)} y1={cy(s.aP.z)} x2={cx(s.bP.x)} y2={cy(s.bP.z)} stroke="#0a8a5a" strokeWidth=".5" opacity=".12" strokeDasharray="5 7" />}

      {/* Bearing lines */}
      {s.aD && <line x1={cx(s.aP.x)} y1={cy(s.aP.z)} x2={cx(s.tg.x)} y2={cy(s.tg.z)} stroke="#c89020" strokeWidth="1.4" opacity=".4" strokeDasharray="6 5" />}
      {s.bD && <line x1={cx(s.bP.x)} y1={cy(s.bP.z)} x2={cx(s.tg.x)} y2={cy(s.tg.z)} stroke="#c06020" strokeWidth="1.4" opacity=".4" strokeDasharray="6 5" />}

      {/* Target */}
      {!blasted && <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="9" fill="#d03030" stroke="#fff" strokeWidth="2" />}
      {!blasted && <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="3.5" fill="#f8a0a0" />}
      {!blasted && <text x={cx(s.tg.x) + 14} y={cy(s.tg.z) + 5} style={{ fontSize: "13px", fill: ts ? "#0a8a5a" : s.pi === 3 ? "#c03030" : "#b02020", fontFamily: "monospace", fontWeight: 700 }}>
        {s.pi >= 7 ? "STRIKE" : ts ? "IDLE" : s.pi >= 2 ? "EVADING" : ""}
      </text>}
      {/* Blast effect on 2D map */}
      {blasted && <g>
        <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="30" fill="#ff4400" fillOpacity="0.4">
          <animate attributeName="r" from="5" to="60" dur="2s" fill="freeze" />
          <animate attributeName="fill-opacity" from="0.8" to="0" dur="2s" fill="freeze" />
        </circle>
        <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="15" fill="#ffff00" fillOpacity="0.6">
          <animate attributeName="r" from="3" to="35" dur="1.5s" fill="freeze" />
          <animate attributeName="fill-opacity" from="1" to="0" dur="1.5s" fill="freeze" />
        </circle>
        <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="50" fill="none" stroke="#ff2200" strokeWidth="3" opacity="0.5">
          <animate attributeName="r" from="10" to="80" dur="2.5s" fill="freeze" />
          <animate attributeName="opacity" from="0.8" to="0" dur="2.5s" fill="freeze" />
        </circle>
        <text x={cx(s.tg.x)} y={cy(s.tg.z) - 5} textAnchor="middle" style={{ fontSize: "18px", fill: "#ff4400", fontFamily: "monospace", fontWeight: 900 }}>
          DESTROYED
        </text>
      </g>}

      {/* Drone A */}
      <polygon points={cx(s.aP.x) + "," + (cy(s.aP.z) - 9) + " " + (cx(s.aP.x) - 8) + "," + (cy(s.aP.z) + 7) + " " + (cx(s.aP.x) + 8) + "," + (cy(s.aP.z) + 7)} fill="#0a8a5a" stroke="#fff" strokeWidth="1.2" />
      <text x={cx(s.aP.x)} y={cy(s.aP.z) - 16} textAnchor="middle" style={{ fontSize: "13px", fill: "#0a8a5a", fontFamily: "monospace", fontWeight: 700 }}>
        {"A" + (dA ? " " + dA + "m" : "") + (s.pi === 3 && !s.aD ? " LOST" : "")}
      </text>

      {/* Drone B */}
      <polygon points={cx(s.bP.x) + "," + (cy(s.bP.z) - 9) + " " + (cx(s.bP.x) - 8) + "," + (cy(s.bP.z) + 7) + " " + (cx(s.bP.x) + 8) + "," + (cy(s.bP.z) + 7)} fill="#4838d0" stroke="#fff" strokeWidth="1.2" />
      <text x={cx(s.bP.x)} y={cy(s.bP.z) - 16} textAnchor="middle" style={{ fontSize: "13px", fill: "#4838d0", fontFamily: "monospace", fontWeight: 700 }}>
        {"B" + (dB ? " " + dB + "m" : "")}
      </text>

      {s.pi >= 5 && <g>
        <rect x={W - 130} y="10" width="118" height="28" rx="5" fill="#eaf6f0" stroke="#0a8a5a30" strokeWidth=".6" />
        <text x={W - 71} y="29" textAnchor="middle" style={{ fontSize: "13px", fill: "#0a8a5a", fontFamily: "monospace", fontWeight: 700 }}>180° OPT</text>
      </g>}

      <line x1="14" y1={H - 18} x2={14 + 20 * SC} y2={H - 18} stroke="#90a0b0" strokeWidth="1" />
      <text x={14 + 10 * SC} y={H - 6} textAnchor="middle" style={{ fontSize: "11px", fill: "#90a0b0", fontFamily: "monospace" }}>20m</text>
    </svg>
  );
}

// ════════════════════════════════════════════════════
// IMU TELEMETRY PANEL
// ════════════════════════════════════════════════════
// V6: speeds are directly available from sim, no TICK_DT needed

function IMUPanel({ label, color, pos, heading, speed, roll, pitch, bearing, dist, inRange }) {
  var hdg = ((heading * 180 / Math.PI) + 360) % 360;
  var brg = ((bearing * 180 / Math.PI) + 360) % 360;
  return (
    <div style={{ background: "#0a1520e8", color: "#8ab0c8", padding: "3px 6px", borderRadius: "0 0 4px 4px", border: "1px solid " + color + "25", borderTop: "none", fontSize: 8, fontFamily: "monospace", display: "flex", gap: 8, flexWrap: "wrap", lineHeight: "13px" }}>
      <span>HDG <span style={{ color: "#e0f0ff" }}>{hdg.toFixed(0)}°</span></span>
      <span>SPD <span style={{ color: "#e0f0ff" }}>{speed.toFixed(1)}</span></span>
      <span>BRG <span style={{ color: inRange ? "#ffd700" : "#e0f0ff" }}>{brg.toFixed(0)}°</span></span>
      <span>DST <span style={{ color: inRange ? "#ffd700" : "#e0f0ff" }}>{dist.toFixed(0)}m</span></span>
    </div>
  );
}

// ════════════════════════════════════════════════════
// FPV CAMERA HUD OVERLAY
// ════════════════════════════════════════════════════
function FPVOverlay({ heading, inRange, dist, color }) {
  var hdg = ((heading * 180 / Math.PI) + 360) % 360;
  return (
    <div style={{ position: "absolute", top: 0, left: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
      <svg width="100%" height="100%" viewBox="0 0 200 140" preserveAspectRatio="xMidYMid meet" style={{ position: "absolute", top: 0, left: 0 }}>
        <line x1="90" y1="70" x2="110" y2="70" stroke="#0f0" strokeWidth="1" opacity=".5" />
        <line x1="100" y1="60" x2="100" y2="80" stroke="#0f0" strokeWidth="1" opacity=".5" />
        <circle cx="100" cy="70" r="18" fill="none" stroke="#0f0" strokeWidth=".5" opacity=".3" />
        <circle cx="100" cy="70" r="6" fill="none" stroke="#0f0" strokeWidth=".5" opacity=".4" />
      </svg>
      <div style={{ position: "absolute", top: 2, width: "100%", textAlign: "center", fontSize: 9, color: "#0f0", opacity: .7, textShadow: "0 0 4px #000" }}>
        HDG {hdg.toFixed(0)}°
      </div>
      {inRange && <div style={{ position: "absolute", bottom: 3, width: "100%", textAlign: "center", fontSize: 10, color: "#ffd700", fontWeight: 700, textShadow: "0 0 6px #000" }}>
        TARGET LOCKED — {dist.toFixed(0)}m
      </div>}
    </div>
  );
}

// ════════════════════════════════════════════════════
// INSTRUCTION SHARING LOG
// ════════════════════════════════════════════════════
function getInstructions(sim, st, proto) {
  var msgs = [];
  var brg = function(from, to) { return (((Math.atan2(to.z - from.z, to.x - from.x) * 180 / Math.PI) + 360) % 360).toFixed(0); };
  var dst = function(a, b) { return d3(a, b).toFixed(0); };
  var tag = "[" + proto + "] ";
  function relay(from, to, text) {
    if (proto === "D2D") { msgs.push({ from: from, to: to, c: "#20d090", text: tag + text }); }
    else { msgs.push({ from: from, to: "GS", c: "#f0a030", text: tag + text }); msgs.push({ from: "GS", to: to, c: "#f0a030", text: "RELAY: " + text }); }
  }
  var a = sim.droneA.state, b = sim.droneB.state;
  if (a === "SEARCH" && b === "SEARCH") {
    msgs.push({ from: "A", to: "GS", c: "#0a8a5a", text: tag + "Searching. No contact." });
    msgs.push({ from: "B", to: "GS", c: "#4838d0", text: tag + "Searching. No contact." });
  } else if (a === "CHASE" && !st.commAB && !st.commBA) {
    msgs.push({ from: "A", to: "GS", c: "#c89020", text: tag + "CONTACT BRG " + brg(st.aP, st.tg) + "° R:" + dst(st.aP, st.tg) + "m." });
  } else if (b === "CHASE" && !st.commAB && !st.commBA) {
    msgs.push({ from: "B", to: "GS", c: "#c89020", text: tag + "CONTACT BRG " + brg(st.bP, st.tg) + "° R:" + dst(st.bP, st.tg) + "m." });
  } else if (st.commAB) {
    relay("A", "B", "TARGET at (" + st.tg.x.toFixed(0) + "," + st.tg.z.toFixed(0) + "). BRG " + brg(st.aP, st.tg) + "°. Intercept!");
    if (b === "INTERCEPT") msgs.push({ from: "B", to: "GS", c: "#4838d0", text: tag + "ACK. En route to intercept." });
  } else if (st.commBA) {
    relay("B", "A", "TARGET at (" + st.tg.x.toFixed(0) + "," + st.tg.z.toFixed(0) + "). BRG " + brg(st.bP, st.tg) + "°. Intercept!");
    if (a === "INTERCEPT") msgs.push({ from: "A", to: "GS", c: "#0a8a5a", text: tag + "ACK. En route to intercept." });
  }
  if (a === "TRIANGULATE" && b === "TRIANGULATE") {
    relay("A", "B", "TRIANGULATE. 180° sep. Orbit R:" + OR + "m.");
    relay("B", "A", "Copy. Target contained.");
  }
  return msgs;
}

function InstructionLog({ messages }) {
  return (
    <div style={{ background: "#0a1520e8", borderRadius: 5, padding: "4px 6px", border: "1px solid #20d09020", fontSize: 8, fontFamily: "monospace", lineHeight: "12px" }}>
      <div style={{ fontWeight: 700, color: "#20d090", marginBottom: 2, fontSize: 7, letterSpacing: 0.5 }}>COMM LOG</div>
      {messages.slice(0, 4).map(function(m, i) {
        return <div key={i} style={{ padding: "1px 0", color: m.c || "#8ab0c8", overflow: "hidden", textOverflow: "ellipsis", whiteSpace: "nowrap" }}>
          <span style={{ fontWeight: 700, color: "#b0c8d8" }}>[{m.from}→{m.to}]</span> {m.text}
        </div>;
      })}
    </div>
  );
}

// ════════════════════════════════════════════════════
// PATH OPTIMIZATION PANEL
// ════════════════════════════════════════════════════
function PathOptPanel({ st, triangPhaseIdx }) {
  if (st.pi < triangPhaseIdx) return null;
  var angA = Math.atan2(st.aP.z - st.tg.z, st.aP.x - st.tg.x);
  var angB = Math.atan2(st.bP.z - st.tg.z, st.bP.x - st.tg.x);
  var sep = Math.abs(angA - angB);
  if (sep > Math.PI) sep = 2 * Math.PI - sep;
  var sepDeg = sep * 180 / Math.PI;
  var gdop = 1 / Math.max(Math.sin(sep / 2), 0.01);
  var rA = d3(st.aP, st.tg), rB = d3(st.bP, st.tg);
  var quality = gdop < 1.2 ? "OPTIMAL" : gdop < 2 ? "GOOD" : "CONVERGING";
  var qCol = gdop < 1.2 ? "#0a8a5a" : gdop < 2 ? "#c89020" : "#c06020";
  return (
    <div style={{ background: "#0a1520e0", borderRadius: 5, padding: "4px 6px", border: "1px solid #c0602025", fontSize: 8, fontFamily: "monospace", color: "#8ab0c8", lineHeight: "13px" }}>
      <div style={{ fontWeight: 700, color: "#c06020", marginBottom: 2, fontSize: 7, letterSpacing: 0.5 }}>TRIANG OPT</div>
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
        <span>SEP <span style={{ color: "#e0f0ff" }}>{sepDeg.toFixed(0)}°/180°</span></span>
        <span>GDOP <span style={{ color: qCol, fontWeight: 700 }}>{gdop.toFixed(1)}</span></span>
        <span>A:<span style={{ color: "#e0f0ff" }}>{rA.toFixed(0)}m</span></span>
        <span>B:<span style={{ color: "#e0f0ff" }}>{rB.toFixed(0)}m</span></span>
      </div>
      <svg width="80" height="35" style={{ marginTop: 2 }}>
        <circle cx="40" cy="20" r="2" fill="#d03030" />
        <line x1="40" y1="20" x2={40 + 18 * Math.cos(angA)} y2={20 - 18 * Math.sin(angA)} stroke="#0a8a5a" strokeWidth="1.5" />
        <line x1="40" y1="20" x2={40 + 18 * Math.cos(angB)} y2={20 - 18 * Math.sin(angB)} stroke="#4838d0" strokeWidth="1.5" />
        <text x="40" y="8" textAnchor="middle" style={{ fontSize: 7, fill: qCol, fontWeight: 700 }}>{sepDeg.toFixed(0)}°</text>
      </svg>
    </div>
  );
}

// ════════════════════════════════════════════════════
// COMM DASHBOARD
// ════════════════════════════════════════════════════
function CommDashboard({ comm, proto }) {
  var pc = PROTOCOLS[proto];
  var pCol = pc.color;
  function sigBar(val, color) {
    return <div style={{ display: "inline-flex", gap: 1, verticalAlign: "middle", marginLeft: 3 }}>
      {[0,1,2,3,4].map(function(i) { return <div key={i} style={{ width: 3, height: 4 + i * 1.5, background: val > (i / 5) ? color : "#333", borderRadius: 1 }} />; })}
    </div>;
  }
  return (
    <div style={{ background: "#0a1520e0", borderRadius: 5, padding: "4px 6px", border: "1px solid " + pCol + "25", fontSize: 8, fontFamily: "monospace", color: "#8ab0c8", lineHeight: "13px" }}>
      <div style={{ fontWeight: 700, color: pCol, marginBottom: 2, fontSize: 7, letterSpacing: 0.5 }}>COMM — {pc.id}</div>
      <div style={{ display: "flex", gap: 8, flexWrap: "wrap" }}>
        <span>{comm.route}</span>
        <span style={{ color: comm.latency < 30 ? "#20d090" : "#c89020" }}>{comm.hasComm ? comm.latency.toFixed(0) + "ms" : "---"}</span>
        <span>{comm.hasComm ? comm.bw.toFixed(1) + "Mbps" : "---"}</span>
      </div>
      <div style={{ display: "flex", gap: 6, marginTop: 2 }}>
        {proto === "D2D" && <span>A↔B {sigBar(comm.sigAB, "#20d090")}</span>}
        {proto === "D2G" && <><span>A↔GS {sigBar(comm.sigAG, "#f0a030")}</span><span>B↔GS {sigBar(comm.sigBG, "#f0a030")}</span></>}
      </div>
      <svg width="100" height="22" style={{ marginTop: 2 }}>
        <circle cx="10" cy="11" r="5" fill={comm.hasComm ? "#0a8a5a" : "#333"} /><text x="10" y="14" textAnchor="middle" style={{ fontSize: 6, fill: "#fff", fontWeight: 700 }}>A</text>
        <circle cx="90" cy="11" r="5" fill={comm.hasComm ? "#4838d0" : "#333"} /><text x="90" y="14" textAnchor="middle" style={{ fontSize: 6, fill: "#fff", fontWeight: 700 }}>B</text>
        {proto === "D2G" && <><rect x="43" y="5" width="14" height="12" rx="2" fill={comm.hasComm ? "#f0a030" : "#333"} /><text x="50" y="14" textAnchor="middle" style={{ fontSize: 5, fill: "#fff" }}>GS</text><line x1="15" y1="11" x2="43" y2="11" stroke={comm.hasComm ? "#f0a030" : "#333"} strokeWidth="1" strokeDasharray="2 2" /><line x1="57" y1="11" x2="85" y2="11" stroke={comm.hasComm ? "#f0a030" : "#333"} strokeWidth="1" strokeDasharray="2 2" /></>}
        {proto === "D2D" && <line x1="15" y1="11" x2="85" y2="11" stroke={comm.hasComm ? "#20d090" : "#333"} strokeWidth="1.5" />}
      </svg>
    </div>
  );
}

// ── MAIN ──
export default function App() {
  var mountRef = useRef(null);
  var fpvARef = useRef(null);
  var fpvBRef = useRef(null);
  var fpvBigRef = useRef(null);
  var initDone = useRef(false);
  var [commProto, setCommProto] = useState("D2G");
  var [mapFull, setMapFull] = useState(false);
  var [running, setRunning] = useState(false);
  var simMode = "manual";
  var runningRef = useRef(false);
  var simModeRef = useRef("manual");
  var [expandedFpv, setExpandedFpv] = useState(null);
  var [mapWidth, setMapWidth] = useState(35);
  var draggingDivider = useRef(false);
  var containerRef = useRef(null);

  // V6: Real-time simulation state
  var simRef = useRef(initialSim());
  var keysRef = useRef({ up: false, down: false, left: false, right: false });
  var [renderState, setRenderState] = useState(function() { return simSnapshot(simRef.current); });
  var simStateRef = useRef(renderState); // for 3D animate loop to read

  var protoRef = useRef(commProto);
  protoRef.current = commProto;
  runningRef.current = running;
  // v6 is always manual mode
  if (mountRef.current && mountRef.current._expandedRef) {
    mountRef.current._expandedRef.current = expandedFpv;
  }

  // Keyboard input for target control
  useEffect(function() {
    function onDown(e) {
      if (e.key === "ArrowUp" || e.key === "w" || e.key === "W") keysRef.current.up = true;
      if (e.key === "ArrowDown" || e.key === "s" || e.key === "S") keysRef.current.down = true;
      if (e.key === "ArrowLeft" || e.key === "a" || e.key === "A") keysRef.current.left = true;
      if (e.key === "ArrowRight" || e.key === "d" || e.key === "D") keysRef.current.right = true;
    }
    function onUp(e) {
      if (e.key === "ArrowUp" || e.key === "w" || e.key === "W") keysRef.current.up = false;
      if (e.key === "ArrowDown" || e.key === "s" || e.key === "S") keysRef.current.down = false;
      if (e.key === "ArrowLeft" || e.key === "a" || e.key === "A") keysRef.current.left = false;
      if (e.key === "ArrowRight" || e.key === "d" || e.key === "D") keysRef.current.right = false;
    }
    window.addEventListener("keydown", onDown);
    window.addEventListener("keyup", onUp);
    return function() { window.removeEventListener("keydown", onDown); window.removeEventListener("keyup", onUp); };
  }, []);

  // Real-time simulation loop (fixed timestep)
  useEffect(function() {
    var lastT = performance.now();
    var accum = 0;
    var SIM_DT = 1 / 60;
    var frameCount = 0;
    function loop(now) {
      var frameMs = Math.min(now - lastT, 100);
      lastT = now;
      if (runningRef.current && !simRef.current.blasted) {
        accum += frameMs / 1000;
        while (accum >= SIM_DT) {
          simRef.current.mode = "manual";
          stepSim(simRef.current, keysRef.current, SIM_DT);
          accum -= SIM_DT;
        }
      }
      frameCount++;
      if (frameCount % 2 === 0) { // update React state at 30fps
        var snap = simSnapshot(simRef.current);
        simStateRef.current = snap;
        setRenderState(snap);
      } else {
        simStateRef.current = simSnapshot(simRef.current);
      }
      requestAnimationFrame(loop);
    }
    requestAnimationFrame(loop);
  }, []);

  // 3D Scene - one-time init
  useEffect(function() {
    if (initDone.current) return;
    var el = mountRef.current;
    if (!el) return;
    initDone.current = true;

    var w = el.clientWidth, h = el.clientHeight;
    var renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setSize(w, h);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setClearColor(0x87ceeb);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.1;
    el.appendChild(renderer.domElement);
    renderer.domElement.style.width = "100%";
    renderer.domElement.style.height = "100%";
    renderer.domElement.style.display = "block";

    var scene = new THREE.Scene();
    scene.fog = new THREE.FogExp2(0x8abcd4, 0.0025);

    // Sky dome
    var skyGeo = new THREE.SphereGeometry(400, 32, 15);
    var skyMat = new THREE.ShaderMaterial({
      side: THREE.BackSide,
      uniforms: {
        topColor: { value: new THREE.Color(0x3a7ec5) },
        bottomColor: { value: new THREE.Color(0xc8e4f8) },
        offset: { value: 20 },
        exponent: { value: 0.4 }
      },
      vertexShader: "varying vec3 vWorldPosition; void main() { vec4 wp = modelMatrix * vec4(position, 1.0); vWorldPosition = wp.xyz; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }",
      fragmentShader: "uniform vec3 topColor; uniform vec3 bottomColor; uniform float offset; uniform float exponent; varying vec3 vWorldPosition; void main() { float h = normalize(vWorldPosition + offset).y; gl_FragColor = vec4(mix(bottomColor, topColor, max(pow(max(h, 0.0), exponent), 0.0)), 1.0); }"
    });
    scene.add(new THREE.Mesh(skyGeo, skyMat));

    var camera = new THREE.PerspectiveCamera(45, w / h, .5, 800);

    // Lighting — hemisphere + directional with shadows + fill
    var hemiLight = new THREE.HemisphereLight(0x87ceeb, 0x3a6b8a, 0.6);
    scene.add(hemiLight);
    var sunLight = new THREE.DirectionalLight(0xfff4e0, 1.8);
    sunLight.position.set(60, 80, 40);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 1024;
    sunLight.shadow.mapSize.height = 1024;
    sunLight.shadow.camera.near = 1;
    sunLight.shadow.camera.far = 250;
    sunLight.shadow.camera.left = -100;
    sunLight.shadow.camera.right = 100;
    sunLight.shadow.camera.top = 100;
    sunLight.shadow.camera.bottom = -100;
    scene.add(sunLight);
    var fillLight = new THREE.DirectionalLight(0x8ab4d0, 0.4);
    fillLight.position.set(-30, 20, -40);
    scene.add(fillLight);

    // Ocean — larger, deeper color, more detail for waves
    var ocean = new THREE.Mesh(
      new THREE.PlaneGeometry(500, 500, 128, 128),
      new THREE.MeshStandardMaterial({
        color: 0x1a6090,
        roughness: 0.3,
        metalness: 0.6,
        transparent: true,
        opacity: 0.92,
        envMapIntensity: 0.8
      })
    );
    ocean.rotation.x = -Math.PI / 2;
    ocean.receiveShadow = true;
    scene.add(ocean);

    // Second ocean layer for depth effect
    var oceanDeep = new THREE.Mesh(
      new THREE.PlaneGeometry(500, 500),
      new THREE.MeshStandardMaterial({ color: 0x0a3050, roughness: 1, metalness: 0 })
    );
    oceanDeep.rotation.x = -Math.PI / 2;
    oceanDeep.position.y = -1.5;
    scene.add(oceanDeep);

    // Subtle grid on the water surface
    var grid = new THREE.GridHelper(200, 40, 0x3a8ab0, 0x3a8ab0);
    grid.position.y = .06; grid.material.opacity = .06; grid.material.transparent = true;
    scene.add(grid);

    // Wake/foam particle system
    var wakeCount = 200;
    var wakeGeo = new THREE.BufferGeometry();
    var wakePos = new Float32Array(wakeCount * 3);
    var wakeAlpha = new Float32Array(wakeCount);
    for (var wi = 0; wi < wakeCount; wi++) { wakePos[wi * 3 + 1] = -10; wakeAlpha[wi] = 0; }
    wakeGeo.setAttribute("position", new THREE.BufferAttribute(wakePos, 3));
    var wakeMat = new THREE.PointsMaterial({ color: 0xffffff, size: 0.6, transparent: true, opacity: 0.4, sizeAttenuation: true });
    var wakeParticles = new THREE.Points(wakeGeo, wakeMat);
    wakeParticles.frustumCulled = false;
    scene.add(wakeParticles);
    var wakeIdx = 0;

    function mkBoat(hc, cc, ac) {
      var g = new THREE.Group();
      var hullMat = new THREE.MeshStandardMaterial({ color: hc, roughness: .25, metalness: .6 });
      // Main hull
      var hull = new THREE.Mesh(new THREE.BoxGeometry(3.8, .9, 1.5), hullMat);
      hull.position.y = .3; hull.castShadow = true; hull.receiveShadow = true; g.add(hull);
      // Bow point
      var bow = new THREE.Mesh(new THREE.CylinderGeometry(0, .75, 1.4, 4), hullMat);
      bow.rotation.z = Math.PI / 2; bow.position.set(2.3, .3, 0); bow.castShadow = true; g.add(bow);
      // Hull keel (dark underside)
      var keel = new THREE.Mesh(new THREE.BoxGeometry(3.4, .3, 1.1), new THREE.MeshStandardMaterial({ color: 0x1a2a3a, roughness: .5 }));
      keel.position.y = -.1; g.add(keel);
      // Cabin with windows
      var cabinMat = new THREE.MeshStandardMaterial({ color: cc, roughness: .2, metalness: .7 });
      var cabin = new THREE.Mesh(new THREE.BoxGeometry(1.6, .9, 1.1), cabinMat);
      cabin.position.set(-.3, 1.15, 0); cabin.castShadow = true; g.add(cabin);
      // Cabin roof
      var roof = new THREE.Mesh(new THREE.BoxGeometry(1.8, .1, 1.3), new THREE.MeshStandardMaterial({ color: cc, roughness: .15, metalness: .8 }));
      roof.position.set(-.3, 1.65, 0); g.add(roof);
      // Window strip
      var winMat = new THREE.MeshBasicMaterial({ color: 0xaaddff, transparent: true, opacity: .7 });
      var win = new THREE.Mesh(new THREE.BoxGeometry(1.4, .3, 1.12), winMat);
      win.position.set(-.3, 1.3, 0); g.add(win);
      // Antenna mast
      var pole = new THREE.Mesh(new THREE.CylinderGeometry(.03, .05, 2.2, 8), new THREE.MeshStandardMaterial({ color: 0x556677, metalness: .8, roughness: .2 }));
      pole.position.set(.8, 1.9, 0); pole.castShadow = true; g.add(pole);
      // Antenna tip (blinking light)
      var tip = new THREE.Mesh(new THREE.SphereGeometry(.14, 8, 8), new THREE.MeshBasicMaterial({ color: ac }));
      tip.position.set(.8, 3.05, 0); g.add(tip);
      g.userData.tip = tip;
      // Flag
      var flag = new THREE.Mesh(new THREE.PlaneGeometry(.55, .35), new THREE.MeshBasicMaterial({ color: ac, side: THREE.DoubleSide }));
      flag.position.set(1.15, 2.8, 0); g.add(flag);
      g.userData.flag = flag;
      // Stern detail (engine housing)
      var stern = new THREE.Mesh(new THREE.BoxGeometry(.8, .5, .9), new THREE.MeshStandardMaterial({ color: 0x2a3a4a, roughness: .4, metalness: .6 }));
      stern.position.set(-2.0, .35, 0); g.add(stern);
      // Sonar detection rings
      var rings = [];
      for (var r = 0; r < 3; r++) {
        var ri = new THREE.Mesh(new THREE.RingGeometry(2.8 + r * 1.4, 3.0 + r * 1.4, 48), new THREE.MeshBasicMaterial({ color: ac, transparent: true, opacity: .18 - r * .05, side: THREE.DoubleSide }));
        ri.rotation.x = -Math.PI / 2; ri.position.y = .08; ri.visible = false; g.add(ri); rings.push(ri);
      }
      g.userData.rings = rings;
      return g;
    }

    var dA = mkBoat(0x18a070, 0x22c888, 0x0a8a5a); scene.add(dA);
    var dB = mkBoat(0x3030b0, 0x4848d8, 0x4838d0); scene.add(dB);
    var tgt = mkBoat(0xc03030, 0xe04040, 0xd03030); scene.add(tgt);

    // 3D Playground boundary (red dashed rectangle on water)
    var bw = BOUNDS.xMax - BOUNDS.xMin, bh = BOUNDS.zMax - BOUNDS.zMin;
    var bcx = (BOUNDS.xMin + BOUNDS.xMax) / 2, bcz = (BOUNDS.zMin + BOUNDS.zMax) / 2;
    var boundaryGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(BOUNDS.xMin, 0.2, BOUNDS.zMin),
      new THREE.Vector3(BOUNDS.xMax, 0.2, BOUNDS.zMin),
      new THREE.Vector3(BOUNDS.xMax, 0.2, BOUNDS.zMax),
      new THREE.Vector3(BOUNDS.xMin, 0.2, BOUNDS.zMax),
      new THREE.Vector3(BOUNDS.xMin, 0.2, BOUNDS.zMin)
    ]);
    var boundaryLine = new THREE.Line(boundaryGeo, new THREE.LineDashedMaterial({ color: 0xc03030, transparent: true, opacity: 0.25, dashSize: 3, gapSize: 2 }));
    boundaryLine.computeLineDistances();
    scene.add(boundaryLine);

    // Ground Station 3D model
    var gsGroup = new THREE.Group();
    var gsPlatform = new THREE.Mesh(new THREE.BoxGeometry(4, 0.8, 4), new THREE.MeshStandardMaterial({ color: 0x2a3a4a, roughness: 0.4, metalness: 0.6 }));
    gsPlatform.position.y = 0.4; gsPlatform.castShadow = true; gsGroup.add(gsPlatform);
    var gsMast = new THREE.Mesh(new THREE.CylinderGeometry(0.06, 0.08, 4, 8), new THREE.MeshStandardMaterial({ color: 0x888888, metalness: 0.9, roughness: 0.2 }));
    gsMast.position.y = 2.8; gsGroup.add(gsMast);
    var gsDish = new THREE.Mesh(new THREE.ConeGeometry(1.0, 0.6, 16), new THREE.MeshStandardMaterial({ color: 0xf0a030, metalness: 0.7, roughness: 0.3 }));
    gsDish.position.y = 4.2; gsDish.rotation.x = Math.PI; gsGroup.add(gsDish);
    var gsBeacon = new THREE.Mesh(new THREE.SphereGeometry(0.2, 8, 8), new THREE.MeshBasicMaterial({ color: 0xf0a030, transparent: true }));
    gsBeacon.position.y = 5.0; gsGroup.add(gsBeacon);
    gsGroup.position.set(GS_POS.x, 0, GS_POS.z);
    scene.add(gsGroup);

    // D2G relay beam objects (TX→GS and GS→RX)
    var d2gBeam1 = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineDashedMaterial({ color: 0xf0a030, transparent: true, opacity: .4, dashSize: 1, gapSize: 1.5 }));
    d2gBeam1.visible = false; scene.add(d2gBeam1);
    var d2gBeam2 = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineDashedMaterial({ color: 0xf0a030, transparent: true, opacity: .4, dashSize: 1, gapSize: 1.5 }));
    d2gBeam2.visible = false; scene.add(d2gBeam2);
    var d2gPkts = [];
    for (var dp = 0; dp < 6; dp++) { var dpm = new THREE.Mesh(new THREE.OctahedronGeometry(0.3), new THREE.MeshBasicMaterial({ color: 0xf0a030, transparent: true, opacity: 0.9 })); dpm.visible = false; scene.add(dpm); d2gPkts.push(dpm); }
    var gsRelayRings = [];
    for (var gr = 0; gr < 3; gr++) { var grm = new THREE.Mesh(new THREE.RingGeometry(1, 1.3, 32), new THREE.MeshBasicMaterial({ color: 0xf0a030, transparent: true, opacity: .3, side: THREE.DoubleSide })); grm.rotation.x = -Math.PI / 2; grm.visible = false; scene.add(grm); gsRelayRings.push(grm); }

    // Explosion effects
    var blastSphere = new THREE.Mesh(new THREE.SphereGeometry(1, 16, 16), new THREE.MeshBasicMaterial({ color: 0xff4400, transparent: true, opacity: 0.9 }));
    blastSphere.visible = false; scene.add(blastSphere);
    var blastRings = [];
    for (var br = 0; br < 5; br++) {
      var brm = new THREE.Mesh(new THREE.RingGeometry(0.5, 1, 32), new THREE.MeshBasicMaterial({ color: br < 2 ? 0xff6600 : 0xff2200, transparent: true, opacity: 0.7, side: THREE.DoubleSide }));
      brm.rotation.x = -Math.PI / 2; brm.visible = false; scene.add(brm);
      blastRings.push(brm);
    }
    var blastFlash = new THREE.PointLight(0xff4400, 0, 100);
    scene.add(blastFlash);

    function mkLine(c, o) {
      var l = new THREE.Line(new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]), new THREE.LineBasicMaterial({ color: c, transparent: true, opacity: o }));
      l.visible = false; scene.add(l); return l;
    }
    var bearA = mkLine(0xc89020, .5);
    var bearB = mkLine(0xc06020, .5);
    var orbRing = new THREE.Mesh(new THREE.RingGeometry(OR - .15, OR + .15, 64), new THREE.MeshBasicMaterial({ color: 0x0a8a5a, transparent: true, opacity: .1, side: THREE.DoubleSide }));
    orbRing.rotation.x = -Math.PI / 2; orbRing.visible = false; scene.add(orbRing);
    var diaLine = mkLine(0x0a8a5a, .12);
    var idleRing = new THREE.Mesh(new THREE.RingGeometry(3.5, 4, 48), new THREE.MeshBasicMaterial({ color: 0x0a8a5a, transparent: true, opacity: .2, side: THREE.DoubleSide }));
    idleRing.rotation.x = -Math.PI / 2; idleRing.visible = false; scene.add(idleRing);

    // Packets (target-tracking data)
    var pkt1 = new THREE.Mesh(new THREE.SphereGeometry(.3, 6, 6), new THREE.MeshBasicMaterial({ color: 0x0a8a5a })); pkt1.visible = false; scene.add(pkt1);
    var pkt2 = new THREE.Mesh(new THREE.SphereGeometry(.3, 6, 6), new THREE.MeshBasicMaterial({ color: 0x4838d0 })); pkt2.visible = false; scene.add(pkt2);

    // Comm beam line A→B
    var commBeam = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineDashedMaterial({ color: 0x20d090, transparent: true, opacity: .4, dashSize: 1.5, gapSize: 1 })
    );
    commBeam.visible = false; scene.add(commBeam);

    // Multiple comm signal packets A→B (4 staggered spheres)
    var commPkts = [];
    for (var cp = 0; cp < 4; cp++) {
      var cpMat = new THREE.MeshBasicMaterial({ color: 0x20d090, transparent: true, opacity: .9 });
      var cpMesh = new THREE.Mesh(new THREE.SphereGeometry(.35, 8, 8), cpMat);
      cpMesh.visible = false; scene.add(cpMesh);
      commPkts.push(cpMesh);
    }

    // Signal pulse rings at A (expanding rings when transmitting)
    var sigRings = [];
    for (var sr = 0; sr < 3; sr++) {
      var srMesh = new THREE.Mesh(
        new THREE.RingGeometry(1, 1.3, 32),
        new THREE.MeshBasicMaterial({ color: 0x20d090, transparent: true, opacity: .3, side: THREE.DoubleSide })
      );
      srMesh.rotation.x = -Math.PI / 2; srMesh.visible = false; scene.add(srMesh);
      sigRings.push(srMesh);
    }

    // Receive pulse ring at B
    var recvRing = new THREE.Mesh(
      new THREE.RingGeometry(1, 1.3, 32),
      new THREE.MeshBasicMaterial({ color: 0x4838d0, transparent: true, opacity: .3, side: THREE.DoubleSide })
    );
    recvRing.rotation.x = -Math.PI / 2; recvRing.visible = false; scene.add(recvRing);

    // 3D FOV cone — dynamic vertices updated each frame (matches 2D map exactly)
    var fovSegs = 24;
    function mkDynFov(color) {
      // Fill mesh: tip + arc points
      var fillVerts = new Float32Array((fovSegs + 2) * 3);
      var fillIdx = [];
      for (var fi = 1; fi <= fovSegs; fi++) fillIdx.push(0, fi, fi + 1);
      var fillGeo = new THREE.BufferGeometry();
      fillGeo.setAttribute("position", new THREE.BufferAttribute(fillVerts, 3));
      fillGeo.setIndex(fillIdx);
      var fillMat = new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: .1, side: THREE.DoubleSide });
      var fillMesh = new THREE.Mesh(fillGeo, fillMat);
      fillMesh.frustumCulled = false;
      // Edge line: tip → arc → tip
      var edgeVerts = new Float32Array((fovSegs + 3) * 3);
      var edgeGeo = new THREE.BufferGeometry();
      edgeGeo.setAttribute("position", new THREE.BufferAttribute(edgeVerts, 3));
      edgeGeo.setDrawRange(0, fovSegs + 3);
      var edgeMat = new THREE.LineBasicMaterial({ color: color, transparent: true, opacity: .25 });
      var edgeLine = new THREE.Line(edgeGeo, edgeMat);
      edgeLine.frustumCulled = false;
      scene.add(fillMesh); scene.add(edgeLine);
      return { fillMesh: fillMesh, fillGeo: fillGeo, fillMat: fillMat, edgeLine: edgeLine, edgeGeo: edgeGeo, edgeMat: edgeMat };
    }
    var fovA = mkDynFov(0x0a8a5a);
    var fovB = mkDynFov(0x4838d0);

    // Update FOV cone vertices from world position and heading
    function updateFovCone(fov, pos, heading, inRange, activeColor, yHeight) {
      var fp = fov.fillGeo.attributes.position.array;
      var ep = fov.edgeGeo.attributes.position.array;
      // Tip at drone position
      fp[0] = pos.x; fp[1] = yHeight; fp[2] = pos.z;
      ep[0] = pos.x; ep[1] = yHeight + 0.02; ep[2] = pos.z;
      // Arc points — use same math as 2D fovTriangle
      for (var i = 0; i <= fovSegs; i++) {
        var a = heading + (-FOV_HALF + (i / fovSegs) * FOV_ANGLE);
        var px = pos.x + SENSE_R * Math.cos(a);
        var pz = pos.z + SENSE_R * Math.sin(a);
        var idx = (i + 1) * 3;
        fp[idx] = px; fp[idx + 1] = yHeight; fp[idx + 2] = pz;
        var eidx = (i + 1) * 3;
        ep[eidx] = px; ep[eidx + 1] = yHeight + 0.02; ep[eidx + 2] = pz;
      }
      // Close edge back to tip
      var lastIdx = (fovSegs + 2) * 3;
      ep[lastIdx] = pos.x; ep[lastIdx + 1] = yHeight + 0.02; ep[lastIdx + 2] = pos.z;
      fov.fillGeo.attributes.position.needsUpdate = true;
      fov.edgeGeo.attributes.position.needsUpdate = true;
      fov.fillMat.opacity = inRange ? .14 : .04;
      fov.fillMat.color.setHex(inRange ? activeColor : 0x6a8a9a);
      fov.edgeMat.opacity = inRange ? .4 : .12;
      fov.edgeMat.color.setHex(inRange ? activeColor : 0x6a8a9a);
    }

    // 3D lock-on indicators around target
    var lockRingA = new THREE.Mesh(
      new THREE.RingGeometry(3.5, 4.0, 32),
      new THREE.MeshBasicMaterial({ color: 0x0a8a5a, transparent: true, opacity: .4, side: THREE.DoubleSide })
    );
    lockRingA.rotation.x = -Math.PI / 2; lockRingA.visible = false; scene.add(lockRingA);
    var lockRingB = new THREE.Mesh(
      new THREE.RingGeometry(4.5, 5.0, 32),
      new THREE.MeshBasicMaterial({ color: 0x4838d0, transparent: true, opacity: .4, side: THREE.DoubleSide })
    );
    lockRingB.rotation.x = -Math.PI / 2; lockRingB.visible = false; scene.add(lockRingB);
    var lockBeamA = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0x0a8a5a, transparent: true, opacity: .6 })
    );
    lockBeamA.visible = false; scene.add(lockBeamA);
    var lockBeamB = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
      new THREE.LineBasicMaterial({ color: 0x4838d0, transparent: true, opacity: .6 })
    );
    lockBeamB.visible = false; scene.add(lockBeamB);

    // Resize handler
    function onResize() {
      var rw = el.clientWidth, rh = el.clientHeight;
      renderer.setSize(rw, rh);
      camera.aspect = rw / rh;
      camera.updateProjectionMatrix();
    }
    window.addEventListener("resize", onResize);

    // Camera controls
    var camTh = Math.PI * .22, camPh = Math.PI * .3, camD = 75;
    var dragging = false, dragX = 0, dragY = 0;
    var cv = renderer.domElement;
    cv.addEventListener("mousedown", function(e) { dragging = true; dragX = e.clientX; dragY = e.clientY; });
    cv.addEventListener("mousemove", function(e) { if (!dragging) return; camTh -= (e.clientX - dragX) * .005; camPh = Math.max(.08, Math.min(1.5, camPh + (e.clientY - dragY) * .005)); dragX = e.clientX; dragY = e.clientY; });
    cv.addEventListener("mouseup", function() { dragging = false; });
    cv.addEventListener("mouseleave", function() { dragging = false; });
    cv.addEventListener("wheel", function(e) { camD = Math.max(28, Math.min(150, camD + e.deltaY * .05)); e.preventDefault(); }, { passive: false });
    cv.addEventListener("touchstart", function(e) { if (e.touches.length === 1) { dragging = true; dragX = e.touches[0].clientX; dragY = e.touches[0].clientY; } });
    cv.addEventListener("touchmove", function(e) { if (!dragging) return; camTh -= (e.touches[0].clientX - dragX) * .005; camPh = Math.max(.08, Math.min(1.5, camPh + (e.touches[0].clientY - dragY) * .005)); dragX = e.touches[0].clientX; dragY = e.touches[0].clientY; });
    cv.addEventListener("touchend", function() { dragging = false; });

    // 3D trails
    function mkTrail(c) {
      var max = 1400, positions = new Float32Array(max * 3);
      var geo = new THREE.BufferGeometry();
      geo.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      geo.setDrawRange(0, 0);
      var line = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: c, transparent: true, opacity: .3 }));
      line.frustumCulled = false; scene.add(line);
      return { line: line, pos: positions, count: 0, max: max };
    }
    var trA = mkTrail(0x0a8a5a), trB = mkTrail(0x4838d0), trT = mkTrail(0xd03030);

    function animate() {
      requestAnimationFrame(animate);
      var st = simStateRef.current;
      var stopped = false; // never stops in v6 real-time mode
      var time = Date.now() * .001;

      // Realistic ocean waves — multiple overlapping sine waves
      var wPos = ocean.geometry.attributes.position;
      for (var i = 0; i < wPos.count; i++) {
        var wx = wPos.getX(i), wy = wPos.getY(i);
        var wave1 = Math.sin(wx * 0.08 + time * 0.6) * 0.35;
        var wave2 = Math.cos(wy * 0.06 + time * 0.4) * 0.25;
        var wave3 = Math.sin((wx + wy) * 0.12 + time * 0.9) * 0.15;
        var wave4 = Math.cos(wx * 0.15 - time * 0.3) * 0.1;
        wPos.setZ(i, wave1 + wave2 + wave3 + wave4);
      }
      wPos.needsUpdate = true;
      ocean.geometry.computeVertexNormals();

      // Boats bob and roll with the waves
      var bob = function(o) { return Math.sin(time * 1.8 + o) * .18 + Math.cos(time * 1.2 + o * 2) * .08 + .5; };
      var rock = function(o) { return Math.sin(time * 1.6 + o) * .05 + Math.cos(time * 2.3 + o) * .025; };
      var pitch = function(o) { return Math.sin(time * 1.3 + o) * .03; };

      // Boat rotation: heading = atan2(dz, dx) is angle from +X in world XZ plane
      // Three.js rotation.y: after rotating by θ, +X direction becomes (cosθ, 0, -sinθ)
      // We want the bow (+X local) to face (cos(heading), 0, sin(heading)) in world
      // So: cosθ = cos(h) and -sinθ = sin(h) → θ = -heading
      dA.position.set(st.aP.x, bob(0), st.aP.z);
      dA.rotation.y = st.aHeading != null ? -st.aHeading : 0;
      dA.rotation.z = rock(0);
      dA.rotation.x = pitch(0);
      dA.userData.rings.forEach(function(r, i) { r.visible = st.aD; if (st.aD) r.material.opacity = (.18 - i * .04) * (.7 + .3 * Math.sin(time * 2 + i)); });

      dB.position.set(st.bP.x, bob(1.5), st.bP.z);
      dB.rotation.y = st.bHeading != null ? -st.bHeading : 0;
      dB.rotation.z = rock(1.5);
      dB.rotation.x = pitch(1.5);
      dB.userData.rings.forEach(function(r, i) { r.visible = st.bD; if (st.bD) r.material.opacity = (.18 - i * .04) * (.7 + .3 * Math.sin(time * 2 + i + 1)); });

      tgt.position.set(st.tg.x, bob(3), st.tg.z);
      var fleeing = st.pi >= 2 && !stopped;
      tgt.rotation.z = fleeing ? Math.sin(time * 3.5 + 3) * .1 : rock(3);
      tgt.rotation.x = fleeing ? Math.sin(time * 2.8) * .06 : pitch(3);
      tgt.rotation.y = -simRef.current.target.heading;

      // Blast animation
      var curSim = simRef.current;
      if (curSim.blasted) {
        var bt = time - (curSim.blastTime ? curSim.blastTime * 0.001 : time); // use real time offset
        var blastAge = curSim.elapsed - curSim.blastTime;
        // Hide drones and target after blast
        if (blastAge > 0.5) { dA.visible = false; dB.visible = false; tgt.visible = false; }
        // Fireball
        blastSphere.visible = true;
        blastSphere.position.set(st.tg.x, 2 + blastAge * 3, st.tg.z);
        var bScale = Math.min(blastAge * 15, 12) * (1 - Math.min(blastAge / 4, 1) * 0.5);
        blastSphere.scale.set(bScale, bScale, bScale);
        blastSphere.material.opacity = Math.max(0, 1 - blastAge / 3);
        blastSphere.material.color.setHex(blastAge < 0.5 ? 0xffff00 : blastAge < 1.5 ? 0xff6600 : 0x333333);
        // Expanding shockwave rings
        for (var bri = 0; bri < blastRings.length; bri++) {
          blastRings[bri].visible = true;
          blastRings[bri].position.set(st.tg.x, 0.3 + bri * 0.5, st.tg.z);
          var ringAge = Math.max(0, blastAge - bri * 0.15);
          var ringScale = ringAge * 20 + bri * 3;
          blastRings[bri].scale.set(ringScale, ringScale, ringScale);
          blastRings[bri].material.opacity = Math.max(0, 0.7 - ringAge * 0.3);
        }
        // Flash light
        blastFlash.position.set(st.tg.x, 5, st.tg.z);
        blastFlash.intensity = Math.max(0, 50 - blastAge * 25);
      } else {
        blastSphere.visible = false;
        for (var bri2 = 0; bri2 < blastRings.length; bri2++) blastRings[bri2].visible = false;
        blastFlash.intensity = 0;
        dA.visible = true; dB.visible = true; tgt.visible = true;
      }

      // 3D FOV cones — rebuild vertices from world coords (same math as 2D map)
      if (st.aHeading != null) updateFovCone(fovA, st.aP, st.aHeading, st.aInRange, 0x0a8a5a, 0.12);
      if (st.bHeading != null) updateFovCone(fovB, st.bP, st.bHeading, st.bInRange, 0x4838d0, 0.12);

      // Lock-on rings around target — spinning, pulsing when in range
      lockRingA.visible = st.aInRange;
      if (st.aInRange) {
        lockRingA.position.set(st.tg.x, 1.2, st.tg.z);
        lockRingA.rotation.z = time * 1.5;
        lockRingA.material.opacity = .3 + .15 * Math.sin(time * 3);
        var lsa = 1 + .15 * Math.sin(time * 2);
        lockRingA.scale.set(lsa, lsa, lsa);
      }
      lockRingB.visible = st.bInRange;
      if (st.bInRange) {
        lockRingB.position.set(st.tg.x, 1.0, st.tg.z);
        lockRingB.rotation.z = -time * 1.2;
        lockRingB.material.opacity = .3 + .15 * Math.sin(time * 3 + 1);
        var lsb = 1 + .15 * Math.sin(time * 2 + 1);
        lockRingB.scale.set(lsb, lsb, lsb);
      }

      // Lock-on beam lines (drone → target) when in sensing range
      function uLockBeam(beam, from, to, show) {
        beam.visible = show;
        if (show) {
          var lp = beam.geometry.attributes.position;
          lp.setXYZ(0, from.x, 2.8, from.z);
          lp.setXYZ(1, to.x, 1.5, to.z);
          lp.needsUpdate = true;
          beam.material.opacity = .4 + .2 * Math.sin(time * 4);
        }
      }
      uLockBeam(lockBeamA, st.aP, st.tg, st.aInRange && !st.aD);
      uLockBeam(lockBeamB, st.bP, st.tg, st.bInRange && !st.bD);

      // Blinking antenna lights
      var blink = Math.sin(time * 4) > 0.3 ? 1 : 0.15;
      var blinkB = Math.sin(time * 4 + 2) > 0.3 ? 1 : 0.15;
      if (dA.userData.tip) dA.userData.tip.material.opacity = blink;
      if (dB.userData.tip) dB.userData.tip.material.opacity = blinkB;

      // Wake foam particles behind moving boats
      if (Math.random() < 0.3) { // spawn wake particles ~30% of frames
        var boats = [st.aP, st.bP, st.tg];
        for (var bi = 0; bi < boats.length; bi++) {
          var wp = wakeIdx % wakeCount;
          wakePos[wp * 3] = boats[bi].x + (Math.random() - 0.5) * 2;
          wakePos[wp * 3 + 1] = 0.15 + Math.random() * 0.1;
          wakePos[wp * 3 + 2] = boats[bi].z + (Math.random() - 0.5) * 2;
          wakeAlpha[wp] = 1;
          wakeIdx++;
        }
      }
      // Fade wake particles
      for (var fwi = 0; fwi < wakeCount; fwi++) {
        wakeAlpha[fwi] *= 0.98;
        if (wakeAlpha[fwi] < 0.01) wakePos[fwi * 3 + 1] = -10;
      }
      wakeGeo.attributes.position.needsUpdate = true;

      // Bearing lines
      function uLine(l, from, to, show) {
        if (show) {
          var p = l.geometry.attributes.position;
          p.setXYZ(0, from.x, 1.5, from.z);
          p.setXYZ(1, to.x, 1, to.z);
          p.needsUpdate = true;
          l.visible = true;
        } else { l.visible = false; }
      }
      uLine(bearA, st.aP, st.tg, st.aD);
      uLine(bearB, st.bP, st.tg, st.bD);

      orbRing.visible = st.pi >= 5;
      if (st.pi >= 5) orbRing.position.set(st.tg.x, .2, st.tg.z);
      diaLine.visible = st.pi >= 5;
      if (st.pi >= 5) { var dp = diaLine.geometry.attributes.position; dp.setXYZ(0, st.aP.x, .5, st.aP.z); dp.setXYZ(1, st.bP.x, .5, st.bP.z); dp.needsUpdate = true; }
      idleRing.visible = stopped;
      if (stopped) { idleRing.position.set(st.tg.x, .1, st.tg.z); idleRing.material.opacity = .15 + .08 * Math.sin(time * 1.5); }

      // Packets (bearing data once both tracking)
      var pp = (time * 2) % 1;
      if (st.cm && st.aD && !st.commAB) {
        var toX = st.bD ? st.bP.x : st.tg.x, toZ = st.bD ? st.bP.z : st.tg.z;
        pkt1.visible = true;
        pkt1.position.set(lr(st.aP.x, toX, pp), 2.2, lr(st.aP.z, toZ, pp));
      } else { pkt1.visible = false; }
      if (st.cm && st.bD && !st.commAB) {
        pkt2.visible = true;
        pkt2.position.set(lr(st.bP.x, st.aP.x, (pp + .5) % 1), 2.2, lr(st.bP.z, st.aP.z, (pp + .5) % 1));
      } else { pkt2.visible = false; }

      // GS beacon blink
      gsBeacon.material.opacity = Math.sin(time * 3) > 0 ? 1 : 0.2;
      gsDish.rotation.y = time * 0.5; // slow radar rotation

      // Protocol-aware 3D comm signals
      var hasComm = st.commAB || st.commBA;
      var curProto = protoRef.current;
      var txPos = st.commAB ? st.aP : st.bP;
      var rxPos = st.commAB ? st.bP : st.aP;

      if (hasComm && curProto === "D2D") {
        // D2D: direct beam between drones (cyan)
        commBeam.visible = true;
        commBeam.material.color.setHex(0x20d090);
        var cbp = commBeam.geometry.attributes.position;
        cbp.setXYZ(0, txPos.x, 2.5, txPos.z); cbp.setXYZ(1, rxPos.x, 2.5, rxPos.z); cbp.needsUpdate = true;
        commBeam.computeLineDistances();
        commBeam.material.opacity = .25 + .15 * Math.sin(time * 3);
        for (var ci = 0; ci < commPkts.length; ci++) {
          var ct = ((time * .8 + ci * .25) % 1);
          commPkts[ci].visible = true; commPkts[ci].material.color.setHex(0x20d090);
          commPkts[ci].position.set(lr(txPos.x, rxPos.x, ct), 2.5 + Math.sin(ct * Math.PI) * 1.2, lr(txPos.z, rxPos.z, ct));
          commPkts[ci].material.opacity = .9 * Math.sin(ct * Math.PI);
          var cs = .25 + .2 * Math.sin(ct * Math.PI); commPkts[ci].scale.set(cs * 3, cs * 3, cs * 3);
        }
        for (var si = 0; si < sigRings.length; si++) {
          sigRings[si].visible = true; sigRings[si].material.color.setHex(0x20d090);
          sigRings[si].position.set(txPos.x, 2.6, txPos.z);
          var sp = ((time * 1.2 + si * .33) % 1); var sr2 = 1 + sp * 5;
          sigRings[si].scale.set(sr2, sr2, sr2); sigRings[si].material.opacity = .35 * (1 - sp);
        }
        recvRing.visible = true; recvRing.material.color.setHex(0x20d090);
        recvRing.position.set(rxPos.x, 2.0, rxPos.z);
        var rp = ((time * 1.5) % 1); var rr = 1 + rp * 3;
        recvRing.scale.set(rr, rr, rr); recvRing.material.opacity = .3 * (1 - rp);
        // Hide D2G objects
        d2gBeam1.visible = false; d2gBeam2.visible = false;
        for (var dp = 0; dp < d2gPkts.length; dp++) d2gPkts[dp].visible = false;
        for (var gri = 0; gri < gsRelayRings.length; gri++) gsRelayRings[gri].visible = false;

      } else if (hasComm && curProto === "D2G") {
        // D2G: relay through ground station (amber)
        commBeam.visible = false;
        for (var ci3 = 0; ci3 < commPkts.length; ci3++) commPkts[ci3].visible = false;
        // Leg 1: TX → GS
        d2gBeam1.visible = true;
        var l1p = d2gBeam1.geometry.attributes.position;
        l1p.setXYZ(0, txPos.x, 2.5, txPos.z); l1p.setXYZ(1, GS_POS.x, 4.5, GS_POS.z); l1p.needsUpdate = true;
        d2gBeam1.computeLineDistances(); d2gBeam1.material.opacity = .25 + .1 * Math.sin(time * 2);
        // Leg 2: GS → RX
        d2gBeam2.visible = true;
        var l2p = d2gBeam2.geometry.attributes.position;
        l2p.setXYZ(0, GS_POS.x, 4.5, GS_POS.z); l2p.setXYZ(1, rxPos.x, 2.5, rxPos.z); l2p.needsUpdate = true;
        d2gBeam2.computeLineDistances(); d2gBeam2.material.opacity = .25 + .1 * Math.sin(time * 2 + 1);
        // D2G packets (3 per leg, slower, diamond shape)
        for (var di = 0; di < 3; di++) {
          var ct1 = ((time * .5 + di * .33) % 1);
          d2gPkts[di].visible = true;
          d2gPkts[di].position.set(lr(txPos.x, GS_POS.x, ct1), lr(2.5, 4.5, ct1) + Math.sin(ct1 * Math.PI) * 0.8, lr(txPos.z, GS_POS.z, ct1));
          d2gPkts[di].material.opacity = .8 * Math.sin(ct1 * Math.PI);
          var ct2 = ((time * .5 + di * .33 + .15) % 1);
          d2gPkts[di + 3].visible = true;
          d2gPkts[di + 3].position.set(lr(GS_POS.x, rxPos.x, ct2), lr(4.5, 2.5, ct2) + Math.sin(ct2 * Math.PI) * 0.8, lr(GS_POS.z, rxPos.z, ct2));
          d2gPkts[di + 3].material.opacity = .8 * Math.sin(ct2 * Math.PI);
        }
        // GS relay pulse rings
        for (var gri2 = 0; gri2 < gsRelayRings.length; gri2++) {
          gsRelayRings[gri2].visible = true; gsRelayRings[gri2].position.set(GS_POS.x, 4.0, GS_POS.z);
          var gsp = ((time * 1.0 + gri2 * .33) % 1); var gsr = 1 + gsp * 4;
          gsRelayRings[gri2].scale.set(gsr, gsr, gsr); gsRelayRings[gri2].material.opacity = .3 * (1 - gsp);
        }
        // TX/RX signal rings (amber)
        for (var si2 = 0; si2 < sigRings.length; si2++) {
          sigRings[si2].visible = true; sigRings[si2].material.color.setHex(0xf0a030);
          sigRings[si2].position.set(txPos.x, 2.6, txPos.z);
          var sp2 = ((time * 1.2 + si2 * .33) % 1); var sr3 = 1 + sp2 * 5;
          sigRings[si2].scale.set(sr3, sr3, sr3); sigRings[si2].material.opacity = .35 * (1 - sp2);
        }
        recvRing.visible = true; recvRing.material.color.setHex(0xf0a030);
        recvRing.position.set(rxPos.x, 2.0, rxPos.z);
        var rp2 = ((time * 1.5) % 1); var rr2 = 1 + rp2 * 3;
        recvRing.scale.set(rr2, rr2, rr2); recvRing.material.opacity = .3 * (1 - rp2);

      } else {
        // No comm — hide everything
        commBeam.visible = false; d2gBeam1.visible = false; d2gBeam2.visible = false;
        for (var ci4 = 0; ci4 < commPkts.length; ci4++) commPkts[ci4].visible = false;
        for (var dp2 = 0; dp2 < d2gPkts.length; dp2++) d2gPkts[dp2].visible = false;
        for (var si3 = 0; si3 < sigRings.length; si3++) sigRings[si3].visible = false;
        recvRing.visible = false;
        for (var gri3 = 0; gri3 < gsRelayRings.length; gri3++) gsRelayRings[gri3].visible = false;
      }

      // 3D trails
      function pushT(tr, x, z) {
        if (tr.count < tr.max) {
          tr.pos[tr.count * 3] = x; tr.pos[tr.count * 3 + 1] = .12; tr.pos[tr.count * 3 + 2] = z;
          tr.count++;
          tr.line.geometry.attributes.position.needsUpdate = true;
          tr.line.geometry.setDrawRange(0, tr.count);
        }
      }
      // 3D trails — time-based (sim accumulates trails)
      var simTrailA = simRef.current.aTrail;
      var simTrailB = simRef.current.bTrail;
      var simTrailT = simRef.current.tTrail;
      if (simTrailA.length > trA.count) { for (var ti = trA.count; ti < Math.min(simTrailA.length, trA.max); ti++) pushT(trA, simTrailA[ti].x, simTrailA[ti].z); }
      if (simTrailB.length > trB.count) { for (var tj = trB.count; tj < Math.min(simTrailB.length, trB.max); tj++) pushT(trB, simTrailB[tj].x, simTrailB[tj].z); }
      if (simTrailT.length > trT.count) { for (var tk2 = trT.count; tk2 < Math.min(simTrailT.length, trT.max); tk2++) pushT(trT, simTrailT[tk2].x, simTrailT[tk2].z); }

      // Camera
      var lookX = st.tg.x * .4, lookZ = st.tg.z * .4 + 4;
      camera.position.set(lookX + camD * Math.sin(camPh) * Math.sin(camTh), camD * Math.cos(camPh), lookZ + camD * Math.sin(camPh) * Math.cos(camTh));
      camera.lookAt(lookX, 0, lookZ);
      renderer.render(scene, camera);

      // FPV camera renders — hide own boat + all comm objects for clean camera feed
      if (fpvRendererA && fpvRendererB) {
        var fpvEye = 2.5, fpvFwd = 4.0, fpvLD = 40;

        // Collect all comm objects to hide during FPV
        var commObjs = [commBeam, d2gBeam1, d2gBeam2, recvRing].concat(commPkts, d2gPkts, sigRings, gsRelayRings);
        var commVis = commObjs.map(function(o) { return o.visible; });
        commObjs.forEach(function(o) { o.visible = false; });

        // Drone A FPV
        var aH = st.aHeading || 0;
        dA.visible = false;
        fpvCamA.position.set(st.aP.x + fpvFwd * Math.cos(aH), fpvEye, st.aP.z + fpvFwd * Math.sin(aH));
        fpvCamA.lookAt(st.aP.x + fpvLD * Math.cos(aH), 0.5, st.aP.z + fpvLD * Math.sin(aH));
        fpvRendererA.render(scene, fpvCamA);
        dA.visible = true;

        // Drone B FPV
        var bH = st.bHeading || 0;
        dB.visible = false;
        fpvCamB.position.set(st.bP.x + fpvFwd * Math.cos(bH), fpvEye, st.bP.z + fpvFwd * Math.sin(bH));
        fpvCamB.lookAt(st.bP.x + fpvLD * Math.cos(bH), 0.5, st.bP.z + fpvLD * Math.sin(bH));
        fpvRendererB.render(scene, fpvCamB);
        dB.visible = true;

        // Restore comm object visibility
        commObjs.forEach(function(o, idx) { o.visible = commVis[idx]; });
      }

      // Expanded (big) FPV render — lazily create renderer when canvas appears
      var expRef = el._expandedRef;
      if (expRef && expRef.current && fpvBigRef.current) {
        // Lazy init
        if (!el._fpvBigRenderer) {
          el._fpvBigRenderer = new THREE.WebGLRenderer({ canvas: fpvBigRef.current, antialias: true });
          el._fpvBigRenderer.setSize(el._fpvBigW, el._fpvBigH);
          el._fpvBigRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
          el._fpvBigRenderer.setClearColor(0x87ceeb);
          el._fpvBigCam = new THREE.PerspectiveCamera(55, el._fpvBigW / el._fpvBigH, 0.5, 300);
        }
        var bigR = el._fpvBigRenderer, bigC = el._fpvBigCam;
        var expH2 = expRef.current === "a" ? (st.aHeading || 0) : (st.bHeading || 0);
        var expP = expRef.current === "a" ? st.aP : st.bP;
        var expBoat = expRef.current === "a" ? dA : dB;
        // Hide comm objects + own boat for clean expanded FPV
        var bigCommObjs = [commBeam, d2gBeam1, d2gBeam2, recvRing].concat(commPkts, d2gPkts, sigRings, gsRelayRings);
        var bigCommVis = bigCommObjs.map(function(o) { return o.visible; });
        bigCommObjs.forEach(function(o) { o.visible = false; });
        expBoat.visible = false;
        bigC.position.set(expP.x + fpvFwd * Math.cos(expH2), fpvEye, expP.z + fpvFwd * Math.sin(expH2));
        bigC.lookAt(expP.x + fpvLD * Math.cos(expH2), 0.5, expP.z + fpvLD * Math.sin(expH2));
        bigR.render(scene, bigC);
        expBoat.visible = true;
        bigCommObjs.forEach(function(o, idx) { o.visible = bigCommVis[idx]; });
      } else if (el._fpvBigRenderer && (!expRef || !expRef.current)) {
        // Dispose when closed to free resources
        el._fpvBigRenderer.dispose();
        el._fpvBigRenderer = null;
        el._fpvBigCam = null;
      }
    }

    // FPV renderers (share scene, small resolution, no shadows)
    var fpvW = 160, fpvHt = 100;
    var fpvRendererA = null, fpvRendererB = null, fpvCamA = null, fpvCamB = null;
    if (fpvARef.current && fpvBRef.current) {
      fpvRendererA = new THREE.WebGLRenderer({ canvas: fpvARef.current, antialias: false });
      fpvRendererA.setSize(fpvW, fpvHt);
      fpvRendererA.setPixelRatio(1);
      fpvRendererA.setClearColor(0x87ceeb);

      fpvRendererB = new THREE.WebGLRenderer({ canvas: fpvBRef.current, antialias: false });
      fpvRendererB.setSize(fpvW, fpvHt);
      fpvRendererB.setPixelRatio(1);
      fpvRendererB.setClearColor(0x87ceeb);

      fpvCamA = new THREE.PerspectiveCamera(60, fpvW / fpvHt, 0.5, 200);
      fpvCamB = new THREE.PerspectiveCamera(60, fpvW / fpvHt, 0.5, 200);
    }

    // Big FPV renderer — created lazily when canvas appears
    var bigW = 640, bigH = 400;
    var expandedRef = { current: null };
    el._expandedRef = expandedRef;
    el._fpvBigRenderer = null;
    el._fpvBigCam = null;
    el._fpvBigW = bigW;
    el._fpvBigH = bigH;
    el._scene = scene;

    animate();
  }, []);

  // Divider drag handlers
  var onDividerDown = useCallback(function(e) {
    e.preventDefault();
    draggingDivider.current = true;
    document.body.style.cursor = "col-resize";
    document.body.style.userSelect = "none";
  }, []);

  useEffect(function() {
    function onMove(e) {
      if (!draggingDivider.current || !containerRef.current) return;
      var rect = containerRef.current.getBoundingClientRect();
      var x = (e.touches ? e.touches[0].clientX : e.clientX) - rect.left;
      var pct = 100 - (x / rect.width * 100);
      setMapWidth(Math.max(20, Math.min(80, pct)));
    }
    function onUp() {
      if (draggingDivider.current) {
        draggingDivider.current = false;
        document.body.style.cursor = "";
        document.body.style.userSelect = "";
      }
    }
    window.addEventListener("mousemove", onMove);
    window.addEventListener("mouseup", onUp);
    window.addEventListener("touchmove", onMove);
    window.addEventListener("touchend", onUp);
    return function() {
      window.removeEventListener("mousemove", onMove);
      window.removeEventListener("mouseup", onUp);
      window.removeEventListener("touchmove", onMove);
      window.removeEventListener("touchend", onUp);
    };
  }, []);

  var st = renderState;
  var sim = simRef.current;

  // Derived IMU data
  var aSpeed = sim.droneA.speed;
  var bSpeed = sim.droneB.speed;
  var now = Date.now() * 0.001;
  var aRoll = Math.sin(now * 1.6) * 0.05 + Math.cos(now * 2.3) * 0.025;
  var aPitch = Math.sin(now * 1.3) * 0.03;
  var bRoll = Math.sin(now * 1.6 + 1.5) * 0.05 + Math.cos(now * 2.3 + 1.5) * 0.025;
  var bPitch = Math.sin(now * 1.3 + 1.5) * 0.03;
  var aBearing = Math.atan2(st.tg.z - st.aP.z, st.tg.x - st.aP.x);
  var bBearing = Math.atan2(st.tg.z - st.bP.z, st.tg.x - st.bP.x);
  var aDist = d3(st.aP, st.tg), bDist = d3(st.bP, st.tg);
  var comm = commState(st.aP, st.bP, st.commAB, st.commBA, commProto);
  var instructions = getInstructions(sim, st, commProto);

  var btn = { border: "none", cursor: "pointer", fontFamily: "monospace", fontWeight: 600, fontSize: 11, borderRadius: 6, padding: "7px 18px" };

  return (
    <div style={{ fontFamily: "monospace", color: "#2a3a4a", background: "#fff", overflow: "hidden", width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "4px 10px", display: "flex", justifyContent: "space-between", alignItems: "center", background: "#f8fafb", borderBottom: "1px solid #e8eff5" }}>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 7, height: 7, borderRadius: "50%", background: "#0a8a5a" }} />
          <span style={{ fontSize: 14, fontWeight: 800, color: "#0a8a5a", letterSpacing: 1.5 }}>SEADRONE</span>
          <span style={{ fontSize: 14, fontWeight: 300, color: "#8a9aaa" }}>MULTI-AGENT</span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 12, fontSize: 11 }}>
          <span style={{ fontSize: 9, fontWeight: 700, color: "#c03030", background: "#c0303015", padding: "2px 8px", borderRadius: 4 }}>MANUAL MODE</span>
          <select value={commProto} onChange={function(e) { setCommProto(e.target.value); }} style={{ fontFamily: "monospace", fontSize: 10, fontWeight: 600, padding: "4px 8px", borderRadius: 5, border: "1px solid " + PROTOCOLS[commProto].color + "60", background: PROTOCOLS[commProto].color + "15", color: PROTOCOLS[commProto].color, cursor: "pointer" }}>
            {Object.values(PROTOCOLS).map(function(p) { return <option key={p.id} value={p.id}>{p.name}</option>; })}
          </select>
          {(function() { var info = simStatusText(sim); return info ? <span style={{ color: info.color, fontWeight: 700 }}>{info.icon} {info.text}</span> : null; })()}
          <span style={{ color: "#b0bcc8", fontSize: 10 }}>T+{sim.elapsed.toFixed(1)}s</span>
        </div>
      </div>

      <div ref={containerRef} style={{ display: "flex", flex: 1, minHeight: 0 }}>
        <div ref={mountRef} style={{ flex: 1, cursor: "grab", position: "relative", background: "#eef3f8", minWidth: 0 }}>
          <div style={{ position: "absolute", top: 8, left: 10, fontSize: 9, fontWeight: 700, color: "#0a8a5a", background: "#f8fafbdd", padding: "2px 8px", borderRadius: 4, border: "1px solid #e0e8f0", zIndex: 1 }}>3D VIEW</div>
          <div style={{ position: "absolute", bottom: 8, left: 10, fontSize: 8, color: "#a0b0c0", zIndex: 1 }}>Drag · Scroll</div>

          {/* Drone A — FPV + IMU (top-left, compact) */}
          <div style={{ position: "absolute", top: 28, left: 8, zIndex: 2, cursor: "pointer", opacity: 0.95 }} onClick={function() { setExpandedFpv(expandedFpv === "a" ? null : "a"); }}>
            <div style={{ fontSize: 7, fontWeight: 700, color: "#0a8a5a", background: "#0a1520", padding: "2px 5px", borderRadius: "4px 4px 0 0", display: "flex", justifyContent: "space-between", letterSpacing: 0.5 }}>
              <span>A — CAM</span><span style={{ color: "#4a6a7a", fontSize: 6 }}>{expandedFpv === "a" ? "CLOSE" : "EXPAND"}</span>
            </div>
            <div style={{ position: "relative" }}>
              <canvas ref={fpvARef} width={160} height={100} style={{ display: "block", border: "1px solid #0a8a5a30" }} />
              <FPVOverlay heading={st.aHeading || 0} inRange={st.aInRange} dist={aDist} color="#0a8a5a" />
            </div>
            <IMUPanel label="A" color="#0a8a5a" pos={st.aP} heading={st.aHeading || 0} speed={aSpeed} roll={aRoll} pitch={aPitch} bearing={aBearing} dist={aDist} inRange={st.aInRange} />
          </div>

          {/* Drone B — FPV + IMU (top-right, compact) */}
          <div style={{ position: "absolute", top: 28, right: 8, zIndex: 2, cursor: "pointer", opacity: 0.95 }} onClick={function() { setExpandedFpv(expandedFpv === "b" ? null : "b"); }}>
            <div style={{ fontSize: 7, fontWeight: 700, color: "#4838d0", background: "#0a1520", padding: "2px 5px", borderRadius: "4px 4px 0 0", display: "flex", justifyContent: "space-between", letterSpacing: 0.5 }}>
              <span>B — CAM</span><span style={{ color: "#4a6a7a", fontSize: 6 }}>{expandedFpv === "b" ? "CLOSE" : "EXPAND"}</span>
            </div>
            <div style={{ position: "relative" }}>
              <canvas ref={fpvBRef} width={160} height={100} style={{ display: "block", border: "1px solid #4838d030" }} />
              <FPVOverlay heading={st.bHeading || 0} inRange={st.bInRange} dist={bDist} color="#4838d0" />
            </div>
            <IMUPanel label="B" color="#4838d0" pos={st.bP} heading={st.bHeading || 0} speed={bSpeed} roll={bRoll} pitch={bPitch} bearing={bBearing} dist={bDist} inRange={st.bInRange} />
          </div>

          {/* Expanded FPV overlay */}
          {expandedFpv && <div style={{ position: "absolute", top: "50%", left: "50%", transform: "translate(-50%, -50%)", zIndex: 10 }} onClick={function(e) { e.stopPropagation(); }}>
            <div style={{ background: "#0a1520", borderRadius: 8, border: "2px solid " + (expandedFpv === "a" ? "#0a8a5a" : "#4838d0"), overflow: "hidden", boxShadow: "0 8px 32px #000a" }}>
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", padding: "4px 10px", background: "#0a1520" }}>
                <span style={{ fontSize: 11, fontWeight: 700, color: expandedFpv === "a" ? "#0a8a5a" : "#4838d0", letterSpacing: 1 }}>
                  DRONE {expandedFpv.toUpperCase()} — CAMERA FEED
                </span>
                <span onClick={function() { setExpandedFpv(null); }} style={{ fontSize: 10, color: "#6a8a9a", cursor: "pointer", padding: "2px 8px", borderRadius: 3, background: "#ffffff10" }}>CLOSE</span>
              </div>
              <div style={{ position: "relative" }}>
                <canvas ref={fpvBigRef} width={640} height={400} style={{ display: "block" }} />
                <FPVOverlay heading={expandedFpv === "a" ? (st.aHeading || 0) : (st.bHeading || 0)} inRange={expandedFpv === "a" ? st.aInRange : st.bInRange} dist={expandedFpv === "a" ? aDist : bDist} color={expandedFpv === "a" ? "#0a8a5a" : "#4838d0"} />
              </div>
              <div style={{ padding: "4px 10px 6px", background: "#0a1520", display: "flex", gap: 16, fontSize: 10, fontFamily: "monospace", color: "#8ab0c8" }}>
                <span>HDG <span style={{ color: "#e0f0ff" }}>{(((expandedFpv === "a" ? st.aHeading : st.bHeading) * 180 / Math.PI + 360) % 360).toFixed(1)}°</span></span>
                <span>SPD <span style={{ color: "#e0f0ff" }}>{(expandedFpv === "a" ? aSpeed : bSpeed).toFixed(1)} m/s</span></span>
                <span>BRG <span style={{ color: (expandedFpv === "a" ? st.aInRange : st.bInRange) ? "#ffd700" : "#e0f0ff" }}>{(((expandedFpv === "a" ? aBearing : bBearing) * 180 / Math.PI + 360) % 360).toFixed(1)}°</span></span>
                <span>DIST <span style={{ color: (expandedFpv === "a" ? st.aInRange : st.bInRange) ? "#ffd700" : "#e0f0ff" }}>{(expandedFpv === "a" ? aDist : bDist).toFixed(1)}m</span></span>
              </div>
            </div>
          </div>}

          {/* Instruction Comm Log (bottom-left, compact) */}
          <div style={{ position: "absolute", bottom: 24, left: 8, zIndex: 2, maxWidth: 280, opacity: 0.9 }}>
            <InstructionLog messages={instructions} />
          </div>
        </div>
        {/* Draggable divider */}
        <div
          onMouseDown={onDividerDown}
          onTouchStart={onDividerDown}
          style={{
            width: 6, cursor: "col-resize", background: "#e0e8f0", flexShrink: 0,
            display: "flex", alignItems: "center", justifyContent: "center",
            transition: "background 0.15s"
          }}
          onMouseEnter={function(e) { e.currentTarget.style.background = "#1a80c0"; }}
          onMouseLeave={function(e) { if (!draggingDivider.current) e.currentTarget.style.background = "#e0e8f0"; }}
        >
          <div style={{ width: 2, height: 40, borderRadius: 1, background: "#b0bcc8" }} />
        </div>
        <div style={{
          width: mapWidth + "%",
          flexShrink: 0,
          padding: 10,
          background: "#fafcfe",
          display: "flex", flexDirection: "column", gap: 6
        }}>
          <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center" }}>
            <span style={{ fontSize: 10, fontWeight: 700, color: "#1a80c0", letterSpacing: .8 }}>SPATIAL MAP</span>
            <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
              {st.pi >= 5 && <span style={{ fontSize: 8, fontWeight: 700, color: "#c06020", background: "#c0602015", padding: "2px 6px", borderRadius: 3 }}>TRIANG ACTIVE</span>}
              <button onClick={function() { setMapFull(!mapFull); }} style={{ border: "1px solid #d0dae4", background: mapFull ? "#edf8f3" : "#f2f6f9", color: mapFull ? "#0a8a5a" : "#6a7a8a", cursor: "pointer", fontFamily: "monospace", fontSize: 9, fontWeight: 600, borderRadius: 4, padding: "2px 8px" }}>
                {mapFull ? "Show Panels" : "Expand Map"}
              </button>
            </div>
          </div>
          <Map2D state={st} trail={{ a: sim.aTrail, b: sim.bTrail, t: sim.tTrail }} proto={commProto} elapsed={sim.elapsed} blasted={sim.blasted} />
          {!mapFull && <div style={{ display: "flex", gap: 6 }}>
            <div style={{ flex: 1 }}><CommDashboard comm={comm} proto={commProto} /></div>
            <div style={{ flex: 1 }}><PathOptPanel st={st} triangPhaseIdx={5} /></div>
          </div>}
          {!mapFull && <div style={{ display: "flex", gap: 10, fontSize: 9, color: "#6a7a8a", flexWrap: "wrap" }}>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 0, height: 0, borderLeft: "4px solid transparent", borderRight: "4px solid transparent", borderBottom: "7px solid #0a8a5a" }} />A</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 0, height: 0, borderLeft: "4px solid transparent", borderRight: "4px solid transparent", borderBottom: "7px solid #4838d0" }} />B</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 7, height: 7, borderRadius: "50%", background: "#d03030" }} />Target</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 12, borderTop: "1.5px dashed #c89020" }} />Bearing</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 0, height: 0, borderLeft: "6px solid transparent", borderRight: "6px solid transparent", borderBottom: "10px solid #90a8b8", opacity: .5 }} />FOV</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 8, height: 8, background: "#1a2a3a", border: "1.5px solid #f0a030", borderRadius: 2 }} />GS</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 12, borderTop: "2px solid #20d090" }} />D2D</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 12, borderTop: "1.5px dashed #f0a030" }} />D2G</span>
          </div>}
        </div>
      </div>

      <div style={{ padding: "4px 10px 5px", background: "#f8fafb", borderTop: "1px solid #e8eff5" }}>
        <div style={{ display: "flex", alignItems: "center", gap: 12, fontFamily: "monospace", fontSize: 10 }}>
          {/* Drone A state */}
          <div style={{ display: "flex", alignItems: "center", gap: 4 }}>
            <div style={{ width: 8, height: 8, borderRadius: "50%", background: sim.droneA.state === "SEARCH" ? "#1a80c0" : sim.droneA.state === "TRIANGULATE" ? "#0a8a5a" : "#c89020" }} />
            <span style={{ fontWeight: 700, color: "#0a8a5a" }}>A: {sim.droneA.state}</span>
          </div>
          {/* Drone B state */}
          <div style={{ display: "flex", alignItems: "center", gap: 4 }}>
            <div style={{ width: 8, height: 8, borderRadius: "50%", background: sim.droneB.state === "SEARCH" ? "#1a80c0" : sim.droneB.state === "TRIANGULATE" ? "#0a8a5a" : "#c89020" }} />
            <span style={{ fontWeight: 700, color: "#4838d0" }}>B: {sim.droneB.state}</span>
          </div>
          <div style={{ width: 1, height: 16, background: "#d0e0e8" }} />
          {/* Target steering info */}
          <div style={{ display: "flex", alignItems: "center", gap: 6, color: "#c03030" }}>
            <span style={{ fontWeight: 700 }}>TARGET</span>
            <span>HDG {((sim.target.heading * 180 / Math.PI + 360) % 360).toFixed(0)}°</span>
            <span>SPD {sim.target.speed.toFixed(1)} m/s</span>
            <div style={{ width: 60, height: 6, background: "#e4ecf2", borderRadius: 3, overflow: "hidden" }}>
              <div style={{ width: (sim.target.speed / TARGET_MAX_SPEED * 100) + "%", height: "100%", background: "#c03030", borderRadius: 3 }} />
            </div>
          </div>
          <div style={{ width: 1, height: 16, background: "#d0e0e8" }} />
          <div style={{ display: "flex", gap: 2, alignItems: "center" }}>
            <span style={{ background: "#e4ecf2", padding: "1px 5px", borderRadius: 3, fontWeight: 700, fontSize: 8 }}>W</span>
            <span style={{ background: "#e4ecf2", padding: "1px 5px", borderRadius: 3, fontWeight: 700, fontSize: 8 }}>A</span>
            <span style={{ background: "#e4ecf2", padding: "1px 5px", borderRadius: 3, fontWeight: 700, fontSize: 8 }}>S</span>
            <span style={{ background: "#e4ecf2", padding: "1px 5px", borderRadius: 3, fontWeight: 700, fontSize: 8 }}>D</span>
            <span style={{ color: "#8a9aaa", fontSize: 7 }}>steer</span>
          </div>
          <div style={{ flex: 1 }} />
          <button onClick={function() { setRunning(!running); }} style={{ border: "1px solid " + (running ? "#c0303025" : "#0a8a5a25"), background: running ? "#fef2f2" : "#edf8f3", color: running ? "#c03030" : "#0a8a5a", cursor: "pointer", fontFamily: "monospace", fontWeight: 700, fontSize: 11, borderRadius: 6, padding: "5px 18px" }}>
            {running ? "Stop" : "▶ Start"}
          </button>
          <button onClick={function() { setRunning(false); simRef.current = initialSim(); debugLog.length = 0; debugLastSnapshot = 0; debugPrevStateA = ""; debugPrevStateB = ""; }} style={{ border: "1px solid #dce6ef", background: "#f2f6f9", color: "#8a9aaa", cursor: "pointer", fontFamily: "monospace", fontWeight: 600, fontSize: 10, borderRadius: 6, padding: "5px 14px" }}>Reset</button>
          <button onClick={function() {
            var blob = new Blob([JSON.stringify({ log: debugLog, totalEntries: debugLog.length }, null, 2)], { type: "application/json" });
            var a = document.createElement("a"); a.href = URL.createObjectURL(blob); a.download = "debug_sim_" + Date.now() + ".json"; a.click();
          }} style={{ border: "1px solid #c89020", background: "#fef8e8", color: "#8a6a2a", cursor: "pointer", fontFamily: "monospace", fontWeight: 600, fontSize: 10, borderRadius: 6, padding: "5px 14px" }}>Debug Log ({debugLog.length})</button>
          <span style={{ color: "#b0bcc8" }}>T+{sim.elapsed.toFixed(1)}s</span>
        </div>
      </div>
    </div>
  );
}
