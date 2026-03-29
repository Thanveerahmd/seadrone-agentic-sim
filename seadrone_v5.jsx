import { useState, useEffect, useRef, useCallback } from "react";
import * as THREE from "three";

const TOTAL = 960;
const PHASES = [
  { id: "DEPLOY", label: "Deploy", c: "#0a8a5a", dur: 20 },
  { id: "SEARCH", label: "Search", c: "#1a80c0", dur: 100 },
  { id: "DETECT", label: "Detect", c: "#c89020", dur: 140 },
  { id: "RELAY", label: "Relay", c: "#c06020", dur: 180 },
  { id: "TRIANG", label: "Triangulate", c: "#c03030", dur: 60 },
  { id: "TRACK", label: "Track", c: "#0a8a5a", dur: 460 },
];
const PE = PHASES.reduce((a, p, i) => { a.push((a[i - 1] || 0) + p.dur); return a; }, []);
const OR = 22;

function gp(tk) {
  let a = 0;
  for (let i = 0; i < PHASES.length; i++) {
    if (tk < a + PHASES[i].dur) return { i, t: (tk - a) / PHASES[i].dur };
    a += PHASES[i].dur;
  }
  return { i: 5, t: 1 };
}

function lr(a, b, t) { return a + (b - a) * Math.max(0, Math.min(1, t)); }
function eIO(t) { return t < .5 ? 2 * t * t : 1 - (-2 * t + 2) ** 2 / 2; }
function d3(a, b) { return Math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2); }

// Search paths: drones start at bottom, A sweeps LEFT+UP, B sweeps RIGHT+UP
// A's path curves into the upper-left where the target will be
function gSA(sx, sz, n) {
  var p = [];
  for (var i = 0; i <= n; i++) {
    var t = i / n;
    p.push({
      x: sx - t * 40 + Math.sin(t * Math.PI * 2.4) * 5,
      z: sz + t * 52 + Math.sin(t * Math.PI * 1.6) * 4
    });
  }
  return p;
}
// B's path goes far right, well away from the target zone
// Extended path — B keeps searching during detect phase before receiving A's signal
function gSB(sx, sz, n) {
  var p = [];
  for (var i = 0; i <= n; i++) {
    var t = i / n;
    p.push({
      x: sx + t * 55 + Math.sin(t * Math.PI * 2.4) * 5,
      z: sz + t * 42 + Math.sin(t * Math.PI * 1.6) * 4
    });
  }
  // Extended search segment: B continues searching further right+up during detect phase
  var last = p[p.length - 1];
  var extLen = 60; // extra waypoints for continued search
  for (var j = 1; j <= extLen; j++) {
    var et = j / extLen;
    p.push({
      x: last.x + et * 15 + Math.sin(et * Math.PI * 3) * 6,
      z: last.z + et * 18 - Math.cos(et * Math.PI * 2) * 5
    });
  }
  return p;
}

var SA = gSA(-3, -26, 90);  // A: bottom → upper-left  (ends ~(-43, 26))
var SB = gSB(3, -26, 90);   // B: bottom → far upper-right (ends ~(58, 16))

// Target starts in A's search zone (upper-left), cruises left
var DETECT_START = 20 + 100; // tick when detect phase begins (DEPLOY + SEARCH)
function tAt(tk) {
  var stop = PE[4];
  if (tk >= stop) return tAt(stop - 1);
  var t = tk / stop;

  if (tk < DETECT_START) {
    // Pre-detection: target cruises in the upper-left zone, drifting toward A
    return {
      x: -18 - t * 55 + Math.sin(t * Math.PI * 2) * 5,
      z: 28 - t * 20 + Math.cos(t * Math.PI * 1.5) * 4
    };
  }
  // Post-detection: target spots A and tries to flee — runs RIGHT (away from A)
  var dt = (tk - DETECT_START) / (stop - DETECT_START);
  var t0 = DETECT_START / stop;
  var baseX = -18 - t0 * 55 + Math.sin(t0 * Math.PI * 2) * 5;
  var baseZ = 28 - t0 * 20 + Math.cos(t0 * Math.PI * 1.5) * 4;
  // Evasion: flee right and down (away from A which is on the left)
  var flee = Math.min(dt * 1.5, 1);
  var evadeX = flee * 40 + Math.sin(dt * Math.PI * 5) * 10 * flee;
  var evadeZ = -flee * 20 + Math.cos(dt * Math.PI * 4.5) * 8 * flee;
  var decay = 1 - dt * dt * 0.4;
  return {
    x: baseX + evadeX * decay,
    z: baseZ + evadeZ * decay
  };
}

function posAt(tk) {
  var r = gp(tk);
  var pi = r.i, lt = r.t;
  var tg = tAt(tk);
  var sbE = SB[SB.length - 1];
  var aP = SA[0], bP = SB[0], aD = false, bD = false, cm = false;

  var commAB = false; // A-to-B communication active

  // B's primary search ends at index 90, extended search continues after
  var bSearchEnd = 90; // index where original search pattern ends
  var bTotalPts = SB.length - 1; // total extended path points

  if (pi === 0) {
    // Deploy from bottom center
    aP = { x: lr(-3, SA[0].x, lt), z: lr(-30, SA[0].z, lt) };
    bP = { x: lr(3, SB[0].x, lt), z: lr(-30, SB[0].z, lt) };
  } else if (pi === 1) {
    // Search: A goes upper-left, B goes upper-right
    var ia = Math.min(Math.floor(lt * SA.length), SA.length - 1);
    var ib = Math.min(Math.floor(lt * bSearchEnd), bSearchEnd - 1);
    aP = SA[ia]; bP = SB[ib];
    if (lt > .92) { aD = true; cm = true; }
  } else if (pi === 2) {
    // Detect: A locks on and chases the fleeing target
    aD = true; cm = true;
    var tgPrev = tAt(Math.max(0, tk - 4));
    var fleeAng = Math.atan2(tg.z - tgPrev.z, tg.x - tgPrev.x);
    var chaseAng = fleeAng + Math.PI + lt * 0.6 - 0.3;
    var chaseDist = OR - lt * 6;
    aP = { x: tg.x + chaseDist * Math.cos(chaseAng), z: tg.z + chaseDist * Math.sin(chaseAng) };

    // B keeps searching independently — doesn't know about target yet
    // First 70%: B continues its extended search pattern
    // At 70%: A's signal reaches B → commAB activates, B starts to react
    if (lt < 0.7) {
      // B continues searching further along its extended path
      var bExtIdx = bSearchEnd + Math.min(Math.floor(lt / 0.7 * (bTotalPts - bSearchEnd)), bTotalPts - bSearchEnd);
      bP = SB[bExtIdx];
    } else {
      // Signal received! B stops searching, begins to turn toward target
      commAB = true;
      var bSignalPos = SB[bTotalPts]; // B's position when signal arrives
      // B starts a slow turn — easing into heading change (first reaction)
      var reactT = (lt - 0.7) / 0.3;
      var reactEase = reactT * reactT; // gentle start
      bP = {
        x: lr(bSignalPos.x, bSignalPos.x + (tg.x - bSignalPos.x) * 0.05, reactEase),
        z: lr(bSignalPos.z, bSignalPos.z + (tg.z - bSignalPos.z) * 0.05, reactEase)
      };
    }
  } else if (pi === 3) {
    // Relay: A keeps pursuing, B accelerates toward the target
    aD = true; cm = true; commAB = true;
    var tgPrev2 = tAt(Math.max(0, tk - 4));
    var fleeAng2 = Math.atan2(tg.z - tgPrev2.z, tg.x - tgPrev2.x);
    var chaseAng2 = fleeAng2 + Math.PI + lt * 1.0;
    var chaseDist2 = (OR - 6) - lt * 2;
    aP = { x: tg.x + chaseDist2 * Math.cos(chaseAng2), z: tg.z + chaseDist2 * Math.sin(chaseAng2) };
    // B's starting position = where it was at end of detect phase (signal received)
    var bStart = SB[bTotalPts];
    // B: first 10% — final heading adjustment, engine throttle-up
    // Then cubic ease-in acceleration toward target orbit position
    var rawT = Math.max(0, (lt - 0.1) / 0.9);
    var bEase = rawT < 0.5
      ? rawT * rawT * rawT / (0.5 * 0.5 * 0.5) * 0.35
      : 0.35 + 0.65 * eIO((rawT - 0.5) / 0.5);
    // B approaches from opposite side of A for triangulation geometry
    var bGoalAng = chaseAng2 + Math.PI;
    var bGoal = { x: tg.x + OR * Math.cos(bGoalAng), z: tg.z + OR * Math.sin(bGoalAng) };
    var bMid = { x: (bStart.x + bGoal.x) / 2, z: (bStart.z + bGoal.z) / 2 + 10 };
    var m = 1 - bEase;
    bP = { x: m * m * bStart.x + 2 * m * bEase * bMid.x + bEase * bEase * bGoal.x, z: m * m * bStart.z + 2 * m * bEase * bMid.z + bEase * bEase * bGoal.z };
    if (lt > .65) bD = true;
  } else {
    // Track + Triangulate: both drones lock the (tiring) target
    // Strict 180° opposition — A and B always on opposite sides, never collide
    aD = true; bD = true; cm = true;
    var dur = PHASES[4].dur + PHASES[5].dur;
    var pt = Math.min(Math.max((tk - PE[3]) / dur, 0), 1);
    // Base angle rotates steadily; both drones stay exactly PI apart
    var baseAng = Math.PI * 0.4 + pt * Math.PI * 3.2;
    // Small radial wobble for realism, but mirrored so they never close in on each other
    var wobble = 1.5 * Math.sin(pt * Math.PI * 6);
    aP = { x: tg.x + (OR + wobble) * Math.cos(baseAng), z: tg.z + (OR + wobble) * Math.sin(baseAng) };
    bP = { x: tg.x + (OR - wobble) * Math.cos(baseAng + Math.PI), z: tg.z + (OR - wobble) * Math.sin(baseAng + Math.PI) };
  }
  return { aP: aP, bP: bP, aD: aD, bD: bD, cm: cm, commAB: commAB, tg: tg, pi: pi, lt: lt };
}

// 2D Map - pure SVG, no state, just reads tick
function Map2D({ tick }) {
  var s = posAt(tick);
  var ts = tick >= PE[4];
  var SC = 4.5;
  var W = 600, H = 600;
  var cx = function(x) { return W / 2 + x * SC; };
  var cy = function(z) { return H * 0.45 - z * SC; };

  // Build trails directly - sample every 12 ticks for performance
  var aTrail = [], bTrail = [], tTrail = [];
  for (var t = 0; t <= tick; t += 12) {
    var st = posAt(t);
    aTrail.push(st.aP);
    bTrail.push(st.bP);
    if (st.pi >= 2) tTrail.push(st.tg);
  }

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

      {aTrail.length > 1 && <path d={pD(aTrail)} fill="none" stroke="#0a8a5a" strokeWidth="2" opacity=".25" />}
      {bTrail.length > 1 && <path d={pD(bTrail)} fill="none" stroke="#4838d0" strokeWidth="2" opacity=".25" />}
      {tTrail.length > 1 && <path d={pD(tTrail)} fill="none" stroke="#c03030" strokeWidth="1.5" opacity=".2" strokeDasharray="4 5" />}

      {/* Comm link A→B */}
      {s.commAB && <line x1={cx(s.aP.x)} y1={cy(s.aP.z)} x2={cx(s.bP.x)} y2={cy(s.bP.z)} stroke="#20d090" strokeWidth="1.5" opacity=".3" strokeDasharray="5 4" />}
      {s.commAB && [0, .25, .5, .75].map(function(off, ci) {
        var ct = ((tick * .008 + off) % 1);
        return <circle key={"cp" + ci} cx={lr(cx(s.aP.x), cx(s.bP.x), ct)} cy={lr(cy(s.aP.z), cy(s.bP.z), ct)} r={3 + 2 * Math.sin(ct * Math.PI)} fill="#20d090" opacity={.7 * Math.sin(ct * Math.PI)} />;
      })}
      {s.commAB && <g>
        <circle cx={cx(s.aP.x)} cy={cy(s.aP.z)} r="14" fill="none" stroke="#20d090" strokeWidth=".8" opacity=".2" />
        <circle cx={cx(s.aP.x)} cy={cy(s.aP.z)} r="22" fill="none" stroke="#20d090" strokeWidth=".5" opacity=".1" />
        <text x={(cx(s.aP.x) + cx(s.bP.x)) / 2} y={(cy(s.aP.z) + cy(s.bP.z)) / 2 - 10} textAnchor="middle" style={{ fontSize: "10px", fill: "#20d090", fontFamily: "monospace", fontWeight: 700 }}>COMM</text>
      </g>}

      {s.pi >= 4 && <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r={OR * SC} fill="none" stroke="#0a8a5a" strokeWidth="1" opacity=".15" strokeDasharray="6 6" />}
      {s.pi >= 4 && <line x1={cx(s.aP.x)} y1={cy(s.aP.z)} x2={cx(s.bP.x)} y2={cy(s.bP.z)} stroke="#0a8a5a" strokeWidth=".5" opacity=".12" strokeDasharray="5 7" />}

      {s.aD && <line x1={cx(s.aP.x)} y1={cy(s.aP.z)} x2={cx(s.tg.x)} y2={cy(s.tg.z)} stroke="#c89020" strokeWidth="1.4" opacity=".4" strokeDasharray="6 5" />}
      {s.bD && <line x1={cx(s.bP.x)} y1={cy(s.bP.z)} x2={cx(s.tg.x)} y2={cy(s.tg.z)} stroke="#c06020" strokeWidth="1.4" opacity=".4" strokeDasharray="6 5" />}

      <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="9" fill="#d03030" stroke="#fff" strokeWidth="2" />
      <circle cx={cx(s.tg.x)} cy={cy(s.tg.z)} r="3.5" fill="#f8a0a0" />
      <text x={cx(s.tg.x) + 14} y={cy(s.tg.z) + 5} style={{ fontSize: "13px", fill: ts ? "#0a8a5a" : "#b02020", fontFamily: "monospace", fontWeight: 700 }}>
        {ts ? "IDLE" : s.pi >= 2 ? "EVADING" : ""}
      </text>

      {s.aD && <circle cx={cx(s.aP.x)} cy={cy(s.aP.z)} r="18" fill="none" stroke="#0a8a5a" strokeWidth="1" opacity=".25" />}
      <polygon points={cx(s.aP.x) + "," + (cy(s.aP.z) - 9) + " " + (cx(s.aP.x) - 8) + "," + (cy(s.aP.z) + 7) + " " + (cx(s.aP.x) + 8) + "," + (cy(s.aP.z) + 7)} fill="#0a8a5a" stroke="#fff" strokeWidth="1.2" />
      <text x={cx(s.aP.x)} y={cy(s.aP.z) - 16} textAnchor="middle" style={{ fontSize: "13px", fill: "#0a8a5a", fontFamily: "monospace", fontWeight: 700 }}>
        {"A" + (dA ? " " + dA + "m" : "")}
      </text>

      {s.bD && <circle cx={cx(s.bP.x)} cy={cy(s.bP.z)} r="18" fill="none" stroke="#4838d0" strokeWidth="1" opacity=".25" />}
      <polygon points={cx(s.bP.x) + "," + (cy(s.bP.z) - 9) + " " + (cx(s.bP.x) - 8) + "," + (cy(s.bP.z) + 7) + " " + (cx(s.bP.x) + 8) + "," + (cy(s.bP.z) + 7)} fill="#4838d0" stroke="#fff" strokeWidth="1.2" />
      <text x={cx(s.bP.x)} y={cy(s.bP.z) - 16} textAnchor="middle" style={{ fontSize: "13px", fill: "#4838d0", fontFamily: "monospace", fontWeight: 700 }}>
        {"B" + (dB ? " " + dB + "m" : "")}
      </text>

      {s.pi >= 4 && <g>
        <rect x={W - 130} y="10" width="118" height="28" rx="5" fill="#eaf6f0" stroke="#0a8a5a30" strokeWidth=".6" />
        <text x={W - 71} y="29" textAnchor="middle" style={{ fontSize: "13px", fill: "#0a8a5a", fontFamily: "monospace", fontWeight: 700 }}>180° OPT</text>
      </g>}

      <line x1="14" y1={H - 18} x2={14 + 20 * SC} y2={H - 18} stroke="#90a0b0" strokeWidth="1" />
      <text x={14 + 10 * SC} y={H - 6} textAnchor="middle" style={{ fontSize: "11px", fill: "#90a0b0", fontFamily: "monospace" }}>20m</text>
    </svg>
  );
}

// ── MAIN ──
export default function App() {
  var mountRef = useRef(null);
  var initDone = useRef(false);
  var [tick, setTick] = useState(0);
  var [playing, setPlaying] = useState(false);
  var [speed, setSpeed] = useState(1);
  var [mapWidth, setMapWidth] = useState(35); // percentage of viewport
  var draggingDivider = useRef(false);
  var containerRef = useRef(null);
  var tickRef = useRef(0);
  var raf = useRef(null);
  var lastTime = useRef(null);

  tickRef.current = tick;

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
      var tk = tickRef.current;
      var st = posAt(tk);
      var stopped = tk >= PE[4];
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

      dA.position.set(st.aP.x, bob(0), st.aP.z);
      dA.rotation.y = Math.atan2(st.tg.x - st.aP.x, st.tg.z - st.aP.z);
      dA.rotation.z = rock(0);
      dA.rotation.x = pitch(0);
      dA.userData.rings.forEach(function(r, i) { r.visible = st.aD; if (st.aD) r.material.opacity = (.18 - i * .04) * (.7 + .3 * Math.sin(time * 2 + i)); });

      dB.position.set(st.bP.x, bob(1.5), st.bP.z);
      dB.rotation.y = Math.atan2(st.tg.x - st.bP.x, st.tg.z - st.bP.z);
      dB.rotation.z = rock(1.5);
      dB.rotation.x = pitch(1.5);
      dB.userData.rings.forEach(function(r, i) { r.visible = st.bD; if (st.bD) r.material.opacity = (.18 - i * .04) * (.7 + .3 * Math.sin(time * 2 + i + 1)); });

      tgt.position.set(st.tg.x, bob(3), st.tg.z);
      var fleeing = st.pi >= 2 && !stopped;
      tgt.rotation.z = fleeing ? Math.sin(time * 3.5 + 3) * .1 : rock(3);
      tgt.rotation.x = fleeing ? Math.sin(time * 2.8) * .06 : pitch(3);
      if (!stopped) { var tp = tAt(Math.max(0, tk - 3)); tgt.rotation.y = Math.atan2(st.tg.x - tp.x, st.tg.z - tp.z); }

      // Blinking antenna lights
      var blink = Math.sin(time * 4) > 0.3 ? 1 : 0.15;
      var blinkB = Math.sin(time * 4 + 2) > 0.3 ? 1 : 0.15;
      if (dA.userData.tip) dA.userData.tip.material.opacity = blink;
      if (dB.userData.tip) dB.userData.tip.material.opacity = blinkB;

      // Wake foam particles behind moving boats
      if (tk > 0 && tk % 2 === 0) {
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

      orbRing.visible = st.pi >= 4;
      if (st.pi >= 4) orbRing.position.set(st.tg.x, .2, st.tg.z);
      diaLine.visible = st.pi >= 4;
      if (st.pi >= 4) { var dp = diaLine.geometry.attributes.position; dp.setXYZ(0, st.aP.x, .5, st.aP.z); dp.setXYZ(1, st.bP.x, .5, st.bP.z); dp.needsUpdate = true; }
      idleRing.visible = stopped;
      if (stopped) { idleRing.position.set(st.tg.x, .1, st.tg.z); idleRing.material.opacity = .15 + .08 * Math.sin(time * 1.5); }

      // Packets (bearing data once both tracking)
      var pp = (tk * .05) % 1;
      if (st.cm && st.aD && !st.commAB) {
        var toX = st.bD ? st.bP.x : st.tg.x, toZ = st.bD ? st.bP.z : st.tg.z;
        pkt1.visible = true;
        pkt1.position.set(lr(st.aP.x, toX, pp), 2.2, lr(st.aP.z, toZ, pp));
      } else { pkt1.visible = false; }
      if (st.cm && st.bD && !st.commAB) {
        pkt2.visible = true;
        pkt2.position.set(lr(st.bP.x, st.aP.x, (pp + .5) % 1), 2.2, lr(st.bP.z, st.aP.z, (pp + .5) % 1));
      } else { pkt2.visible = false; }

      // Comm signals A→B (detect + relay phases)
      if (st.commAB) {
        // Comm beam line
        commBeam.visible = true;
        var cbp = commBeam.geometry.attributes.position;
        cbp.setXYZ(0, st.aP.x, 2.5, st.aP.z);
        cbp.setXYZ(1, st.bP.x, 2.5, st.bP.z);
        cbp.needsUpdate = true;
        commBeam.computeLineDistances();
        commBeam.material.opacity = .2 + .15 * Math.sin(time * 3);

        // Staggered packets traveling A→B
        for (var ci = 0; ci < commPkts.length; ci++) {
          var ct = ((time * .8 + ci * .25) % 1);
          commPkts[ci].visible = true;
          commPkts[ci].position.set(
            lr(st.aP.x, st.bP.x, ct),
            2.5 + Math.sin(ct * Math.PI) * 1.2,
            lr(st.aP.z, st.bP.z, ct)
          );
          commPkts[ci].material.opacity = .9 * Math.sin(ct * Math.PI);
          var cs = .25 + .2 * Math.sin(ct * Math.PI);
          commPkts[ci].scale.set(cs * 3, cs * 3, cs * 3);
        }

        // Signal pulse rings at A (transmitting)
        for (var si = 0; si < sigRings.length; si++) {
          sigRings[si].visible = true;
          sigRings[si].position.set(st.aP.x, 2.6, st.aP.z);
          var sp = ((time * 1.2 + si * .33) % 1);
          var sr2 = 1 + sp * 5;
          sigRings[si].scale.set(sr2, sr2, sr2);
          sigRings[si].material.opacity = .35 * (1 - sp);
        }

        // Receive pulse at B
        recvRing.visible = true;
        recvRing.position.set(st.bP.x, 2.0, st.bP.z);
        var rp = ((time * 1.5) % 1);
        var rr = 1 + rp * 3;
        recvRing.scale.set(rr, rr, rr);
        recvRing.material.opacity = .3 * (1 - rp);
      } else {
        commBeam.visible = false;
        for (var ci2 = 0; ci2 < commPkts.length; ci2++) commPkts[ci2].visible = false;
        for (var si2 = 0; si2 < sigRings.length; si2++) sigRings[si2].visible = false;
        recvRing.visible = false;
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
      if (tk % 3 === 0 && tk > 0) {
        pushT(trA, st.aP.x, st.aP.z);
        pushT(trB, st.bP.x, st.bP.z);
        if (st.pi >= 2) pushT(trT, st.tg.x, st.tg.z);
      }

      // Reset trails when tick is 0
      if (tk === 0) {
        [trA, trB, trT].forEach(function(tr) { tr.count = 0; tr.line.geometry.setDrawRange(0, 0); });
      }

      // Camera
      var lookX = st.tg.x * .4, lookZ = st.tg.z * .4 + 4;
      camera.position.set(lookX + camD * Math.sin(camPh) * Math.sin(camTh), camD * Math.cos(camPh), lookZ + camD * Math.sin(camPh) * Math.cos(camTh));
      camera.lookAt(lookX, 0, lookZ);
      renderer.render(scene, camera);
    }
    animate();
  }, []);

  // Playback loop
  var loop = useCallback(function(ts) {
    if (!lastTime.current) lastTime.current = ts;
    if (ts - lastTime.current > 1000 / (42 * speed)) {
      lastTime.current = ts;
      setTick(function(p) { if (p >= TOTAL - 1) { setPlaying(false); return TOTAL - 1; } return p + 1; });
    }
    raf.current = requestAnimationFrame(loop);
  }, [speed]);

  useEffect(function() {
    if (playing) { lastTime.current = null; raf.current = requestAnimationFrame(loop); }
    else if (raf.current) cancelAnimationFrame(raf.current);
    return function() { if (raf.current) cancelAnimationFrame(raf.current); };
  }, [playing, loop]);

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

  var phase = gp(tick);
  var ph = PHASES[phase.i];
  var ts = tick >= PE[4];
  var st = posAt(tick);
  var btn = { border: "none", cursor: "pointer", fontFamily: "monospace", fontWeight: 600, fontSize: 11, borderRadius: 6, padding: "7px 18px" };

  return (
    <div style={{ fontFamily: "monospace", color: "#2a3a4a", background: "#fff", overflow: "hidden", width: "100vw", height: "100vh", display: "flex", flexDirection: "column" }}>
      <div style={{ padding: "8px 14px", display: "flex", justifyContent: "space-between", alignItems: "center", background: "#f8fafb", borderBottom: "1px solid #e8eff5" }}>
        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
          <div style={{ width: 7, height: 7, borderRadius: "50%", background: playing ? "#0a8a5a" : "#c0ccd4" }} />
          <span style={{ fontSize: 14, fontWeight: 800, color: "#0a8a5a", letterSpacing: 1.5 }}>SEADRONE</span>
          <span style={{ fontSize: 14, fontWeight: 300, color: "#8a9aaa" }}>MULTI-AGENT</span>
        </div>
        <div style={{ fontSize: 11 }}>
          {ts ? <span style={{ color: "#0a8a5a", fontWeight: 700 }}>● CONTAINED</span>
            : st.aD && st.bD && !st.commAB ? <span style={{ color: "#c06020" }}>◉ TRIANGULATING</span>
            : st.commAB && st.bD ? <span style={{ color: "#c06020" }}>◉ B CLOSING IN</span>
            : st.commAB && !st.bD ? <span style={{ color: "#20d090" }}>◈ SIGNAL SENT · B RESPONDING</span>
            : st.aD && !st.bD && !st.commAB ? <span style={{ color: "#c89020" }}>◐ A CHASING · B SEARCHING</span>
            : phase.i >= 1 ? <span style={{ color: "#1a80c0" }}>○ SEARCHING</span> : null}
          <span style={{ color: "#b0bcc8", marginLeft: 12, fontSize: 10 }}>T+{(tick / TOTAL * 65).toFixed(1)}s</span>
        </div>
      </div>

      <div ref={containerRef} style={{ display: "flex", flex: 1, minHeight: 0 }}>
        <div ref={mountRef} style={{ flex: 1, cursor: "grab", position: "relative", background: "#eef3f8", minWidth: 0 }}>
          <div style={{ position: "absolute", top: 8, left: 10, fontSize: 9, fontWeight: 700, color: "#0a8a5a", background: "#f8fafbdd", padding: "2px 8px", borderRadius: 4, border: "1px solid #e0e8f0", zIndex: 1 }}>3D VIEW</div>
          <div style={{ position: "absolute", bottom: 8, left: 10, fontSize: 8, color: "#a0b0c0", zIndex: 1 }}>Drag · Scroll</div>
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
          <span style={{ fontSize: 10, fontWeight: 700, color: "#1a80c0", letterSpacing: .8 }}>SPATIAL MAP</span>
          <Map2D tick={tick} />
          <div style={{ display: "flex", gap: 10, fontSize: 9, color: "#6a7a8a" }}>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 0, height: 0, borderLeft: "4px solid transparent", borderRight: "4px solid transparent", borderBottom: "7px solid #0a8a5a" }} />A</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 0, height: 0, borderLeft: "4px solid transparent", borderRight: "4px solid transparent", borderBottom: "7px solid #4838d0" }} />B</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 7, height: 7, borderRadius: "50%", background: "#d03030" }} />Target</span>
            <span style={{ display: "flex", alignItems: "center", gap: 3 }}><span style={{ display: "inline-block", width: 12, borderTop: "1.5px dashed #c89020" }} />Bearing</span>
          </div>
        </div>
      </div>

      <div style={{ padding: "8px 14px 10px", background: "#f8fafb", borderTop: "1px solid #e8eff5" }}>
        <div style={{ display: "flex", gap: 2, marginBottom: 6 }}>
          {PHASES.map(function(p, idx) {
            var start = idx === 0 ? 0 : PE[idx - 1];
            var act = idx <= phase.i, cur = idx === phase.i;
            return <div key={idx} style={{ flex: p.dur, cursor: "pointer" }} onClick={function() { setPlaying(false); setTick(start); }}>
              <div style={{ height: 5, borderRadius: 3, background: act ? p.c : "#e4ecf2", opacity: cur ? 1 : act ? .45 : .25, position: "relative" }}>
                {cur && <div style={{ position: "absolute", left: (phase.t * 100) + "%", top: -2, width: 4, height: 9, borderRadius: 2, background: "#2a3a4a" }} />}
              </div>
              <div style={{ fontSize: 8, textAlign: "center", fontWeight: cur ? 700 : 400, color: cur ? p.c : act ? "#8a9aaa" : "#c0ccd8", marginTop: 3 }}>{p.label}</div>
            </div>;
          })}
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
          <button style={{ ...btn, background: playing ? "#fef2f2" : "#edf8f3", color: playing ? "#c03030" : "#0a8a5a", border: "1px solid " + (playing ? "#c0303025" : "#0a8a5a25") }}
            onClick={function() { if (tick >= TOTAL - 1) setTick(0); setPlaying(!playing); }}>
            {playing ? "Pause" : tick >= TOTAL - 1 ? "Replay" : "▶ Play"}
          </button>
          <button style={{ ...btn, background: "#f2f6f9", color: "#8a9aaa", border: "1px solid #dce6ef" }} onClick={function() { setPlaying(false); setTick(0); }}>Reset</button>
          <div style={{ display: "flex", gap: 2, background: "#f2f6f9", borderRadius: 6, padding: 2, border: "1px solid #e4ecf2" }}>
            {[.5, 1, 2, 3].map(function(s) {
              return <button key={s} onClick={function() { setSpeed(s); }} style={{ border: "none", cursor: "pointer", fontFamily: "monospace", fontSize: 10, fontWeight: speed === s ? 700 : 400, padding: "4px 10px", borderRadius: 4, background: speed === s ? "#fff" : "transparent", color: speed === s ? "#0a8a5a" : "#a0b0c0", boxShadow: speed === s ? "0 1px 3px #0001" : "none" }}>{s}x</button>;
            })}
          </div>
          <input type="range" min="0" max={TOTAL - 1} value={tick} onChange={function(e) { setPlaying(false); setTick(+e.target.value); }} style={{ flex: 1, accentColor: ph.c }} />
          <span style={{ fontSize: 10, color: "#b0bcc8", minWidth: 30, textAlign: "right" }}>{Math.round(tick / TOTAL * 100)}%</span>
        </div>
      </div>
    </div>
  );
}
