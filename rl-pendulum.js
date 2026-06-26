// rl-pendulum.js - Simplified Portfolio-themed physics simulator and controller

// Helper to wrap angle to [-pi, pi]
function wrapAngle(x) {
  return Math.atan2(Math.sin(x), Math.cos(x));
}

// Gaussian Elimination solver for A * x = B
function solveLinearSystem(A, B) {
  const n = B.length;
  const M = [];
  for (let i = 0; i < n; i++) {
    M[i] = [...A[i], B[i]];
  }
  
  for (let i = 0; i < n; i++) {
    // Search for maximum pivot
    let maxEl = Math.abs(M[i][i]);
    let maxRow = i;
    for (let k = i + 1; k < n; k++) {
      if (Math.abs(M[k][i]) > maxEl) {
        maxEl = Math.abs(M[k][i]);
        maxRow = k;
      }
    }
    
    // Swap rows
    const temp = M[maxRow];
    M[maxRow] = M[i];
    M[i] = temp;
    
    // Normalize row
    const diag = M[i][i];
    if (Math.abs(diag) < 1e-10) {
      return new Array(n).fill(0); // singular matrix
    }
    for (let k = i; k <= n; k++) {
      M[i][k] /= diag;
    }
    
    // Eliminate elements below pivot
    for (let k = i + 1; k < n; k++) {
      const factor = M[k][i];
      for (let j = i; j <= n; j++) {
        M[k][j] -= factor * M[i][j];
      }
    }
  }
  
  // Back substitution
  const x = new Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    x[i] = M[i][n];
    for (let k = i + 1; k < n; k++) {
      x[i] -= M[i][k] * x[k];
    }
  }
  return x;
}

// Main State
const state = {
  modelName: '3pend',
  N: 3,
  
  params: {
    M_c: 1.0,
    m1: 0.1,
    m2: 0.1,
    m3: 0.1,
    L: 0.6,
    g: 9.81,
    d_c: 1.0,
    d_p: 0.05
  },
  
  q: null,       // coordinates: [x, alpha1, alpha2, ...]
  qd: null,      // velocities: [xd, ad1, ad2, ...]
  f_applied: 0.0,
  paused: false,
  stepCount: 0,
  cumulativeReward: 0.0,
  
  autoMode: true,
  startUpright: false,
  
  loadedModels: {},
  activePolicy: null,
  
  scale: 130, // pixels per meter
  originX: 0,
  originY: 0,
  trailPoints: [],
  maxTrailLen: 100
};

// DOM bindings
const el = {
  modelSelect: document.getElementById('modelSelect'),
  btnAuto: document.getElementById('btnAuto'),
  btnManual: document.getElementById('btnManual'),
  btnPlayPause: document.getElementById('btnPlayPause'),
  btnReset: document.getElementById('btnReset'),
  
  canvas: document.getElementById('simCanvas')
};

// Physics Accelerations
function computeAccelerations(q, qd, force) {
  let result;
  if (state.N === 2) {
    result = getEOM_2pend(q, qd, state.params, force);
  } else {
    result = getEOM_3pend(q, qd, state.params, force);
  }
  return solveLinearSystem(result.A, result.B);
}

// Derivative vector
function systemDerivative(Y, force) {
  const size = state.N + 1;
  const q = Y.slice(0, size);
  const qd = Y.slice(size);
  const qdd = computeAccelerations(q, qd, force);
  return [...qd, ...qdd];
}

// Integration
function stepRK4(dt, force) {
  const size = state.N + 1;
  const Y = [...state.q, ...state.qd];
  
  const k1 = systemDerivative(Y, force);
  
  const Y_k2 = Y.map((y, i) => y + 0.5 * dt * k1[i]);
  const k2 = systemDerivative(Y_k2, force);
  
  const Y_k3 = Y.map((y, i) => y + 0.5 * dt * k2[i]);
  const k3 = systemDerivative(Y_k3, force);
  
  const Y_k4 = Y.map((y, i) => y + dt * k3[i]);
  const k4 = systemDerivative(Y_k4, force);
  
  for (let i = 0; i < Y.length; i++) {
    Y[i] += (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
  }
  
  state.q = Y.slice(0, size);
  state.qd = Y.slice(size);
}

function getScreenCoordinates() {
  const positions = [{ x: state.originX + state.q[0] * state.scale, y: state.originY }];
  const L = state.params.L;
  let cx = state.q[0];
  let cz = 0;
  for (let i = 1; i <= state.N; i++) {
    cx += L * Math.sin(state.q[i]);
    cz += L * Math.cos(state.q[i]);
    positions.push({
      x: state.originX + cx * state.scale,
      y: state.originY - cz * state.scale
    });
  }
  return positions;
}

function resetSim() {
  const N = state.N;
  state.q = new Float32Array(N + 1);
  state.qd = new Float32Array(N + 1);
  
  state.q[0] = (Math.random() - 0.5) * 0.08;
  state.qd[0] = (Math.random() - 0.5) * 0.08;
  
  const range = 0.04;
  if (state.startUpright) {
    for (let i = 1; i <= N; i++) {
      state.q[i] = (Math.random() - 0.5) * range;
      state.qd[i] = (Math.random() - 0.5) * range;
    }
  } else {
    const piNoise = Math.PI + (Math.random() - 0.5) * range;
    for (let i = 1; i <= N; i++) {
      state.q[i] = piNoise + (Math.random() - 0.5) * range;
      state.qd[i] = (Math.random() - 0.5) * range;
    }
  }
  
  state.f_applied = 0.0;
  state.stepCount = 0;
  state.cumulativeReward = 0.0;
  state.trailPoints = [];
  
  if (state.activePolicy) {
    state.activePolicy.reset();
  }
}

async function selectModel(name) {
  state.modelName = name;
  state.N = name === '2pend' ? 2 : 3;
  state.scale = name === '2pend' ? 200 : 150;
  
  if (el.txtAngle3Row) {
    el.txtAngle3Row.style.display = state.N === 3 ? 'table-row' : 'none';
  }
  
  if (state.loadedModels[name]) {
    state.activePolicy = state.loadedModels[name];
    resetSim();
    return;
  }
  
  try {
    const jsonResponse = await fetch(`policy_${name}.json`);
    const metadata = await jsonResponse.json();
    const binResponse = await fetch(`policy_${name}.bin`);
    const buffer = await binResponse.arrayBuffer();
    
    const weights = {};
    for (const [key, info] of Object.entries(metadata)) {
      const floatArray = new Float32Array(buffer, info.offset, info.size);
      weights[key] = floatArray;
    }
    
    state.loadedModels[name] = new PolicyNet(name, { metadata, weights });
    state.activePolicy = state.loadedModels[name];
    resetSim();
  } catch (err) {
    console.error("Failed to load weights for portfolio pendulum:", err);
  }
}

function computeReward() {
  if (state.N === 2) {
    const t1 = wrapAngle(state.q[1]);
    const t2 = wrapAngle(state.q[2]);
    const h1 = Math.abs(t1) < Math.PI / 2 ? (Math.cos(t1) + 1.0) / 2.0 : 0.0;
    const h2 = Math.abs(t2) < Math.PI / 2 ? (Math.cos(t2) + 1.0) / 2.0 : 0.0;
    return (h1 + h2) / 2.0;
  } else {
    const t1 = wrapAngle(state.q[1]);
    const t2 = wrapAngle(state.q[2]);
    const t3 = wrapAngle(state.q[3]);
    const h1 = Math.abs(t1) < Math.PI / 2 ? (Math.cos(t1) + 1.0) / 2.0 : 0.0;
    const h2 = Math.abs(t2) < Math.PI / 2 ? (Math.cos(t2) + 1.0) / 2.0 : 0.0;
    const h3 = Math.abs(t3) < Math.PI / 2 ? (Math.cos(t3) + 1.0) / 2.0 : 0.0;
    return (h1 + h2 + h3) / 3.0;
  }
}

function getPolicyControlForce() {
  if (!state.activePolicy) return 0.0;
  
  const obs = new Float32Array(state.activePolicy.obsDim);
  obs[0] = state.q[0];
  obs[1] = state.qd[0];
  obs[2] = wrapAngle(state.q[1]);
  obs[3] = state.qd[1];
  for (let i = 2; i <= state.N; i++) {
    const idx = 2 + 2 * (i - 1);
    obs[idx] = wrapAngle(state.q[i] - state.q[i-1]);
    obs[idx + 1] = state.qd[i] - state.qd[i-1];
  }
  
  const result = state.activePolicy.forward(obs, false);
  // Match trained policy peak force scale (50.0 control magnitude with a motor gear multiplier of 10)
  const maxForce = state.modelName === '4pend' ? 200.0 : 500.0;
  return result.action * maxForce;
}

function drawScene() {
  const canvas = el.canvas;
  const ctx = canvas.getContext('2d');
  
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;
  ctx.scale(dpr, dpr);
  
  const w = rect.width;
  const h = rect.height;
  
  // Set scale dynamically so that the entire [-12, 12] meter track fits on the screen with margin
  state.scale = (w - 60) / 24;
  
  state.originX = w / 2;
  state.originY = h / 2 + 60;
  
  // Background
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0, 0, w, h);
  
  // Subtle grid
  ctx.strokeStyle = '#f1f3f5';
  ctx.lineWidth = 1;
  const gridSpace = 40;
  for (let x = 0; x < w; x += gridSpace) {
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();
  }
  for (let y = 0; y < h; y += gridSpace) {
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }
  
  // Track
  ctx.strokeStyle = '#e2e8f0';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(state.originX - 12 * state.scale, state.originY);
  ctx.lineTo(state.originX + 12 * state.scale, state.originY);
  ctx.stroke();
  
  // Limits
  ctx.strokeStyle = '#fee2e2';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(state.originX - 12 * state.scale, 0);
  ctx.lineTo(state.originX - 12 * state.scale, h);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(state.originX + 12 * state.scale, 0);
  ctx.lineTo(state.originX + 12 * state.scale, h);
  ctx.stroke();
  
  const joints = getScreenCoordinates();
  const cartPos = joints[0];
  const tipPos = joints[joints.length - 1];
  
  // Record trail
  if (!state.paused) {
    state.trailPoints.push({ x: tipPos.x, y: tipPos.y });
    if (state.trailPoints.length > state.maxTrailLen) {
      state.trailPoints.shift();
    }
  }
  
  // Draw trail
  if (state.trailPoints.length > 1) {
    ctx.lineWidth = 2;
    ctx.strokeStyle = 'rgba(139, 92, 246, 0.25)';
    ctx.beginPath();
    ctx.moveTo(state.trailPoints[0].x, state.trailPoints[0].y);
    for (let i = 1; i < state.trailPoints.length; i++) {
      ctx.lineTo(state.trailPoints[i].x, state.trailPoints[i].y);
    }
    ctx.stroke();
  }
  
  // Draw Cart
  const cartW = 0.35 * state.scale;
  const cartH = 0.15 * state.scale;
  ctx.fillStyle = '#2d3748';
  ctx.beginPath();
  ctx.roundRect(cartPos.x - cartW/2, cartPos.y - cartH/2, cartW, cartH, 3);
  ctx.fill();
  
  // Draw Links
  const linkColors = ['#e53e3e', '#3182ce', '#dd6b20'];
  for (let i = 1; i <= state.N; i++) {
    const p1 = joints[i-1];
    const p2 = joints[i];
    ctx.strokeStyle = linkColors[(i-1) % linkColors.length];
    ctx.lineWidth = 4.5;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(p1.x, p1.y);
    ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
  }
  
  // Draw Hinge circles
  ctx.fillStyle = '#ffffff';
  ctx.strokeStyle = '#2d3748';
  ctx.lineWidth = 2;
  for (let i = 0; i < joints.length; i++) {
    const p = joints[i];
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4.5, 0, 2*Math.PI);
    ctx.fill();
    ctx.stroke();
  }
}



function setupEventListeners() {
  el.modelSelect.addEventListener('change', (e) => {
    selectModel(e.target.value);
  });
  
  el.btnAuto.addEventListener('click', () => {
    state.autoMode = true;
    el.btnAuto.classList.add('active');
    el.btnManual.classList.remove('active');
  });
  
  el.btnManual.addEventListener('click', () => {
    state.autoMode = false;
    el.btnManual.classList.add('active');
    el.btnAuto.classList.remove('active');
  });
  
  el.btnPlayPause.addEventListener('click', () => {
    state.paused = !state.paused;
    el.btnPlayPause.textContent = state.paused ? 'Play' : 'Pause';
  });
  
  el.btnReset.addEventListener('click', resetSim);
  
  // Arrow keys manual input
  const keys = {};
  window.addEventListener('keydown', (e) => {
    keys[e.key] = true;
    if (['ArrowLeft', 'ArrowRight', ' '].includes(e.key)) {
      e.preventDefault();
    }
  });
  window.addEventListener('keyup', (e) => {
    keys[e.key] = false;
  });
  state.keys = keys;
}

let lastTime = 0;
const SIM_DT = 0.02;

function animLoop(timestamp) {
  if (!lastTime) lastTime = timestamp;
  let elapsed = (timestamp - lastTime) / 1000;
  lastTime = timestamp;
  if (elapsed > 0.1) elapsed = 0.1;
  
  if (!state.paused && state.q) {
    const steps = Math.round(elapsed / SIM_DT);
    for (let step = 0; step < Math.min(4, steps); step++) {
      let force = 0;
      if (state.autoMode) {
        force = getPolicyControlForce();
      } else {
        if (state.keys['ArrowLeft']) force = -50.0;
        if (state.keys['ArrowRight']) force = 50.0;
      }
      
      state.f_applied = force;
      stepRK4(SIM_DT, force);
      state.stepCount++;
      
      if (Math.abs(state.q[0]) > 12.0) {
        resetSim();
        break;
      }
    }
  }
  
  if (state.q) {
    drawScene();
  }
  
  requestAnimationFrame(animLoop);
}

async function init() {
  setupEventListeners();
  await selectModel('3pend');
  requestAnimationFrame(animLoop);
}

window.onload = init;
