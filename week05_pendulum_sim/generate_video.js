const fs = require('fs');
const path = require('path');

// Physics constants (matching the HTML)
const g = 9.8, m1 = 10, m2 = 10, L1 = 140, L2 = 140;
const DEG = Math.PI / 180;

// Double pendulum equations of motion
function accel(a1, w1, a2, w2) {
  const da = a1 - a2;
  const sinDa = Math.sin(da), cosDa = Math.cos(da);
  const sin1 = Math.sin(a1), cos1 = Math.cos(a1);
  const M = m1 + m2;
  const denom = 2 * M - 2 * m2 * cosDa * cosDa;
  
  const dw1 = (-g * (2*m1 + m2) * sin1 - m2 * g * Math.sin(a1 - 2*a2) 
    - 2 * sinDa * m2 * (w2*w2*L2 + w1*w1*L1*cosDa)) / (L1 * denom);
  
  const dw2 = (2 * sinDa * (w1*w1*L1*M + g * M * cos1 
    + w2*w2*L2*m2*cosDa)) / (L2 * denom);
  
  return [w1, dw1, w2, dw2];
}

// RK4 integrator
function rk4(s, dt) {
  const k1 = accel(s[0], s[1], s[2], s[3]);
  const s2 = s.map((v,i) => v + k1[i]*dt/2);
  const k2 = accel(s2[0], s2[1], s2[2], s2[3]);
  const s3 = s.map((v,i) => v + k2[i]*dt/2);
  const k3 = accel(s3[0], s3[1], s3[2], s3[3]);
  const s4 = s.map((v,i) => v + k3[i]*dt);
  const k4 = accel(s4[0], s4[1], s4[2], s4[3]);
  return s.map((v,i) => v + (k1[i]+2*k2[i]+2*k3[i]+k4[i])*dt/6);
}

// Calculate bob positions
function bobPos(s) {
  const [a1, , a2] = s;
  const pivotX = 350, pivotY = 160;
  const x1 = pivotX + L1 * Math.sin(a1);
  const y1 = pivotY + L1 * Math.cos(a1);
  const x2 = x1 + L2 * Math.sin(a2);
  const y2 = y1 + L2 * Math.cos(a2);
  return { x1, y1, x2, y2, pivotX, pivotY };
}

// Generate frames
const duration = 10; // seconds
const fps = 30;
const dt = 0.002; // physics timestep
const stepsPerFrame = Math.round(1 / (fps * dt));

let state = [120*DEG, 0, 120*DEG, 0];
const frames = [];

console.log(`Generating ${duration * fps} frames at ${fps} fps...`);

for (let frame = 0; frame < duration * fps; frame++) {
  // Advance physics
  for (let i = 0; i < stepsPerFrame; i++) {
    state = rk4(state, dt);
  }
  
  const pos = bobPos(state);
  frames.push({
    frame,
    time: frame / fps,
    ...pos,
    a1: state[0] / DEG,
    a2: state[2] / DEG
  });
}

// Save frame data
const outputDir = path.join(__dirname, 'frames');
if (!fs.existsSync(outputDir)) {
  fs.mkdirSync(outputDir);
}

fs.writeFileSync(
  path.join(outputDir, 'simulation_data.json'),
  JSON.stringify(frames, null, 2)
);

console.log(`Saved ${frames.length} frames to frames/simulation_data.json`);
console.log('Sample frame:', frames[0]);
