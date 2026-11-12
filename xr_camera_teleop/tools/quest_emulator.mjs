// A scripted, emulated Quest for the Quest server: headless Chrome opens the Vuer page, Meta's IWER (bundled in
// Vuer's client) stands in for WebXR, and a fixed operator script moves the head and controllers, switches to
// hands, presses buttons and recenters. Exercises the real client -> websocket -> server -> xr_frame path without a
// headset. Rates are headless Chrome's (software GL, ~40 Hz), not the Quest's.
//
//   node xr_camera_teleop/tools/quest_emulator.mjs [--url https://localhost:8012/] [--seconds 40]
//
// Needs google-chrome and Playwright (data/browser-tools: `npm install` there). The script it plays, by time t (s):
//   always   head at 1.65 m swaying +-3 cm, yaw +-30 deg at 0.1 Hz; controllers and hands in front of the chest
//   5-8      right stick pushed forward (WebXR y = -1)
//   8-13     the sway eases out (8-9) and back in (12-13): the head is still, upright and looking ahead from 9 to 12
//   10-11.5  A held on the right controller: a calibration (head still >= 1 s, A held >= 1 s; fullbody/calibration.py)
//   15-25    hands instead of controllers (IWER can only switch both sides at once)
//   20-23    head 0.2 m lower (crouch)                               28     recenter (Quest long-press)
//   32-      left controller dropped out
// At the end it prints the XR sessions started and ended and the page loads, so a caller can tell that the immersive
// session lived through the whole run (e.g. a Quest server restart) without a reload or a second Enter VR.
//
//   node xr_camera_teleop/tools/quest_emulator.mjs --puppet 1 [--url ...]
//
// Puppet mode plays no script: a caller (tools/quest_puppet.py) drives the emulated operator with one JSON command
// per stdin line and gets one reply line on stdout starting with "@@ ". Commands:
//   {"id": n, "set": {...}}   deep-merge into the operator state (below), applied every frame from then on
//   {"id": n, "recenter": 1}  Quest recenter          {"id": n, "bitmaps": 1}  image decode log (and clear it)
//   {"id": n, "probe": 1}     XR sessions, XR frames rendered, puppet steps applied, head position
//   {"id": n, "quit": 1}
// State (WebXR world, metres, quaternions [x, y, z, w]):
//   head {pos, quat}, mode "controller" | "hand",
//   controllers.left|right {pos, quat, trigger, squeeze, stick: [x, y], stickPress, a, b, connected},
//   hands.left|right {pos, quat, pinch, pose}   pose: "default" | "point" | "grip" (IWER's own poses, plus "grip": a
//                                               fist, its curled fingers from "point" and its thumb and index from
//                                               "pinch", each expressed relative to its own pose's wrist)
// Every pose gets a sub-millimetre wobble each frame: a real head or hand is never bit-identical twice, and the
// server drops identical resends as no new sample.
// Both modes log every image the page decodes (createImageBitmap: Vuer's path for binary textures) with its byte
// size and the wall-clock times of the call and of the decoded bitmap, for latency measurements.
import { createRequire } from 'node:module';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const repo = path.resolve(here, '../..');
const args = Object.fromEntries(process.argv.slice(2).reduce((acc, a, i, all) => {
  if (a.startsWith('--')) acc.push([a.slice(2), all[i + 1]]);
  return acc;
}, []));
const url = args.url || 'https://localhost:8012/';
const seconds = Number(args.seconds || 40);
const puppet = Boolean(args.puppet);
const chrome = args.chrome || '/usr/bin/google-chrome';
const require = createRequire(path.join(repo, 'data/browser-tools/package.json'));
const { chromium } = require('playwright');

// native WebXR off, so IWER's runtime is the only one. GL: software (SwiftShader) by default, so it runs anywhere;
// --gl NAME picks an ANGLE backend instead (e.g. gl-egl: the GPU), which the robot view needs to keep the emulated
// frame rate (software GL falls to ~2 frames/s with a 2x960x540 30 fps view).
const gl = args.gl || 'swiftshader';
const browser = await chromium.launch({
  executablePath: chrome, headless: true,
  args: ['--ignore-certificate-errors', '--disable-features=WebXR', '--use-gl=angle', `--use-angle=${gl}`,
         ...(gl === 'swiftshader' ? ['--enable-unsafe-swiftshader'] : ['--ignore-gpu-blocklist'])],
});
const page = await browser.newPage({ ignoreHTTPSErrors: true, viewport: { width: 1280, height: 720 } });
page.on('pageerror', (e) => console.log('[emulator] page error:', e.message));
page.on('console', (m) => { if (m.type() === 'error') console.log('[emulator] console error:', m.text().slice(0, 300)); });
await page.addInitScript(() => {
  // every decoded image: (call time, decoded time) on the wall clock, ms, and the blob size
  window.__bitmaps = [];
  const native = window.createImageBitmap.bind(window);
  window.createImageBitmap = (src, ...rest) => {
    const rec = { t_call: performance.timeOrigin + performance.now(), size: src && src.size, t_done: null };
    if (window.__bitmaps.length < 20000) window.__bitmaps.push(rec);
    return native(src, ...rest).then((b) => { rec.t_done = performance.timeOrigin + performance.now(); return b; });
  };
});
let loads = 0;
page.on('load', () => { loads += 1; });
await page.goto(url);
await page.waitForTimeout(4000);

// Install IWER from Vuer's own bundle (Vuer builds its XR store with emulate:false, so it never does this itself).
const ok = await page.evaluate(async () => {
  const sources = performance.getEntriesByType('resource').map((r) => r.name).filter((n) => n.endsWith('.js'));
  let chunk = null;
  for (const src of sources) {
    const m = (await (await fetch(src)).text()).match(/\{emulate:\w+\}=await import\("\.\/(chunk-[^"]+\.js)"\)/);
    if (m) { chunk = new URL(m[1], src).href; break; }
  }
  if (!chunk) return 'IWER chunk not found in the Vuer client';
  const { emulate } = await import(chunk);
  const dev = emulate({ type: 'metaQuest3', syntheticEnvironment: false });
  if (dev.devui) dev.devui.render = () => {};          // its own three.js instance clashes with Vuer's
  // IWER removes XRWebGLBinding without the layers polyfill; three.js only checks its prototype
  Object.defineProperty(window, 'XRWebGLBinding', { value: function () {}, writable: true, configurable: true });
  window.__dev = dev;
  window.__xr = { started: 0, ended: 0 };
  const request = navigator.xr.requestSession.bind(navigator.xr);
  navigator.xr.requestSession = async (...a) => {
    const session = await request(...a);
    window.__xr.started += 1;
    session.addEventListener('end', () => { window.__xr.ended += 1; });
    return session;
  };
  return true;
});
if (ok !== true) { console.log('[emulator]', ok); await browser.close(); process.exit(1); }
// Enter VR, and make sure the immersive session really renders: under load (SONIC loading TensorRT on the same GPU)
// a click can come before the page is ready and start nothing. Frames are counted on the session's own rAF.
await page.evaluate(() => {
  window.__xrFrames = 0;
  const request = navigator.xr.requestSession;
  navigator.xr.requestSession = async (...a) => {
    const session = await request(...a);
    const tick = () => { window.__xrFrames += 1; session.requestAnimationFrame(tick); };
    session.requestAnimationFrame(tick);
    return session;
  };
});
let rendering = false;
for (let attempt = 1; attempt <= 4 && !rendering; attempt += 1) {
  await page.getByRole('button', { name: /virtual reality/i }).first().click();
  for (let k = 0; k < 40 && !rendering; k += 1) {
    await page.waitForTimeout(250);
    rendering = await page.evaluate(() => window.__xrFrames > 10);
  }
  if (!rendering) console.log(`[emulator] Enter VR attempt ${attempt}: no XR frames yet`);
}
if (!rendering) { console.log('[emulator] the immersive session does not render'); await browser.close(); process.exit(1); }
if (puppet) {
  await runPuppet();
  await browser.close();
  process.exit(0);
}
console.log(`[emulator] in emulated VR at ${url}, playing ${seconds} s`);

await page.evaluate((seconds) => new Promise((resolve) => {
  const dev = window.__dev, t0 = performance.now();
  const S = { mode: 'controller', aHeld: false, left: true };
  // After the recenter the script keeps playing the same room motion, seen from the new origin: a Quest recenter
  // moves local-floor's origin under the head and turns it to the head's heading (rc).
  let rc = null;
  const quatYaw = (a) => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
  const place = (obj, x, y, z, yaw = 0) => {
    if (rc) {
      const dx = x - rc.x, dz = z - rc.z, c = Math.cos(-rc.yaw), s = Math.sin(-rc.yaw);
      [x, z, yaw] = [c * dx + s * dz, -s * dx + c * dz, yaw - rc.yaw];   // rotation about +Y by -rc.yaw
    }
    obj.position.set(x, y, z);
    obj.quaternion.set(...quatYaw(yaw));
  };
  const step = () => {
    const t = (performance.now() - t0) / 1000;
    if (t > seconds) { resolve(); return; }
    // sway envelope: 1, eased to 0 over 8-9 s, 0 until 12 s, back to 1 over 12-13 s; a still head still gets a
    // sub-millimetre wobble (the server drops bit-identical resends as no new sample)
    const ease = (u) => 0.5 - 0.5 * Math.cos(Math.PI * Math.min(1, Math.max(0, u)));
    const env = t < 9 ? 1 - ease(t - 8) : ease(t - 12);
    const wob = 0.0003 * Math.sin(2 * Math.PI * 1.3 * t);
    const yaw = env * (30 * Math.PI / 180) * Math.sin(2 * Math.PI * 0.1 * t);
    const hx = env * 0.03 * Math.sin(2 * Math.PI * 0.3 * t) + wob, hz = env * 0.02 * Math.cos(2 * Math.PI * 0.2 * t);
    if (t >= 28 && !rc) { rc = { x: hx, z: hz, yaw }; dev.recenter(); }
    place(dev, hx, 1.65 - (t >= 20 && t < 23 ? 0.2 : 0), hz, yaw);
    const mode = t >= 15 && t < 25 ? 'hand' : 'controller';
    if (mode !== S.mode) { dev.primaryInputMode = mode; S.mode = mode; }
    const r = dev.controllers.right;
    place(r, 0.2 + 0.02 * Math.sin(2 * Math.PI * 0.5 * t), 1.1, -0.35);
    r.updateAxes('thumbstick', 0, t >= 5 && t < 8 ? -1 : 0);
    const a = t >= 10 && t < 11.5;
    if (a !== S.aHeld) { r.updateButtonValue('a-button', a ? 1 : 0); S.aHeld = a; }
    for (const [side, h] of Object.entries(dev.hands)) {
      place(h, (side === 'left' ? -0.2 : 0.2) + 0.03 * Math.sin(2 * Math.PI * 0.5 * t), 1.1, -0.3);
    }
    place(dev.controllers.left, -0.2, 1.1 + 0.02 * Math.sin(2 * Math.PI * 0.4 * t), -0.35);
    if (t >= 32 && S.left) { dev.controllers.left.connected = false; S.left = false; }
    setTimeout(step, 5);
  };
  step();
}), seconds);
const xr = await page.evaluate(() => window.__xr);
console.log(`[emulator] done: XR sessions started ${xr.started} ended ${xr.ended}, page loads ${loads}`);
await browser.close();

async function runPuppet() {
  await page.evaluate(() => {
    const dev = window.__dev;
    const S = window.__S = {
      head: { pos: [0, 1.65, 0], quat: [0, 0, 0, 1] }, mode: 'controller',
      controllers: {
        left: { pos: [-0.2, 1.1, -0.35], quat: [0, 0, 0, 1], trigger: 0, squeeze: 0, stick: [0, 0], stickPress: false,
                a: false, b: false, connected: true },
        right: { pos: [0.2, 1.1, -0.35], quat: [0, 0, 0, 1], trigger: 0, squeeze: 0, stick: [0, 0], stickPress: false,
                 a: false, b: false, connected: true },
      },
      hands: { left: { pos: [-0.2, 1.1, -0.3], quat: [0, 0, 0, 1], pinch: 0, pose: 'default' },
               right: { pos: [0.2, 1.1, -0.3], quat: [0, 0, 0, 1], pinch: 0, pose: 'default' } },
    };
    window.__merge = function merge(dst, src) {
      for (const [k, v] of Object.entries(src)) {
        if (v && typeof v === 'object' && !Array.isArray(v) && dst[k] && typeof dst[k] === 'object') merge(dst[k], v);
        else dst[k] = v;
      }
    };
    // "grip": compose a fist from IWER's own hand poses (column-major 4x4 offset matrices relative to the hand)
    const mul = (a, b) => { const o = new Array(16).fill(0);
      for (let c = 0; c < 4; c += 1) for (let r = 0; r < 4; r += 1) for (let k = 0; k < 4; k += 1) o[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
      return o; };
    const inv = (m) => { const o = new Array(16).fill(0);            // rigid: R^T, -R^T t
      for (let r = 0; r < 3; r += 1) for (let c = 0; c < 3; c += 1) o[c * 4 + r] = m[r * 4 + c];
      for (let r = 0; r < 3; r += 1) o[12 + r] = -(o[r] * m[12] + o[4 + r] * m[13] + o[8 + r] * m[14]);
      o[15] = 1; return o; };
    for (const side of ['left', 'right']) {
      const h = dev.hands[side];
      const sym = Object.getOwnPropertySymbols(h).find((k) => h[k] && h[k].poses);
      const P = h[sym].poses;
      const fromPinch = mul(P.point.jointTransforms.wrist.offsetMatrix, inv(P.pinch.jointTransforms.wrist.offsetMatrix));
      const joints = {};
      for (const [j, v] of Object.entries(P.point.jointTransforms)) {
        const pinchJ = j.startsWith('thumb') || j.startsWith('index');
        joints[j] = pinchJ ? { ...P.pinch.jointTransforms[j], offsetMatrix: mul(fromPinch, P.pinch.jointTransforms[j].offsetMatrix) }
          : { ...v };
      }
      P.grip = { ...P.point, jointTransforms: joints };
    }
    const last = { mode: 'controller', buttons: {}, pinch: {}, pose: {}, connected: { left: true, right: true } };
    const t0 = performance.now();
    const put = (obj, pose, k) => {
      const t = (performance.now() - t0) / 1000, w = 0.0003 * Math.sin(2 * Math.PI * (1.3 + 0.17 * k) * t);
      obj.position.set(pose.pos[0] + w, pose.pos[1] + 0.7 * w, pose.pos[2] - 0.5 * w);
      obj.quaternion.set(...pose.quat);
    };
    const button = (c, side, id, v) => {
      const key = side + id;
      if (last.buttons[key] !== v) { c.updateButtonValue(id, v); last.buttons[key] = v; }
    };
    const step = () => {
      if (S.mode !== last.mode) { dev.primaryInputMode = S.mode; last.mode = S.mode; }
      put(dev, S.head, 0);
      let k = 1;
      for (const side of ['left', 'right']) {
        const c = dev.controllers[side], cs = S.controllers[side];
        if (cs.connected !== last.connected[side]) { c.connected = cs.connected; last.connected[side] = cs.connected; }
        put(c, cs, k++);
        c.updateAxes('thumbstick', cs.stick[0], cs.stick[1]);
        button(c, side, 'trigger', cs.trigger);
        button(c, side, 'squeeze', cs.squeeze);
        button(c, side, 'thumbstick', cs.stickPress ? 1 : 0);
        button(c, side, side === 'right' ? 'a-button' : 'x-button', cs.a ? 1 : 0);
        button(c, side, side === 'right' ? 'b-button' : 'y-button', cs.b ? 1 : 0);
        const h = dev.hands[side], hs = S.hands[side];
        put(h, hs, k++);
        if (last.pinch[side] !== hs.pinch) { h.updatePinchValue(hs.pinch); last.pinch[side] = hs.pinch; }
        if (last.pose[side] !== hs.pose) { h.poseId = hs.pose; last.pose[side] = hs.pose; }
      }
      window.__steps = (window.__steps || 0) + 1;
      setTimeout(step, 5);
    };
    step();
  });
  console.log('@@ ' + JSON.stringify({ ready: true }));
  const readline = await import('node:readline');
  const rl = readline.createInterface({ input: process.stdin });
  for await (const line of rl) {
    if (!line.trim()) continue;
    let cmd;
    try { cmd = JSON.parse(line); } catch (e) { console.log('@@ ' + JSON.stringify({ error: String(e) })); continue; }
    const reply = { id: cmd.id };
    if (cmd.set) await page.evaluate((patch) => window.__merge(window.__S, patch), cmd.set);
    if (cmd.recenter) await page.evaluate(() => window.__dev.recenter());
    if (cmd.bitmaps) reply.bitmaps = await page.evaluate(() => window.__bitmaps.splice(0));
    if (cmd.quit) { console.log('@@ ' + JSON.stringify(reply)); break; }
    if (cmd.probe) {
      reply.probe = await page.evaluate(() => ({ xr: window.__xr, xrFrames: window.__xrFrames, puppetSteps: window.__steps,
        head: [window.__dev.position.x, window.__dev.position.y, window.__dev.position.z], loads: performance.getEntriesByType('navigation').length }));
    }
    console.log('@@ ' + JSON.stringify(reply));
  }
}
