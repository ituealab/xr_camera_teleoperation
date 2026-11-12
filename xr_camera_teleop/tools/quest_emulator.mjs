// A scripted, emulated Quest for the Quest server: headless Chrome opens the Vuer page, Meta's IWER (bundled in
// Vuer's client) stands in for WebXR, and a fixed operator script moves the head and controllers, switches to
// hands, presses buttons and recenters. Exercises the real client -> websocket -> server -> xr_frame path without a
// headset. Rates are headless Chrome's (software GL, ~40 Hz), not the Quest's.
//
//   node xr_camera_teleop/tools/quest_emulator.mjs [--url https://localhost:8012/] [--seconds 40]
//
// Needs google-chrome and Playwright (data/browser-tools: `npm install` there). The script it plays, by time t (s):
//   always   head at 1.65 m swaying +-3 cm, yaw +-30 deg at 0.1 Hz; controllers and hands in front of the chest
//   5-8      right stick pushed forward (WebXR y = -1)              10-11  A held on the right controller
//   15-25    hands instead of controllers (IWER can only switch both sides at once)
//   20-23    head 0.2 m lower (crouch)                               28     recenter (Quest long-press)
//   32-      left controller dropped out
// At the end it prints the XR sessions started and ended and the page loads, so a caller can tell that the immersive
// session lived through the whole run (e.g. a Quest server restart) without a reload or a second Enter VR.
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
const chrome = args.chrome || '/usr/bin/google-chrome';
const require = createRequire(path.join(repo, 'data/browser-tools/package.json'));
const { chromium } = require('playwright');

const browser = await chromium.launch({
  executablePath: chrome, headless: true,
  // native WebXR off, so IWER's runtime is the only one; software GL so it runs without a GPU
  args: ['--ignore-certificate-errors', '--disable-features=WebXR', '--use-gl=angle', '--use-angle=swiftshader',
         '--enable-unsafe-swiftshader'],
});
const page = await browser.newPage({ ignoreHTTPSErrors: true, viewport: { width: 1280, height: 720 } });
page.on('pageerror', (e) => console.log('[emulator] page error:', e.message));
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
await page.getByRole('button', { name: /virtual reality/i }).first().click();
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
    const yaw = (30 * Math.PI / 180) * Math.sin(2 * Math.PI * 0.1 * t);
    const hx = 0.03 * Math.sin(2 * Math.PI * 0.3 * t), hz = 0.02 * Math.cos(2 * Math.PI * 0.2 * t);
    if (t >= 28 && !rc) { rc = { x: hx, z: hz, yaw }; dev.recenter(); }
    place(dev, hx, 1.65 - (t >= 20 && t < 23 ? 0.2 : 0), hz, yaw);
    const mode = t >= 15 && t < 25 ? 'hand' : 'controller';
    if (mode !== S.mode) { dev.primaryInputMode = mode; S.mode = mode; }
    const r = dev.controllers.right;
    place(r, 0.2 + 0.02 * Math.sin(2 * Math.PI * 0.5 * t), 1.1, -0.35);
    r.updateAxes('thumbstick', 0, t >= 5 && t < 8 ? -1 : 0);
    const a = t >= 10 && t < 11;
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
