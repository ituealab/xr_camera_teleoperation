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
  return true;
});
if (ok !== true) { console.log('[emulator]', ok); await browser.close(); process.exit(1); }
await page.getByRole('button', { name: /virtual reality/i }).first().click();
console.log(`[emulator] in emulated VR at ${url}, playing ${seconds} s`);

await page.evaluate((seconds) => new Promise((resolve) => {
  const dev = window.__dev, t0 = performance.now();
  const S = { recentered: false, mode: 'controller', aHeld: false, left: true };
  const quatYaw = (a) => [0, Math.sin(a / 2), 0, Math.cos(a / 2)];
  const step = () => {
    const t = (performance.now() - t0) / 1000;
    if (t > seconds) { resolve(); return; }
    const yaw = (30 * Math.PI / 180) * Math.sin(2 * Math.PI * 0.1 * t);
    const y = 1.65 - (t >= 20 && t < 23 ? 0.2 : 0);
    dev.position.set(0.03 * Math.sin(2 * Math.PI * 0.3 * t), y, 0.02 * Math.cos(2 * Math.PI * 0.2 * t));
    dev.quaternion.set(...quatYaw(yaw));
    if (t >= 28 && !S.recentered) { dev.recenter(); S.recentered = true; }
    const mode = t >= 15 && t < 25 ? 'hand' : 'controller';
    if (mode !== S.mode) { dev.primaryInputMode = mode; S.mode = mode; }
    const r = dev.controllers.right;
    r.position.set(0.2 + 0.02 * Math.sin(2 * Math.PI * 0.5 * t), 1.1, -0.35);
    r.updateAxes('thumbstick', 0, t >= 5 && t < 8 ? -1 : 0);
    const a = t >= 10 && t < 11;
    if (a !== S.aHeld) { r.updateButtonValue('a-button', a ? 1 : 0); S.aHeld = a; }
    for (const [side, h] of Object.entries(dev.hands)) {
      h.position.set((side === 'left' ? -0.2 : 0.2) + 0.03 * Math.sin(2 * Math.PI * 0.5 * t), 1.1, -0.3);
    }
    const l = dev.controllers.left;
    l.position.set(-0.2, 1.1 + 0.02 * Math.sin(2 * Math.PI * 0.4 * t), -0.35);
    if (t >= 32 && S.left) { l.connected = false; S.left = false; }
    setTimeout(step, 5);
  };
  step();
}), seconds);
console.log('[emulator] done');
await browser.close();
