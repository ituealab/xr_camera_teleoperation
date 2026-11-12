"""operator/quest/vuer_app.py: the reconnect script injected into Vuer's page, run in node against a scripted
WebSocket (a server that is down for a long time, comes back, drops again), and its injection into index.html."""
import json
import shutil
import subprocess
from pathlib import Path

import pytest

from xr_camera_teleop.operator.quest.vuer_app import RECONNECT_DIAL_MS, RECONNECT_JS, with_reconnect

REPO = Path(__file__).resolve().parents[2]
CLIENT_INDEX = next((REPO / "data" / "quest-env" / "lib").glob("python*/site-packages/vuer/client_build/index.html"),
                    None) if (REPO / "data" / "quest-env" / "lib").exists() else None

# Runs the injected script in a vm context whose WebSocket is a fake: each dial fails while `serverUp` is false.
# Timers are a manual queue, so "100 s down" is 100 dial periods, run instantly.
HARNESS = r"""
const vm = require('node:vm');
const src = require('node:fs').readFileSync(0, 'utf8').replace(/^<script>|<\/script>$/g, '');
const timers = []; let serverUp = false; const dials = [];
const later = (fn, ms) => { timers.push({ fn, ms }); return timers.length; };
class FakeNative {
  constructor(url, protocols) { this.url = url; this.readyState = 0; this.sent = []; dials.push(this);
    later(() => { if (serverUp) { this.readyState = 1; this.onopen && this.onopen({}); }
                       else { this.readyState = 3; this.onerror && this.onerror({});
                              this.onclose && this.onclose({ code: 1006, reason: '', wasClean: false }); } }, 0); }
  send(d) { this.sent.push(d); }
  close(code) { this.readyState = 3; this.onclose && this.onclose({ code: code || 1005, reason: '', wasClean: true }); }
  drop(withError) { this.readyState = 3; withError && this.onerror && this.onerror({});
                    this.onclose && this.onclose({ code: 1006, reason: 'gone', wasClean: false }); }
}
const window = { WebSocket: FakeNative };
const ctx = vm.createContext({ window, EventTarget, Event, MessageEvent, CloseEvent, DOMException,
  setTimeout: later,
  clearTimeout: (id) => { if (timers[id - 1]) timers[id - 1].fn = null; } });
vm.runInContext(src, ctx);
const run = (n) => { for (let i = 0; i < n; i++) { const t = timers.find((t) => t.fn); if (!t) return; const f = t.fn; t.fn = null; f(); } };
const delays = () => [...new Set(timers.filter((t) => t.ms > 0).map((t) => t.ms))];
const watch = (ws) => { const ev = []; ws.onopen = () => ev.push('open'); ws.onclose = (e) => ev.push('close:' + e.code);
  ws.onerror = () => ev.push('error'); ws.addEventListener('message', (e) => ev.push('message:' + e.data)); return ev; };
const out = {};

const ws = new window.WebSocket('wss://msi:8012', undefined);
out.instanceOf = ws instanceof window.WebSocket && window.WebSocket.OPEN === 1 && ws.CONNECTING === 0;
const ev = watch(ws);
run(200);                                // 100 dials failed (each: the attempt, then the 1 s wait), #101 pending
out.down = { dials: dials.length, state: ws.readyState, events: ev.slice(), delays: delays() };
let threw = false; try { ws.send('x'); } catch (e) { threw = e.name === 'InvalidStateError'; }
out.sendWhileDown = threw;
serverUp = true; run(2);
out.up = { state: ws.readyState, events: ev.slice() };
ws.send('init'); dials.at(-1).onmessage({ data: 'scene' });
out.traffic = { sent: dials.at(-1).sent, events: ev.slice() };
const before = dials.length; dials.at(-1).drop(true); run(10);
out.drop = { events: ev.slice(), newDials: dials.length - before };

serverUp = true;                                          // a drop the browser reports without an error event
const ws3 = new window.WebSocket('wss://msi:8012'); const ev3 = watch(ws3); run(2); dials.at(-1).drop(false);
out.quietDrop = ev3;
const ws4 = new window.WebSocket('wss://msi:8012'); const ev4 = watch(ws4); run(2); ws4.close(1000);
out.closedByClient = ev4;

serverUp = false;
const ws2 = new window.WebSocket('wss://msi:8012'); const ev2 = watch(ws2);
run(20); const n2 = dials.length; ws2.close(); run(20);
out.closedWhileDialling = { state: ws2.readyState, events: ev2, newDials: dials.length - n2 };
console.log(JSON.stringify(out));
"""


@pytest.fixture(scope="module")
def shim():
    if not shutil.which("node"):
        pytest.skip("needs node")
    r = subprocess.run(["node", "-e", HARNESS], input=RECONNECT_JS, capture_output=True, text=True, timeout=30)
    assert r.returncode == 0, r.stderr
    return json.loads(r.stdout)


def test_long_outage_is_one_attempt_that_stays_connecting(shim):
    assert shim["instanceOf"]
    down = shim["down"]
    assert down["dials"] == 101 and down["state"] == 0          # CONNECTING the whole time
    assert down["events"] == []                                 # the client never sees a failed attempt
    assert down["delays"] == [RECONNECT_DIAL_MS]
    assert shim["sendWhileDown"]


def test_opens_when_the_server_is_back_and_is_transparent_after(shim):
    assert shim["up"] == {"state": 1, "events": ["open"]}
    assert shim["traffic"]["sent"] == ["init"] and shim["traffic"]["events"][-1] == "message:scene"
    drop = shim["drop"]
    assert drop["events"][-2:] == ["error", "close:1006"]       # the client sees the real drop and retries itself
    assert drop["newDials"] == 0                                # an opened wrapper never re-dials on its own


def test_every_drop_reaches_the_client_as_error_then_close(shim):
    # the 0.1.6 client retries only on `error`; Chrome reports a server that exits as close 1006 alone
    assert shim["quietDrop"] == ["open", "error", "close:1006"]
    assert shim["closedByClient"] == ["open", "close:1000"]    # a close the client asked for stays a plain close


def test_close_while_dialling_stops_quietly(shim):
    assert shim["closedWhileDialling"] == {"state": 3, "events": [], "newDials": 0}


def test_injected_first_in_head():
    html = "<!DOCTYPE html><html><head><title>x</title></head><body><script src=a.js type=module></script></body>"
    out = with_reconnect(html)
    assert out.startswith("<!DOCTYPE html><html><head>" + RECONNECT_JS + "<title>")
    assert out.count("window.WebSocket = PatientWebSocket") == 1
    with pytest.raises(RuntimeError):
        with_reconnect("<html><body></body></html>")


@pytest.mark.skipif(CLIENT_INDEX is None, reason="needs data/quest-env (Vuer 0.1.6's bundled client)")
def test_bundled_client_page_takes_the_script():
    out = with_reconnect(CLIENT_INDEX.read_text(encoding="utf-8"))
    assert out.index(RECONNECT_JS) < out.index('type="module"')     # runs before the client's bundle
