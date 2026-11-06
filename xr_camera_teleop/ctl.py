"""Small local control plane shared by the operator and robot programs.

* CtlServer: ZMQ REP on an abstract unix socket `ipc://@xrcam_<name>` (Linux), served from a thread. Replaces
  upstream's xr_ctl.py / IPC_Server: `xrcam-ctl robot stop`, `xrcam-ctl operator start`, ...
* KeyReader: single-key reader on a TTY stdin (cbreak mode), no extra dependency; does nothing without a TTY.

Handlers are called from the server thread: they must only set flags / put items on queues.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading

import zmq


def endpoint(name: str) -> str:
    return f"ipc://@xrcam_{name}"


class CtlServer:
    def __init__(self, name: str, handlers: dict):
        self.name, self.handlers = name, handlers
        self._stop = threading.Event()
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.REP)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(endpoint(name))          # raises ZMQError if another instance owns the name
        self._thread = threading.Thread(target=self._serve, daemon=True, name=f"ctl-{name}")

    def start(self):
        self._thread.start()
        return self

    def _serve(self):
        while not self._stop.is_set():
            if not self._sock.poll(100):
                continue
            try:
                req = json.loads(self._sock.recv())
                cmd = str(req.get("cmd", ""))
                h = self.handlers.get(cmd)
                rep = {"ok": False, "error": f"unknown command {cmd!r}; have {sorted(self.handlers)}"} if h is None \
                    else {"ok": True, **(h(req) or {})}
            except Exception as e:  # noqa: BLE001
                rep = {"ok": False, "error": repr(e)}
            self._sock.send(json.dumps(rep, default=str).encode())

    def close(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._sock.close(0)
        self._ctx.term()


def request(name: str, cmd: str, timeout_ms: int = 2000, **kw) -> dict:
    ctx = zmq.Context()
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.LINGER, 0)
    s.connect(endpoint(name))
    try:
        s.send(json.dumps({"cmd": cmd, **kw}).encode())
        if not s.poll(timeout_ms):
            return {"ok": False, "error": f"no answer from '{name}' (is it running on this machine?)"}
        return json.loads(s.recv())
    finally:
        s.close(0)
        ctx.term()


class KeyReader:
    """Calls on_key(ch) for every key pressed on a TTY stdin. Restores the terminal on stop()."""

    def __init__(self, on_key):
        self.on_key = on_key
        self._stop = threading.Event()
        self._thread = None
        self._old = None

    def start(self):
        if not sys.stdin.isatty():
            return self
        import termios
        import tty
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._thread = threading.Thread(target=self._run, daemon=True, name="keys")
        self._thread.start()
        return self

    def _run(self):
        import select
        while not self._stop.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1)
                if ch:
                    self.on_key(ch)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._old is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
            self._old = None


def main(argv=None):
    ap = argparse.ArgumentParser(description="Send a command to a running xrcam-robot / xrcam-operator / xrcam-ui "
                                             "on THIS machine.")
    ap.add_argument("target", choices=["robot", "operator"])
    ap.add_argument("cmd", help="robot: status|stop|home   operator: status|start|stop|home|calibrate|reset|quit")
    ap.add_argument("--timeout", type=float, default=2.0)
    a = ap.parse_args(argv)
    rep = request(a.target, a.cmd, int(a.timeout * 1000))
    print(json.dumps(rep, indent=1, default=str))
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())
