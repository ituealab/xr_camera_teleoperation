"""The one place a Vuer app for the Quest is built, shared by the Quest server and the input probe.

Runs in data/quest-env (vuer==0.1.6, envs/quest-env.freeze.txt). Vuer serves its client build at / and the
`workspace` directory at /workspace to the whole network: it gets an empty directory, never the repo (the TLS key
lives under data/).

Reconnects: the 0.1.6 client retries a dropped websocket 5 times (after 1, 2, 4, 8, 16 s) and then gives up; the
`?reconnect=` query that 0.0.60 honoured is gone. A server that is down for more than ~30 s needs the headset page
reopened (deploy/quest_adb.sh open) and Enter VR pressed again.
"""
from __future__ import annotations

from pathlib import Path

VUER_VERSION = "0.1.6"


def make_vuer_app(port: int, cert: str, key: str, workspace: Path):
    from importlib.metadata import version

    found = version("vuer")
    if found != VUER_VERSION:
        raise RuntimeError(f"vuer {found} found, {VUER_VERSION} expected: run with data/quest-env")
    from vuer import Vuer

    workspace.mkdir(parents=True, exist_ok=True)
    return Vuer(host="0.0.0.0", port=port, cert=cert, key=key, queries=dict(grid=False), queue_len=3,
                workspace=str(workspace))
