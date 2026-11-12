"""The panel link between the Quest server and fullbody_node (PLAN §3, §6a). Wire format and delivery rules. Pure.

    Quest server (operator/quest/ui.py)  ──actions, PUB node.ui_addr (:5597)──►  fullbody_node
                                         ◄──status, PUB node.status_addr (:5595)──

Two msgpack messages, one part each, no topic (configs/fullbody.yaml `node:` holds both addresses; the server takes
them as --ui-zmq / --node-status):

actions   {"v": 1, "kind": "ui_actions", "server": str, "actions": [UiAction as a dict, oldest first]}
          The server binds the PUB. A panel action is a discrete event over a lossy PUB/SUB link, so every message
          carries *all* actions confirmed in the last `ActionOutbox.resend_s`, one message per new action and a
          heartbeat every `heartbeat_s` while any is that recent: a dropped message is repaired by the next one.
          `server` names the server run; `seq` counts per server run, so a restarted server is a new sequence.
          The node keeps each (server, seq) once (`ActionInbox`) and drops actions older than `max_age_s` on
          arrival (both ends read the MSI wall clock; PC2's clock follows MSI's, P2), so an action is never
          applied twice and a node started later never acts on a panel press it did not see happen.

status    {"v": 1, "kind": "node_status", **NodeStatus}
          The node binds the PUB and sends every node.status_period_s; the server keeps the latest and shows it on
          the panel and the status strip, and serves it at GET /state. A status older than STATUS_STALE_S means the
          node is gone: the panel then refuses everything that needs it (Durdur still goes out).
"""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, fields
from typing import Optional

import msgpack

from xr_camera_teleop.fullbody.settings import UiAction, UiKind

VERSION = 1
STATUS_STALE_S = 1.0


class LinkError(ValueError):
    """A message that is not a valid panel-link message."""


# ---------------------------------------------------------------------------------------------------- UiAction

def action_to_dict(a: UiAction) -> dict:
    return {"kind": a.kind.value, "key": a.key, "value": a.value, "t": float(a.t), "seq": int(a.seq)}


def action_from_dict(d: dict) -> UiAction:
    try:
        return UiAction(kind=UiKind(d["kind"]), key=d.get("key"), value=d.get("value"), t=float(d["t"]),
                        seq=int(d["seq"]))
    except (KeyError, TypeError, ValueError) as e:
        raise LinkError(f"bad UiAction {d!r}: {e}") from e


def _unpack(raw: bytes, kind: str) -> dict:
    try:
        m = msgpack.unpackb(raw, raw=False)
    except (ValueError, msgpack.ExtraData, msgpack.FormatError, msgpack.StackError) as e:
        raise LinkError(f"not msgpack: {e}") from e
    if not isinstance(m, dict) or m.get("kind") != kind or m.get("v") != VERSION:
        raise LinkError(f"not a v{VERSION} {kind} message: {str(m)[:120]}")
    return m


class ActionOutbox:
    """Server side: numbers confirmed actions and builds the messages that carry them."""

    def __init__(self, server: str, resend_s: float = 2.0, heartbeat_s: float = 0.2):
        self.server, self.resend_s, self.heartbeat_s = server, resend_s, heartbeat_s
        self._recent: deque = deque()
        self._seq = 0
        self._last_sent = -1e18

    @property
    def sent(self) -> int:
        return self._seq

    def add(self, kind: UiKind, now: float, key: Optional[str] = None, value=None) -> UiAction:
        self._seq += 1
        a = UiAction(kind=UiKind(kind), key=key, value=value, t=float(now), seq=self._seq)
        self._recent.append(a)
        self._last_sent = -1e18                     # send now
        return a

    def message(self, now: float) -> Optional[bytes]:
        """The message to send now (a new action, or a heartbeat while actions are recent), else None."""
        while self._recent and now - self._recent[0].t > self.resend_s:
            self._recent.popleft()
        if not self._recent or now - self._last_sent < self.heartbeat_s:
            return None
        self._last_sent = now
        return msgpack.packb({"v": VERSION, "kind": "ui_actions", "server": self.server,
                              "actions": [action_to_dict(a) for a in self._recent]}, use_bin_type=True)


class ActionInbox:
    """Node side: each action once, in order, and only while fresh."""

    def __init__(self, max_age_s: float = 1.0):
        self.max_age_s = max_age_s
        self.server: Optional[str] = None
        self._seen: dict = {}                       # seq -> t of the current server's actions already taken
        self.last_seq = 0

    def accept(self, raw: bytes, now: float) -> list:
        m = _unpack(raw, "ui_actions")
        actions = [action_from_dict(d) for d in m.get("actions", ())]
        if m.get("server") != self.server:
            self.server, self._seen = m.get("server"), {}
        new = []
        for a in sorted(actions, key=lambda a: a.seq):
            if a.seq in self._seen:
                continue
            self._seen[a.seq] = a.t
            if now - a.t <= self.max_age_s:
                new.append(a)
                self.last_seq = a.seq
        horizon = now - 10.0 * max(self.max_age_s, 1.0)
        self._seen = {s: t for s, t in self._seen.items() if t >= horizon}
        return new


# ---------------------------------------------------------------------------------------------------- status

@dataclass(frozen=True)
class NodeStatus:
    """What the node tells the panel. Plain values only (msgpack and JSON both carry it)."""
    t: float                      # node wall clock when sent
    tick: int
    state: str                    # supervisor State value: off | standby | active | paused
    resumes_by_itself: bool       # PAUSED and the pause ends without the operator (stale head)
    phase: str                    # SONIC lifecycle Phase value
    sonic: str                    # absent | waiting | controlling
    live: bool                    # operator input alive (head fresh)
    ready: bool                   # source can drive (Quest: calibrated for this session and recenter epoch)
    operator: dict                # OperatorInput.status: mode, heading_lock, calibrating (source-defined)
    settings: dict                # OperatorSettings.to_dict(), the values in effect
    rate_hz: float                # measured tick rate over the last status period
    overruns: int                 # ticks that missed their deadline, since start
    ages_ms: dict                 # sample age at the node per xr_frame source and "telemetry"; None = none valid
    run: Optional[str]            # run directory name, None when not logging
    segment: bool                 # a trial segment (MARK start) is open
    marks: int                    # MARK actions taken
    last_ui_seq: int              # last panel action taken
    notes: list                   # most recent state transition notes, newest last

    def encode(self) -> bytes:
        return msgpack.packb({"v": VERSION, "kind": "node_status", **asdict(self)}, use_bin_type=True)

    @classmethod
    def decode(cls, raw: bytes) -> "NodeStatus":
        m = _unpack(raw, "node_status")
        try:
            return cls(**{f.name: m[f.name] for f in fields(cls)})
        except KeyError as e:
            raise LinkError(f"node_status without {e}") from e

    def to_dict(self) -> dict:
        return asdict(self)
