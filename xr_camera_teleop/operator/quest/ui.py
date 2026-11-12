"""Headset control panel (PLAN §6a, D13): layout, pointers, raycast, hover, hold-to-confirm, pages. Pure.

Everything the operator can do with the panel is decided here, from the xr_frame (pointer sources, gestures) and the
node's status; Vuer only draws. operator/quest/ui_draw.py turns a `PanelPicture` / `StripPicture` into pixels and
operator/quest/server.py upserts them, publishes the requests through fullbody/operator_link.py and serves `describe()`
at GET /state. No I/O and no clock reads here: the server passes `now` in.

Opening and closing
  Open:  press a thumbstick (either controller; Quest Browser does not give WebXR the menu button), or turn the left
         palm toward the eyes for palm_hold_s (hand tracking). The palm gesture re-arms once the palm turns away.
  Close: the X tab, a thumbstick press, "Devam" (resume) or "Kalibre et ve devral" (which engages the robot).
  The panel is world-locked where it opened: `distance` ahead of the head along its heading, `drop` below the eyes
  (chest height), tilted to face the eyes.

Freeze while open (PLAN §6a): opening pauses an ACTIVE robot (PAUSE), and the panel keeps the robot out of ACTIVE
while it is open (a PAUSE again if anything else engages it, at most every repause_s). A pause during the node's
automatic stale-head pause becomes the operator's (fullbody/supervisor.py), so a head coming back cannot resume the
robot under the panel. Closing resumes (RESUME) only a pause the panel made; "Duraklat" on the panel hands the pause
to the operator (closing then leaves the robot paused).

Pointers (one per side, `pointers`)
  Controller (physical, PLAN D8 WALK mode): its WebXR target ray (+X of its pose in F); click = trigger, with
         hysteresis (trigger_on / trigger_off).
  Hand (HANDS mode): the Quest system pointer model, a ray from an estimated shoulder through the index knuckle (the
         knuckle does not move when the fingers pinch, so the ray holds still while clicking); click = thumb-index
         pinch distance below pinch_on, released above pinch_off.
  Either works on every page; a side uses its controller when one is tracked, else its hand.

Selection
  Raycast from each pointer to the panel plane; the item under the hit point is hovered. A press (click edge) on an
  enabled item captures that pointer. Items with `hold` confirm after hold_s of continuous press on the same item
  (the ring fills); releasing or leaving earlier cancels. Steppers repeat every repeat_s while still held after
  confirming. Durdur (stop), the page tabs and X act on the press itself: Stop must never wait, and tabs and ✕ do not
  reach the robot. Everything that reaches the robot or the view is hold-to-confirm.

Pages (tabs): Robot (Başlat, Kalibre et ve devral, Duraklat/Devam, status), Hareket and Eller (one row per
fullbody/settings.py KNOBS entry of that page: no knob list here), Görüntü (ego / immersive / off), Sistem
(recalibrate, trial segment marks in the run log, rates). Durdur is on top of every page.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import C_F_XR, yaw_of
from xr_camera_teleop.fullbody.operator_link import STATUS_STALE_S, NodeStatus
from xr_camera_teleop.fullbody.settings import KNOBS, UiKind
from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame

PAGES = ("robot", "hareket", "eller", "goruntu", "sistem")
PAGE_TITLES = {"robot": "Robot", "hareket": "Hareket", "eller": "Eller", "goruntu": "Görüntü", "sistem": "Sistem"}
VIEWS = ("ego", "immersive", "none")
VIEW_LABELS = {"ego": "Ego", "immersive": "İmmersif", "none": "Kapalı"}
STATE_LABELS = {"off": "KAPALI", "standby": "HAZIR", "active": "AKTİF", "paused": "DURAKLADI"}
SONIC_LABELS = {"controlling": "kontrol ediyor", "waiting": "bekliyor", "absent": "yok"}
MODE_LABELS = {"walk": "YÜRÜME", "hands": "ELLER"}

# WebXR hand joints (fullbody/xr_frame.py, operator/mano_to_webxr.py order)
WRIST, INDEX_MC, INDEX_KNUCKLE, MIDDLE_MC, PINKY_MC = 0, 5, 6, 10, 20


@dataclass(frozen=True)
class UiParams:
    width: float = 0.44                  # panel size, m
    height: float = 0.28
    distance: float = 0.45               # panel centre ahead of the eyes along the head's heading, m (PLAN §6a)
    drop: float = 0.28                   # and below the eyes: chest height, m
    hold_s: float = 0.6                  # hold-to-confirm (PLAN §6a)
    repeat_s: float = 0.3                # stepper repeat while held after confirming
    flash_s: float = 0.3                 # click feedback
    repause_s: float = 0.5               # re-send PAUSE while open and the robot is ACTIVE at most this often
    controller_timeout_s: float = 0.15   # a controller sample older than this is no pointer (PLAN §6 staleness)
    hand_timeout_s: float = 0.25         # PLAN §5 hand loss: hold 0.25 s
    trigger_on: float = 0.6
    trigger_off: float = 0.3
    pinch_on: float = 0.015              # thumb-index tip distance, m
    pinch_off: float = 0.030
    shoulder: tuple = (-0.05, 0.17, -0.22)   # shoulder from the eyes in the head-yaw frame (fwd, side, up), m
    palm_cos: float = 0.75               # palm normal . direction to the eyes above this: palm faces the eyes
    palm_reach: float = 0.7              # and the palm within this of the eyes, m
    palm_hold_s: float = 0.4
    node_stale_s: float = STATUS_STALE_S


# ---------------------------------------------------------------------------------------------------- layout

@dataclass(frozen=True)
class Item:
    """A selectable area of the panel. rect = (x0, y0, x1, y1) in panel metres from the top-left corner, y down."""
    id: str
    rect: tuple
    label: str
    kind: str                            # button | tab | stepper | toggle
    hold: bool                           # hold-to-confirm, else acts on the press
    style: str = "normal"                # normal | danger | primary
    repeat: bool = False

    def contains(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.rect
        return x0 <= x <= x1 and y0 <= y <= y1


@dataclass(frozen=True)
class Label:
    """Static text area of a page (status lines are filled at draw time by key)."""
    id: str
    rect: tuple


def _row_items(page: str, W: float, top: float, bottom: float):
    """One row per knob of `page`, in KNOBS order: label, value, then [−] [+] or a toggle."""
    keys = [k for k, kb in KNOBS.items() if kb.page == page]
    if not keys:
        return [], []
    h = min(0.034, (bottom - top) / len(keys))
    items, labels = [], []
    for n, key in enumerate(keys):
        y0, y1 = top + n * h + 0.003, top + (n + 1) * h - 0.003
        kb = KNOBS[key]
        labels.append(Label(f"knob.{key}", (0.015, y0, 0.27, y1)))
        if kb.lo is None:
            items.append(Item(f"set.{key}", (0.28, y0, W - 0.015, y1), kb.label, "toggle", True))
        else:
            mid = 0.28 + (W - 0.015 - 0.28) / 2
            items.append(Item(f"set.{key}.dec", (0.28, y0, mid - 0.004, y1), "–", "stepper", True, repeat=True))
            items.append(Item(f"set.{key}.inc", (mid + 0.004, y0, W - 0.015, y1), "+", "stepper", True, repeat=True))
    return items, labels


def build_layout(p: UiParams = UiParams()) -> dict:
    """{page: (items, labels)}; items on every page (Durdur, X, tabs) are included in each."""
    W, H = p.width, p.height
    common = [Item("stop", (0.012, 0.010, 0.135, 0.058), "DURDUR", "button", False, "danger"),
              Item("close", (W - 0.050, 0.010, W - 0.012, 0.058), "X", "tab", False)]
    tab_w = (W - 0.024) / len(PAGES)
    for n, page in enumerate(PAGES):
        common.append(Item(f"tab.{page}", (0.012 + n * tab_w + 0.002, 0.066, 0.012 + (n + 1) * tab_w - 0.002, 0.104),
                           PAGE_TITLES[page], "tab", False))
    header = [Label("header", (0.145, 0.010, W - 0.060, 0.058))]
    top, bottom = 0.116, H - 0.010
    pages = {}
    col = 0.205
    robot = [Item("robot.start", (0.012, top, col, top + 0.044), "Başlat", "button", True, "primary"),
             Item("robot.calibrate", (0.012, top + 0.052, col, top + 0.096), "Kalibre et ve devral", "button", True,
                  "primary"),
             Item("robot.pause", (0.012, top + 0.104, col, top + 0.148), "Duraklat", "button", True)]
    pages["robot"] = (common + robot, header + [Label("robot.status", (col + 0.015, top, W - 0.012, bottom))])
    for page in ("hareket", "eller"):
        items, labels = _row_items(page, W, top, bottom - 0.03 if page == "eller" else bottom)
        extra = [Label("eller.mode", (0.015, bottom - 0.03, W - 0.015, bottom))] if page == "eller" else []
        pages[page] = (common + items, header + labels + extra)
    vw = (W - 0.024) / len(VIEWS)
    views = [Item(f"view.{v}", (0.012 + n * vw + 0.004, top + 0.01, 0.012 + (n + 1) * vw - 0.004, top + 0.07),
                  VIEW_LABELS[v], "button", True) for n, v in enumerate(VIEWS)]
    pages["goruntu"] = (common + views, header + [Label("goruntu.note", (0.015, top + 0.085, W - 0.015, bottom))])
    system = [Item("system.calibrate", (0.012, top, col, top + 0.044), "Yeniden kalibre et", "button", True),
              Item("system.mark", (0.012, top + 0.052, col, top + 0.096), "Bölüm başlat", "button", True)]
    pages["sistem"] = (common + system, header + [Label("sistem.rates", (col + 0.015, top, W - 0.012, bottom)),
                                                  Label("sistem.run", (0.012, top + 0.104, col, bottom))])
    return pages


# ---------------------------------------------------------------------------------------------------- geometry

def panel_pose(head_T, p: UiParams = UiParams()) -> np.ndarray:
    """World-locked panel pose in F for a head pose in F: columns (right, up, normal toward the eyes), origin at the
    panel centre. Its local axes are those of a three.js plane (x right, y up, +z out of the face)."""
    head_T = np.asarray(head_T, dtype=float)
    eye = head_T[:3, 3]
    psi = yaw_of(head_T)
    fwd = np.array([math.cos(psi), math.sin(psi), 0.0])
    c = eye + p.distance * fwd + np.array([0.0, 0.0, -p.drop])
    n = eye - c
    n /= np.linalg.norm(n)
    u = np.cross([0.0, 0.0, 1.0], n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = u, v, n, c
    return T


def raycast(T_panel, origin, direction, p: UiParams = UiParams(), margin: float = 0.0):
    """(x, y, point) where the ray meets the panel's front face, in panel metres from its top-left corner (y down)
    and in F, or None if it misses (behind the ray, from behind the panel, or outside the panel + margin)."""
    c, u, v, n = T_panel[:3, 3], T_panel[:3, 0], T_panel[:3, 1], T_panel[:3, 2]
    d = np.asarray(direction, dtype=float)
    o = np.asarray(origin, dtype=float)
    den = float(d @ n)
    if den >= -1e-6:                     # parallel, or coming from behind the panel
        return None
    t = float((c - o) @ n) / den
    if t <= 0.0:
        return None
    hit = o + t * d
    x = float((hit - c) @ u) + p.width / 2
    y = p.height / 2 - float((hit - c) @ v)
    if not (-margin <= x <= p.width + margin and -margin <= y <= p.height + margin):
        return None
    return x, y, hit


def panel_point(T_panel, x: float, y: float, p: UiParams = UiParams()) -> np.ndarray:
    """F point of panel coordinates (x right, y down from the top-left corner), on its face."""
    return T_panel[:3, 3] + (x - p.width / 2) * T_panel[:3, 0] + (p.height / 2 - y) * T_panel[:3, 1]


def to_xr(T_F) -> tuple[list, list]:
    """A panel-like pose in F -> (position, three.js XYZ Euler rotation) in the WebXR world Vuer draws in. Only the
    world axes change (the body axes are the plane's own)."""
    R = C_F_XR.T @ np.asarray(T_F, dtype=float)[:3, :3]
    p = C_F_XR.T @ np.asarray(T_F, dtype=float)[:3, 3]
    ry = math.asin(max(-1.0, min(1.0, R[0, 2])))
    if abs(R[0, 2]) < 0.9999999:
        rx, rz = math.atan2(-R[1, 2], R[2, 2]), math.atan2(-R[0, 1], R[0, 0])
    else:
        rx, rz = math.atan2(R[2, 1], R[1, 1]), 0.0
    return [float(x) for x in p], [rx, ry, rz]


def point_to_xr(p_F) -> list:
    return [float(x) for x in C_F_XR.T @ np.asarray(p_F, dtype=float)]


# ---------------------------------------------------------------------------------------------------- pointers

@dataclass(frozen=True)
class Pointer:
    side: str
    source: str                          # controller | hand
    origin: np.ndarray                   # F
    direction: np.ndarray                # F, unit
    pressed: bool


def shoulder(head_T, side: str, p: UiParams = UiParams()) -> np.ndarray:
    psi = yaw_of(head_T)
    fwd, sidew, up = p.shoulder
    c, s = math.cos(psi), math.sin(psi)
    lat = sidew if side == "left" else -sidew
    return np.asarray(head_T, dtype=float)[:3, 3] + np.array([c * fwd - s * lat, s * fwd + c * lat, up])


def palm_normal(joints, side: str) -> np.ndarray:
    """Unit normal out of the palm from the wrist and the index and pinky metacarpals (no joint-axis conventions)."""
    j = np.asarray(joints, dtype=float)
    n = np.cross(j[INDEX_MC] - j[WRIST], j[PINKY_MC] - j[WRIST])
    n = n if side == "right" else -n
    return n / max(np.linalg.norm(n), 1e-9)


def palm_faces_eyes(joints, side: str, head_T, p: UiParams = UiParams()) -> bool:
    j = np.asarray(joints, dtype=float)
    centre = (j[WRIST] + j[INDEX_MC] + j[MIDDLE_MC] + j[PINKY_MC]) / 4
    to_eye = np.asarray(head_T, dtype=float)[:3, 3] - centre
    dist = float(np.linalg.norm(to_eye))
    return 0.05 < dist <= p.palm_reach and float(palm_normal(j, side) @ to_eye) / dist >= p.palm_cos


def pointers(frame: Optional[XRFrame], now: float, was_pressed: dict, p: UiParams = UiParams()) -> dict:
    """{side: Pointer} for the sides with a usable pointer source. `was_pressed`: {side: bool} of the last update
    (click hysteresis)."""
    out = {}
    if frame is None or not frame.connected:
        return out
    head = frame.head
    for side in SIDES:
        c, h = frame.controllers[side], frame.hands[side]
        before = was_pressed.get(side, False)
        if c.stamp.fresh(now, p.controller_timeout_s) and c.T is not None:
            pressed = c.trigger >= p.trigger_off if before else c.trigger >= p.trigger_on
            out[side] = Pointer(side, "controller", c.T[:3, 3].copy(), c.T[:3, 0] / np.linalg.norm(c.T[:3, 0]),
                                pressed)
        elif h.stamp.fresh(now, p.hand_timeout_s) and h.joints is not None and head.T is not None:
            knuckle = np.asarray(h.joints[INDEX_KNUCKLE], dtype=float)
            d = knuckle - shoulder(head.T, side, p)
            if np.linalg.norm(d) < 1e-6:
                continue
            pinch = h.pinch
            pressed = (not math.isnan(pinch)) and (pinch <= p.pinch_off if before else pinch <= p.pinch_on)
            out[side] = Pointer(side, "hand", knuckle, d / np.linalg.norm(d), pressed)
    return out


# ---------------------------------------------------------------------------------------------------- panel

@dataclass(frozen=True)
class Request:
    """Something for the node: becomes a UiAction (fullbody/operator_link.py ActionOutbox adds time and seq)."""
    kind: UiKind
    key: Optional[str] = None
    value: object = None


@dataclass(frozen=True)
class UiStep:
    requests: tuple = ()                 # for the node, in order
    view: Optional[str] = None           # robot view mode chosen on the panel (the server's to apply)


@dataclass(frozen=True)
class DrawnItem:
    id: str
    rect: tuple
    label: str
    kind: str
    style: str
    enabled: bool
    on: bool                             # toggle on / current view / current tab
    hovered: bool
    progress: float                      # hold-to-confirm ring 0..1 (0 = not held)
    flash: bool                          # just fired


@dataclass(frozen=True)
class PanelPicture:
    """Everything ui_draw needs for one panel image (equality = nothing to redraw)."""
    page: str
    items: tuple
    texts: tuple                         # (rect, lines) per label area
    cursors: tuple                       # (x, y, pressed) in panel metres


@dataclass(frozen=True)
class StripPicture:
    segments: tuple                      # (text, tone) with tone in ok | warn | bad | dim


@dataclass
class _Capture:
    side: str
    item: str
    t0: float
    fired: bool = False
    next_repeat: float = math.inf


def _fresh(status: Optional[NodeStatus], status_rx: Optional[float], now: float, p: UiParams) -> bool:
    return status is not None and status_rx is not None and now - status_rx <= p.node_stale_s


def _fmt_value(key: str, v) -> str:
    kb = KNOBS[key]
    if kb.lo is None:
        return "AÇIK" if v else "KAPALI"
    return f"{float(v):.2f}"


class Panel:
    """The panel's state. `step` once per server update (any rate; ~60 Hz); read `picture()`, `strip()`, `rays`,
    `describe()` after it."""

    def __init__(self, params: UiParams = UiParams()):
        self.p = params
        self.layout = build_layout(params)
        self.open = False
        self.T: Optional[np.ndarray] = None  # panel pose in F while open
        self.page = "robot"
        self.holds_pause = False             # the panel paused the robot: closing resumes it
        self.hover: dict = {}                # side -> item id
        self.hits: dict = {}                 # side -> (x, y, point F)
        self.rays: dict = {}                 # side -> (origin F, end F, pressed)
        self.capture: Optional[_Capture] = None
        self.flash: dict = {}                # item id -> until
        self._pressed: dict = {}
        self._sticks: dict = {}
        self._palm_since: Optional[float] = None
        self._palm_armed = True
        self._last_pause = -math.inf
        self.status: Optional[NodeStatus] = None
        self.status_rx: Optional[float] = None
        self.view = "ego"
        self.rates: dict = {}
        self.frame: Optional[XRFrame] = None
        self.now = 0.0
        self.opened = 0

    # ------------------------------------------------------------------ queries
    def items(self) -> list:
        return self.layout[self.page][0]

    def item(self, item_id: str) -> Optional[Item]:
        return next((it for it in self.items() if it.id == item_id), None)

    def node_ok(self) -> bool:
        return _fresh(self.status, self.status_rx, self.now, self.p)

    def enabled(self, it: Item) -> bool:
        if it.id == "stop" or it.kind == "tab" or it.id.startswith("view."):
            return True
        if not self.node_ok():
            return False
        st = self.status
        if it.id == "robot.start":
            return st.state == "off" and st.phase == "off"
        if it.id in ("robot.calibrate", "system.calibrate"):
            return st.state in ("standby", "active", "paused") and st.live
        if it.id == "robot.pause":
            return st.state in ("active", "paused")
        if it.id.startswith("set."):
            return it.id.split(".")[1] in st.settings
        return True

    def _pause_label(self) -> str:
        st = self.status
        if st is not None and st.state == "paused" and not self.holds_pause:
            return "Devam"
        return "Duraklat"

    # ------------------------------------------------------------------ step
    def step(self, now: float, frame: Optional[XRFrame], status: Optional[NodeStatus], status_rx: Optional[float],
             view: str, rates: Optional[dict] = None) -> UiStep:
        self.now, self.frame, self.status, self.status_rx, self.view = now, frame, status, status_rx, view
        self.rates = rates or {}
        req: list = []
        new_view = None
        ptrs = pointers(frame, now, self._pressed, self.p)
        edges = {s: ptr.pressed and not self._pressed.get(s, False) for s, ptr in ptrs.items()}
        self._pressed = {s: ptr.pressed for s, ptr in ptrs.items()}
        self.flash = {k: v for k, v in self.flash.items() if v > now}

        toggle = self._stick_edge(frame, now)
        if not self.open:
            if toggle or self._palm_gesture(frame, now):
                self._open(frame, now, req)
        elif toggle:
            self._close(req)

        self.hover, self.hits, self.rays = {}, {}, {}
        if self.open:
            for s, ptr in ptrs.items():
                hit = raycast(self.T, ptr.origin, ptr.direction, self.p)
                if hit is None:
                    self.rays[s] = (ptr.origin, ptr.origin + 0.6 * ptr.direction, ptr.pressed)
                    continue
                x, y, point = hit
                self.hits[s] = (x, y, point)
                self.rays[s] = (ptr.origin, point, ptr.pressed)
                it = next((i for i in self.items() if i.contains(x, y)), None)
                if it is not None:
                    self.hover[s] = it.id
            new_view = self._interact(ptrs, edges, now, req)
            if self.open:
                self._keep_frozen(now, req)
        else:
            self.capture = None
        return UiStep(tuple(req), new_view)

    def _stick_edge(self, frame, now) -> bool:
        edge = False
        sticks = {}
        if frame is not None and frame.connected:
            for s in SIDES:
                c = frame.controllers[s]
                down = c.stamp.fresh(now, self.p.controller_timeout_s) and c.stick_press
                sticks[s] = down
                edge |= down and not self._sticks.get(s, False)
        self._sticks = sticks
        return edge

    def _palm_gesture(self, frame, now) -> bool:
        h = frame.hands["left"] if frame is not None and frame.connected else None
        up = (h is not None and h.stamp.fresh(now, self.p.hand_timeout_s) and h.joints is not None
              and frame.head.T is not None and palm_faces_eyes(h.joints, "left", frame.head.T, self.p))
        if not up:
            self._palm_since, self._palm_armed = None, True
            return False
        if not self._palm_armed:
            return False
        if self._palm_since is None:
            self._palm_since = now
        if now - self._palm_since >= self.p.palm_hold_s:
            self._palm_armed = False
            return True
        return False

    def _open(self, frame, now, req):
        if frame is None or frame.head.T is None:
            return
        self.open, self.T, self.capture = True, panel_pose(frame.head.T, self.p), None
        self.opened += 1
        self._palm_armed = False                    # a palm still up must turn away before it opens it again
        st = self.status
        if self.node_ok() and (st.state == "active" or (st.state == "paused" and st.resumes_by_itself)):
            req.append(Request(UiKind.PAUSE))
            self.holds_pause, self._last_pause = True, now

    def _close(self, req, resume: bool = True):
        st = self.status
        if resume and self.holds_pause and self.node_ok() and st.state == "paused":
            req.append(Request(UiKind.RESUME))
        self.open, self.holds_pause, self.capture = False, False, None
        self.hover, self.hits, self.rays = {}, {}, {}

    def _keep_frozen(self, now, req):
        st = self.status
        if self.node_ok() and st.state == "active" and now - self._last_pause >= self.p.repause_s:
            req.append(Request(UiKind.PAUSE))
            self.holds_pause, self._last_pause = True, now

    def _interact(self, ptrs, edges, now, req):
        new_view = None
        cap = self.capture
        if cap is None:
            for s, ptr in ptrs.items():
                it = self.item(self.hover.get(s, ""))
                if edges.get(s) and it is not None and self.enabled(it):
                    cap = self.capture = _Capture(s, it.id, now)
                    if not it.hold:
                        cap.fired = True
                        new_view = self._fire(it, req) or new_view
                    break
            return new_view
        it = self.item(cap.item)
        ptr = ptrs.get(cap.side)
        if it is None or ptr is None or not ptr.pressed or self.hover.get(cap.side) != cap.item or not self.enabled(it):
            self.capture = None
            return None
        if it.hold and not cap.fired and now - cap.t0 >= self.p.hold_s:
            cap.fired = True
            cap.next_repeat = now + self.p.repeat_s if it.repeat else math.inf
            new_view = self._fire(it, req)
        elif cap.fired and it.repeat and now >= cap.next_repeat:
            cap.next_repeat += self.p.repeat_s
            new_view = self._fire(it, req)
        return new_view

    def _fire(self, it: Item, req) -> Optional[str]:
        """Act on a confirmed item. Returns a new view mode, if any."""
        self.flash[it.id] = self.now + self.p.flash_s
        st = self.status
        if it.id == "stop":
            req.append(Request(UiKind.STOP))
        elif it.id == "close":
            self._close(req)
        elif it.id.startswith("tab."):
            self.page = it.id[4:]
        elif it.id == "robot.start":
            req.append(Request(UiKind.START))
        elif it.id in ("robot.calibrate", "system.calibrate"):
            req.append(Request(UiKind.CALIBRATE))
            self._close(req, resume=False)              # calibrating engages: the operator takes over
        elif it.id == "robot.pause":
            if self._pause_label() == "Devam":
                req.append(Request(UiKind.RESUME))
                self._close(req, resume=False)
            else:
                req.append(Request(UiKind.PAUSE))
                self.holds_pause, self._last_pause = False, self.now
        elif it.id == "system.mark":
            req.append(Request(UiKind.MARK, value="end" if st.segment else "start"))
        elif it.id.startswith("view."):
            return it.id[5:]
        elif it.id.startswith("set."):
            parts = it.id.split(".")
            key, kb, cur = parts[1], KNOBS[parts[1]], st.settings[parts[1]]
            if kb.lo is None:
                req.append(Request(UiKind.SET, key, not bool(cur)))
            else:
                step = kb.step if parts[2] == "inc" else -kb.step
                v = min(kb.hi, max(kb.lo, round(round((float(cur) + step) / kb.step) * kb.step, 6)))
                req.append(Request(UiKind.SET, key, v))
        return None

    # ------------------------------------------------------------------ pictures
    def _item_on(self, it: Item) -> bool:
        st = self.status
        if it.id.startswith("tab."):
            return it.id[4:] == self.page
        if it.id.startswith("view."):
            return it.id[5:] == self.view
        if it.id == "system.mark" and st is not None:
            return st.segment
        return False

    def _label(self, it: Item) -> str:
        if it.id == "robot.pause":
            return self._pause_label()
        if it.id == "system.mark" and self.status is not None and self.status.segment:
            return "Bölüm bitir"
        if it.kind == "toggle":                      # the row shows the state; the button says what it does
            v = self.status.settings.get(it.id.split(".")[1]) if self.status is not None else None
            return "—" if v is None else ("Kapat" if v else "Aç")
        return it.label

    def _texts(self) -> tuple:
        st, ok = self.status, self.node_ok()
        out = []
        for lb in self.layout[self.page][1]:
            out.append((lb.rect, tuple(self._text(lb.id, st, ok))))
        return tuple(out)

    def _devices(self) -> str:
        f = self.frame
        if f is None or not f.connected:
            return "yok"
        kinds = []
        for s, tr in (("left", "sol"), ("right", "sağ")):
            if f.controllers[s].stamp.valid:
                kinds.append(f"{tr} kontrolcü")
            elif f.hands[s].stamp.valid:
                kinds.append(f"{tr} el")
        return ", ".join(kinds) or "takip yok"

    def _text(self, label_id: str, st: Optional[NodeStatus], ok: bool) -> list:
        if label_id == "header":
            if not ok:
                return ["Node yok: robot komutları kapalı"]
            mode = MODE_LABELS.get(st.operator.get("mode"), "—")
            return [f"{STATE_LABELS.get(st.state, st.state)}  ·  {mode}  ·  SONIC {SONIC_LABELS.get(st.sonic, st.sonic)}"]
        if label_id.startswith("knob."):
            key = label_id[5:]
            v = st.settings.get(key) if ok else None
            return [KNOBS[key].label, "—" if v is None else _fmt_value(key, v)]
        if label_id == "robot.status":
            if not ok:
                return ["Node yanıt vermiyor.", "Durdur her zaman çalışır."]
            lock = st.operator.get("heading_lock", st.settings.get("heading_lock"))
            lines = [f"Robot: {STATE_LABELS.get(st.state, st.state)}",
                     f"SONIC: {SONIC_LABELS.get(st.sonic, st.sonic)}",
                     f"Kalibrasyon: {'var' if st.ready else 'yok'}",
                     f"Mod: {MODE_LABELS.get(st.operator.get('mode'), '—')}",
                     f"Yön kilidi: {'açık' if lock else 'kapalı'}",
                     f"Giriş: {self._devices()}"]
            if self.holds_pause:
                lines.append("Panel kapanınca devam eder")
            elif st.state == "paused":
                lines.append("Duraklatıldı: Devam ile sürdür")
            return lines
        if label_id == "eller.mode":
            mode = MODE_LABELS.get(st.operator.get("mode"), "—") if ok else "—"
            return [f"Mod: {mode}  ·  Giriş: {self._devices()}"]
        if label_id == "goruntu.note":
            return ["Robot kamerası (ZED stereo).", "Ego: önde pencere, İmmersif: tüm görüş."]
        if label_id == "sistem.rates":
            r = self.rates
            lines = []
            if "head_hz" in r:
                lines.append(f"Kafa {r['head_hz']:.0f} Hz")
                lines.append(f"El L/R {r['hand_hz'][0]:.0f}/{r['hand_hz'][1]:.0f} · "
                             f"Kontrolcü {r['ctrl_hz'][0]:.0f}/{r['ctrl_hz'][1]:.0f} Hz")
            if r.get("camera_fps") is not None:
                lines.append(f"Kamera {r['camera_fps']:.0f} fps · yaş {r['camera_age_ms']:.0f} ms")
            if ok:
                lines.append(f"Node {st.rate_hz:.0f} Hz · gecikmiş tik {st.overruns}")
                head = st.ages_ms.get("head")
                lines.append(f"Kafa yaşı {'—' if head is None else f'{head:.0f} ms'}")
            return lines
        if label_id == "sistem.run":
            if not ok:
                return []
            return [f"Kayıt: {st.run or 'kapalı'}", f"Bölüm: {'açık' if st.segment else 'yok'} ({st.marks})"]
        return []

    def picture(self) -> PanelPicture:
        drawn = []
        for it in self.items():
            cap = self.capture
            progress = 0.0
            if cap is not None and cap.item == it.id and it.hold and not cap.fired:
                progress = min(1.0, (self.now - cap.t0) / self.p.hold_s)
            drawn.append(DrawnItem(it.id, it.rect, self._label(it), it.kind, it.style, self.enabled(it),
                                   self._item_on(it), it.id in self.hover.values(), round(progress, 2),
                                   it.id in self.flash))
        cursors = tuple((round(x, 3), round(y, 3), bool(self.rays.get(s, (0, 0, False))[2]))
                        for s, (x, y, _) in sorted(self.hits.items()))
        return PanelPicture(self.page, tuple(drawn), self._texts(), cursors)

    def strip(self) -> StripPicture:
        st, ok = self.status, self.node_ok()
        if not ok:
            return StripPicture((("NODE YOK", "bad"), ("Menü: stick bas / sol avuç", "dim")))
        tone = {"active": "ok", "paused": "warn", "standby": "ok", "off": "bad"}.get(st.state, "warn")
        segs = [(STATE_LABELS.get(st.state, st.state), tone)]
        mode = st.operator.get("mode")
        if mode in MODE_LABELS:
            segs.append((MODE_LABELS[mode], "ok"))
        if st.operator.get("heading_lock", st.settings.get("heading_lock")):
            segs.append(("KİLİT", "warn"))
        if st.sonic != "controlling":
            segs.append((f"SONIC {SONIC_LABELS.get(st.sonic, st.sonic)}", "bad"))
        if st.state in ("standby", "paused") and not st.ready:
            segs.append(("kalibre et", "warn"))
        if not st.live:
            segs.append(("kafa verisi yok", "bad"))
        if st.segment:
            segs.append(("● bölüm", "warn"))
        segs.append(("Menü: stick / sol avuç", "dim"))
        return StripPicture(tuple(segs))

    def describe(self) -> dict:
        """JSON-able state for GET /state: what is open, where (WebXR world), what is hovered and held."""
        d = {"open": self.open, "page": self.page, "holds_pause": self.holds_pause, "opened": self.opened,
             "hover": dict(self.hover), "capture": None, "pointers": {}, "items": []}
        if self.capture is not None:
            c = self.capture
            d["capture"] = {"side": c.side, "item": c.item, "held_s": round(self.now - c.t0, 3), "fired": c.fired}
        for s, (o, e, pressed) in self.rays.items():
            hit = self.hits.get(s)
            d["pointers"][s] = {"origin_xr": point_to_xr(o), "end_xr": point_to_xr(e), "pressed": bool(pressed),
                                "hit": None if hit is None else {"x": hit[0], "y": hit[1],
                                                                 "point_xr": point_to_xr(hit[2])}}
        if self.open:
            pos, rot = to_xr(self.T)
            d["pose_xr"] = {"position": pos, "rotation": rot}
            d["normal_xr"] = point_to_xr(self.T[:3, 2])
            for it in self.items():
                x0, y0, x1, y1 = it.rect
                d["items"].append({"id": it.id, "label": self._label(it), "enabled": self.enabled(it),
                                   "center_xr": point_to_xr(panel_point(self.T, (x0 + x1) / 2, (y0 + y1) / 2, self.p)),
                                   "rect": list(it.rect)})
        return d

