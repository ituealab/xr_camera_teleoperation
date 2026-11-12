"""operator/quest/ui.py (and ui_draw.py): raycast, hover, hold-to-confirm, immediate Stop, pages built from the knob
table, SET values, opening and closing (stick, palm), the freeze-while-open rule, hand and controller pointers."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import C_F_XR, pose, rot_x, rot_y, rot_z
from xr_camera_teleop.fullbody.operator_link import NodeStatus
from xr_camera_teleop.fullbody.settings import KNOBS, OperatorSettings, UiKind
from xr_camera_teleop.fullbody.xr_frame import Controller, Hand, Head, Stamp, XRFrame
from xr_camera_teleop.operator.quest import ui

P = ui.UiParams()
EYE = np.array([0.0, 0.0, 1.6])


def status(state="standby", **kw):
    d = dict(t=0.0, tick=1, state=state, resumes_by_itself=False, phase="running" if state != "off" else "off",
             sonic="controlling" if state != "off" else "absent", live=True, ready=True, operator={"mode": "walk"},
             settings=OperatorSettings().to_dict(), rate_hz=50.0, overruns=0, ages_ms={"head": 10.0}, run="r",
             segment=False, marks=0, last_ui_seq=0, notes=[])
    d.update(kw)
    return NodeStatus(**d)


def aim(origin, target):
    """Pose at `origin` whose +X (a WebXR target ray in F) points at `target`."""
    x = np.asarray(target, float) - np.asarray(origin, float)
    x /= np.linalg.norm(x)
    y = np.cross([0, 0, 1.0], x)
    y /= np.linalg.norm(y)
    return pose(np.column_stack([x, y, np.cross(x, y)]), origin)


class Rig:
    """A headset operator: head at EYE looking along +X, a right controller (or right hand) and a Panel."""

    def __init__(self, st=None):
        self.panel, self.t, self.st = ui.Panel(P), 100.0, st or status()
        self.ctrl = dict(T=aim([0.25, -0.2, 1.2], [1, -0.2, 1.2]), trigger=0.0, stick_press=False)
        self.hand = None                                # (joints, pinch) for the right hand instead of a controller
        self.left_hand = None
        self.view = "ego"
        self.seq = 0

    def frame(self):
        self.seq += 1
        s = Stamp(self.t, self.t, self.seq, True)
        ctrls = {"left": Controller(), "right": Controller()}
        hands = {"left": Hand(), "right": Hand()}
        if self.hand is None:
            ctrls["right"] = Controller(s, self.ctrl["T"], np.zeros(2), trigger=self.ctrl["trigger"],
                                        stick_press=self.ctrl["stick_press"])
        else:
            joints, pinch = self.hand
            hands["right"] = Hand(s, pose(np.eye(3), joints[0]), joints, pinch)
        if self.left_hand is not None:
            hands["left"] = Hand(s, pose(np.eye(3), self.left_hand[0]), self.left_hand, 0.08)
        return XRFrame(seq=self.seq, t_pub=self.t, session=1, connected=True,
                       head=Head(s, pose(np.eye(3), EYE)), controllers=ctrls, hands=hands)

    def step(self, dt=0.02, status_age=0.05):
        self.t += dt
        out = self.panel.step(self.t, self.frame(), self.st, self.t - status_age, self.view)
        if out.view:
            self.view = out.view
        return out

    def run(self, seconds, **kw):
        reqs = []
        for _ in range(round(seconds / 0.02)):
            reqs += list(self.step(**kw).requests)
        return reqs

    def open(self):
        self.ctrl["stick_press"] = True
        out = self.step()
        self.ctrl["stick_press"] = False
        self.step()
        assert self.panel.open
        return out

    def point_at(self, item_id):
        it = self.panel.item(item_id)
        x0, y0, x1, y1 = it.rect
        target = ui.panel_point(self.panel.T, (x0 + x1) / 2, (y0 + y1) / 2, P)
        self.ctrl["T"] = aim(self.ctrl["T"][:3, 3], target)

    def press(self, item_id, seconds):
        self.point_at(item_id)
        self.step()
        self.ctrl["trigger"] = 1.0
        reqs = self.run(seconds)
        self.ctrl["trigger"] = 0.0
        reqs += self.run(0.04)
        return reqs


# ---------------------------------------------------------------------------------------------------- geometry

def test_panel_is_world_locked_ahead_at_chest_upright_facing_the_operator():
    head = pose(rot_z(math.radians(90)) @ rot_y(math.radians(30)), [1.0, 2.0, 1.7])   # turned left, looking down
    T = ui.panel_pose(head, P)
    assert np.allclose(T[:3, 3], [1.0, 2.0 + P.distance, 1.7 - P.drop])
    assert np.allclose(T[:3, 2], [0.0, -1.0, 0.0])                                   # faces the operator
    assert np.allclose(T[:3, 1], [0.0, 0.0, 1.0])                                    # upright
    assert np.allclose(T[:3, 0], [1.0, 0.0, 0.0])                                    # its right = operator's right
    assert np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3)) and np.linalg.det(T[:3, :3]) > 0


def test_a_hand_at_the_waist_can_point_at_the_whole_panel():
    T = ui.panel_pose(pose(np.eye(3), EYE), P)
    for o in (np.array([0.38, -0.2, 1.0]), np.array([0.3, 0.25, 1.15])):
        for x, y in ((0.0, 0.0), (P.width, P.height), (P.width / 2, P.height / 2)):
            target = ui.panel_point(T, x, y, P)
            assert ui.raycast(T, o, (target - o) / np.linalg.norm(target - o), P, margin=1e-6) is not None


def test_raycast_maps_hits_to_panel_coordinates_and_misses():
    T = ui.panel_pose(pose(np.eye(3), EYE), P)
    o = EYE + [0.0, 0.0, -0.1]
    for x, y in ((P.width / 2, P.height / 2), (0.02, 0.02), (P.width - 0.01, P.height - 0.03)):
        target = ui.panel_point(T, x, y, P)
        hx, hy, point = ui.raycast(T, o, (target - o) / np.linalg.norm(target - o), P)
        assert (hx, hy) == pytest.approx((x, y), abs=1e-9) and np.allclose(point, target)
    left = ui.panel_point(T, 0.0, P.height / 2, P)
    assert left[1] > T[1, 3]                                  # x = 0 is on the operator's left (+Y in F)
    assert ui.raycast(T, o, [-1.0, 0.0, 0.0], P) is None      # pointing away
    assert ui.raycast(T, o, [1.0, 0.0, 1.0] / np.sqrt(2), P) is None      # above the panel
    behind = T[:3, 3] - 0.3 * T[:3, 2]
    assert ui.raycast(T, behind, T[:3, 2], P) is None          # from behind the panel


def test_to_xr_matches_three_js_euler_and_world_axes():
    T = ui.panel_pose(pose(rot_z(0.7) @ rot_y(0.2), [0.3, -0.4, 1.65]), P)
    pos, (rx, ry, rz) = ui.to_xr(T)
    R3 = rot_x(rx) @ rot_y(ry) @ rot_z(rz)                     # three.js Euler 'XYZ'
    assert np.allclose(C_F_XR @ R3, T[:3, :3], atol=1e-9)
    assert np.allclose(C_F_XR @ pos, T[:3, 3])


# ---------------------------------------------------------------------------------------------------- pages

def test_knob_pages_are_built_from_the_table():
    layout = ui.build_layout(P)
    for key, kb in KNOBS.items():
        page_items = [it.id for it in layout[kb.page][0]]
        if kb.lo is None:
            assert f"set.{key}" in page_items
        else:
            assert f"set.{key}.dec" in page_items and f"set.{key}.inc" in page_items
        others = [it.id for page, (items, _) in layout.items() if page != kb.page for it in items]
        assert not any(i.startswith(f"set.{key}") for i in others)
        assert any(lb.id == f"knob.{key}" for lb in layout[kb.page][1])
    for page, (items, _) in layout.items():
        assert {"stop", "close"} <= {it.id for it in items} and len({it.id for it in items}) == len(items)
        for it in items:                                      # inside the panel, not overlapping another item
            x0, y0, x1, y1 = it.rect
            assert 0 <= x0 < x1 <= P.width and 0 <= y0 < y1 <= P.height
            for other in items:
                if other is not it:
                    a0, b0, a1, b1 = other.rect
                    assert x1 <= a0 or a1 <= x0 or y1 <= b0 or b1 <= y0, (it.id, other.id)


def test_knob_rows_show_the_nodes_values_with_the_knob_labels():
    r = Rig(status(settings={**OperatorSettings().to_dict(), "speed_scale": 0.7, "heading_lock": True}))
    r.open()
    r.panel.page = "hareket"
    texts = {tuple(lines)[0]: tuple(lines) for _, lines in r.panel.picture().texts if lines}
    assert texts["Hız"] == ("Hız", "0.70") and texts["Yön kilidi"] == ("Yön kilidi", "AÇIK")


def test_tabs_switch_pages_on_the_press():
    r = Rig()
    r.open()
    r.point_at("tab.hareket")
    r.step()
    r.ctrl["trigger"] = 1.0
    r.step()
    assert r.panel.page == "hareket"


# ---------------------------------------------------------------------------------------------------- selection

def test_hover_follows_the_controller_ray():
    r = Rig()
    r.open()
    r.point_at("robot.calibrate")
    r.step()
    assert r.panel.hover == {"right": "robot.calibrate"}
    assert r.panel.picture().cursors and r.panel.describe()["pointers"]["right"]["hit"] is not None
    r.ctrl["T"] = aim(r.ctrl["T"][:3, 3], [0.0, -3.0, 1.2])          # away from the panel
    r.step()
    assert r.panel.hover == {}


def test_hold_to_confirm_fires_once_after_the_hold():
    r = Rig(status("off"))
    r.open()
    reqs = r.press("robot.start", 0.5)
    assert reqs == []                                                 # released early: cancelled
    r.point_at("robot.start")
    r.step()
    r.ctrl["trigger"] = 1.0
    assert r.run(0.56) == []
    pic = next(i for i in r.panel.picture().items if i.id == "robot.start")
    assert 0.8 < pic.progress < 1.0                                   # the ring is almost full
    reqs = r.run(0.1) + r.run(1.0)                                    # still held: no repeat for a button
    assert reqs == [ui.Request(UiKind.START)]


def test_leaving_the_item_cancels_the_hold():
    r = Rig(status("off"))
    r.open()
    r.point_at("robot.start")
    r.step()
    r.ctrl["trigger"] = 1.0
    r.run(0.3)
    r.point_at("robot.calibrate")
    assert r.run(0.5) == []
    r.point_at("robot.start")                                         # coming back while pressed does not resume
    assert r.run(0.8) == []


def test_stop_is_immediate_and_works_without_the_node():
    r = Rig()
    r.open()
    r.point_at("stop")
    r.step()
    r.ctrl["trigger"] = 1.0
    assert list(r.step().requests) == [ui.Request(UiKind.STOP)]
    r.ctrl["trigger"] = 0.0
    r.run(0.1, status_age=5.0)                                        # node silent: everything else is disabled
    assert not r.panel.enabled(r.panel.item("robot.calibrate"))
    r.point_at("stop")
    r.step(status_age=5.0)
    r.ctrl["trigger"] = 1.0
    assert list(r.step(status_age=5.0).requests) == [ui.Request(UiKind.STOP)]


def test_disabled_items_do_not_fire():
    r = Rig(status("standby"))
    r.open()
    assert r.press("robot.start", 1.0) == []                          # SONIC already running


def test_steppers_set_from_the_nodes_value_and_repeat_while_held():
    r = Rig(status(settings={**OperatorSettings().to_dict(), "speed_scale": 0.5}))
    r.open()
    r.panel.page = "hareket"
    reqs = r.press("set.speed_scale.inc", 0.6 + 0.3 * 2 + 0.1)        # confirm + two repeats
    assert reqs == [ui.Request(UiKind.SET, "speed_scale", 0.6)] * 3   # the node's value did not change meanwhile
    r.st = status(settings={**OperatorSettings().to_dict(), "speed_scale": 1.0})
    assert r.press("set.speed_scale.inc", 0.7) == [ui.Request(UiKind.SET, "speed_scale", 1.0)]   # clamped


def test_toggles_flip_the_nodes_value():
    r = Rig(status(settings={**OperatorSettings().to_dict(), "fingers": True}))
    r.open()
    r.panel.page = "eller"
    assert r.press("set.fingers", 0.7) == [ui.Request(UiKind.SET, "fingers", False)]


def test_view_modes_are_the_servers():
    r = Rig()
    r.open()
    r.panel.page = "goruntu"
    assert r.press("view.immersive", 0.7) == [] and r.view == "immersive"
    r.press("view.none", 0.7)
    assert r.view == "none"


def test_mark_toggles_the_trial_segment():
    r = Rig()
    r.open()
    r.panel.page = "sistem"
    assert r.press("system.mark", 0.7) == [ui.Request(UiKind.MARK, value="start")]
    r.st = status(segment=True)
    assert r.press("system.mark", 0.7) == [ui.Request(UiKind.MARK, value="end")]


# ---------------------------------------------------------------------------------------------------- open / freeze

def test_opening_pauses_an_active_robot_and_closing_resumes_it():
    r = Rig(status("active"))
    out = r.open()
    assert list(out.requests) == [ui.Request(UiKind.PAUSE)] and r.panel.holds_pause
    r.st = status("paused")
    assert r.press("close", 0.05) == [ui.Request(UiKind.RESUME)] and not r.panel.open


def test_stick_press_toggles_the_panel():
    r = Rig()
    r.open()
    r.ctrl["stick_press"] = True
    r.step()
    assert not r.panel.open


def test_an_automatic_pause_is_taken_over_and_resumed_on_close():
    r = Rig(status("paused", resumes_by_itself=True))
    assert list(r.open().requests) == [ui.Request(UiKind.PAUSE)]
    r.st = status("paused")
    assert r.press("close", 0.05) == [ui.Request(UiKind.RESUME)]


def test_an_operator_pause_is_left_alone():
    r = Rig(status("paused"))
    assert list(r.open().requests) == []
    assert r.press("close", 0.05) == []


def test_the_robot_stays_frozen_while_open():
    r = Rig(status("standby"))
    r.open()
    r.st = status("active")                                           # engaged by a button meanwhile
    reqs = r.run(1.1)
    assert reqs.count(ui.Request(UiKind.PAUSE)) == 3                  # at once, then every repause_s
    r.st = status("paused")
    assert r.press("close", 0.05) == [ui.Request(UiKind.RESUME)]


def test_duraklat_keeps_the_robot_paused_after_closing():
    r = Rig(status("active"))
    r.open()
    r.st = status("paused")
    assert r.panel._label(r.panel.item("robot.pause")) == "Duraklat"
    assert r.press("robot.pause", 0.7) == [ui.Request(UiKind.PAUSE)]
    assert not r.panel.holds_pause
    assert r.panel._label(r.panel.item("robot.pause")) == "Devam"
    assert r.press("close", 0.05) == []


def test_devam_resumes_and_closes():
    r = Rig(status("paused"))
    r.open()
    assert r.press("robot.pause", 0.7) == [ui.Request(UiKind.RESUME)] and not r.panel.open


def test_calibrate_engages_and_closes_without_a_resume():
    r = Rig(status("active"))
    r.open()
    r.st = status("paused")
    assert r.press("robot.calibrate", 0.7) == [ui.Request(UiKind.CALIBRATE)] and not r.panel.open


# ---------------------------------------------------------------------------------------------------- hands

def hand_joints(wrist, palm_normal, fingers, side):
    """25 WebXR joints of a flat hand: wrist, metacarpals spread across the palm, fingers along `fingers`."""
    n, f = np.asarray(palm_normal, float), np.asarray(fingers, float)
    across = np.cross(f, n)                                  # right hand: toward the thumb ... sign by side below
    across = across if side == "right" else -across          # +across = toward the thumb side
    j = np.zeros((25, 3))
    j[0] = wrist
    for k in range(1, 5):                                    # thumb
        j[k] = wrist + 0.03 * across + 0.02 * k * (f + across) / np.sqrt(2)
    for finger, off in enumerate((0.025, 0.008, -0.008, -0.024)):
        base = 5 + 5 * finger
        for k in range(5):
            j[base + k] = wrist + off * across + (0.07 + 0.025 * k) * f
    return j


def test_palm_normal_points_out_of_the_palm_for_either_hand():
    for side in ("left", "right"):
        j = hand_joints([0, 0, 1.0], [0, 0, -1.0], [1.0, 0, 0], side)                # palm down
        assert np.allclose(ui.palm_normal(j, side), [0, 0, -1.0], atol=1e-9)


def test_left_palm_toward_the_eyes_opens_after_the_hold_and_rearms():
    r = Rig()
    palm = EYE + [0.3, 0.1, -0.3]
    up = hand_joints(palm, (EYE - palm) / np.linalg.norm(EYE - palm), [0.3, 0.6, 0.5], "left")
    r.left_hand = up
    r.run(0.3)
    assert not r.panel.open
    r.run(0.15)
    assert r.panel.open
    r.press("close", 0.05)
    r.run(1.0)
    assert not r.panel.open                                  # palm still up: not re-opened until it turns away
    r.left_hand = hand_joints(palm, [0, 0, -1.0], [1.0, 0, 0], "left")
    r.run(0.1)
    r.left_hand = up
    r.run(0.5)
    assert r.panel.open


def test_hand_ray_from_the_shoulder_through_the_knuckle_and_pinch_click():
    r = Rig(status("off"))
    r.open()
    it = r.panel.item("robot.start")
    target = ui.panel_point(r.panel.T, (it.rect[0] + it.rect[2]) / 2, (it.rect[1] + it.rect[3]) / 2, P)
    sh = ui.shoulder(pose(np.eye(3), EYE), "right", P)
    knuckle = sh + 0.45 * (target - sh)
    j = hand_joints(knuckle - [0.08, 0.0, 0.0], [0, 0, -1.0], [1.0, 0, 0], "right")
    j += knuckle - j[ui.INDEX_KNUCKLE]
    r.hand = (j, 0.06)
    r.step()
    assert r.panel.hover == {"right": "robot.start"}
    assert r.panel.describe()["pointers"]["right"]["pressed"] is False
    r.hand = (j, 0.01)                                       # pinch
    reqs = r.run(0.5)
    r.hand = (j, 0.025)                                      # opening a little (inside the hysteresis): still held
    reqs += r.run(0.2)
    assert reqs == [ui.Request(UiKind.START)]
    r.hand = (j, 0.05)
    r.step()
    assert not r.panel.describe()["pointers"]["right"]["pressed"]


def test_stale_pointer_sources_are_ignored():
    r = Rig()
    r.open()
    frame = r.frame()
    later = r.t + 1.0                                        # every sample now 1 s old
    assert ui.pointers(frame, later, {}, P) == {}


# ---------------------------------------------------------------------------------------------------- pictures

def test_pictures_render():
    from xr_camera_teleop.operator.quest import ui_draw
    r = Rig()
    r.open()
    r.point_at("robot.calibrate")
    r.step()
    jpg = ui_draw.render_panel(r.panel.picture(), P.width, P.height)
    from xr_camera_teleop.operator.quest.convert import jpeg_size
    assert jpeg_size(jpg) == (round(P.width * ui_draw.PX_PER_M), round(P.height * ui_draw.PX_PER_M))
    strip, aspect = ui_draw.render_strip(r.panel.strip())
    assert jpeg_size(strip) is not None and aspect > 3
    assert r.panel.strip().segments[0] == ("HAZIR", "ok")
    silent = ui.Panel(P)
    silent.step(1.0, None, None, None, "ego")
    assert silent.strip().segments[0][1] == "bad"

