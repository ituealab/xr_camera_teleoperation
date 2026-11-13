"""Pixels of the headset panel and status strip: `PanelPicture` / `StripPicture` (operator/quest/ui.py) -> JPEG.

One job: draw. What is on the panel, where, and in which state is ui.py's; the server upserts the JPEG as a
world-locked Vuer `Image` (panel) or a head-locked image (strip, vuer_app.head_image). Drawing on the server with a TrueType
font keeps the Turkish labels exact in the headset (Vuer's text elements would need a font download on the Quest).

The panel image maps 1:1 onto the panel rectangle: PX_PER_M pixels per metre, origin top left, y down, the same
coordinates as ui.py's layout. At the panel's 0.45 m that is about the Quest 3S's display resolution.
"""
from __future__ import annotations

import io
from functools import lru_cache

from PIL import Image, ImageDraw, ImageFont

from xr_camera_teleop.operator.quest.ui import PanelPicture, StripPicture

PX_PER_M = 2400
JPEG_QUALITY = 85
FONT_DIRS = ("/usr/share/fonts/truetype/noto", "/usr/share/fonts/truetype/dejavu")
FONT_FILES = {False: ("NotoSans-Regular.ttf", "DejaVuSans.ttf"), True: ("NotoSans-Bold.ttf", "DejaVuSans-Bold.ttf")}

BG = (22, 26, 31)
TEXT = (232, 236, 240)
DIM_TEXT = (110, 120, 130)
FILL = {"normal": (55, 66, 77), "primary": (32, 104, 64), "danger": (190, 36, 36)}
DISABLED = (36, 41, 47)
ON = (21, 101, 192)
HOVER_EDGE = (255, 255, 255)
RING = (255, 214, 0)
TONES = {"ok": (120, 220, 140), "warn": (255, 200, 60), "bad": (255, 95, 95), "dim": (150, 160, 170)}


@lru_cache(maxsize=None)
def font(size_px: int, bold: bool = False):
    import os
    for d in FONT_DIRS:
        for name in FONT_FILES[bold]:
            path = os.path.join(d, name)
            if os.path.exists(path):
                return ImageFont.truetype(path, size_px)
    return ImageFont.load_default()


def _px(v: float) -> int:
    return int(round(v * PX_PER_M))


def _box(rect):
    x0, y0, x1, y1 = rect
    return [_px(x0), _px(y0), _px(x1), _px(y1)]


def _lighter(c, k=1.25):
    return tuple(min(255, int(v * k + 12)) for v in c)


def _jpeg(img: Image.Image) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def _fit(draw, text: str, max_w: int, size: int, bold: bool):
    while size > 10:
        f = font(size, bold)
        if draw.textlength(text, font=f) <= max_w:
            return f
        size -= 2
    return font(size, bold)


def render_panel(pic: PanelPicture, width_m: float, height_m: float) -> bytes:
    img = Image.new("RGB", (_px(width_m), _px(height_m)), BG)
    d = ImageDraw.Draw(img)
    d.rounded_rectangle([0, 0, img.width - 1, img.height - 1], radius=24, outline=(70, 80, 92), width=3)
    for it in pic.items:
        box = _box(it.rect)
        h = box[3] - box[1]
        if not it.enabled:
            fill = DISABLED
        elif it.on:
            fill = ON
        else:
            fill = FILL.get(it.style, FILL["normal"])
        if it.enabled and it.hovered:
            fill = _lighter(fill)
        if it.flash:
            fill = _lighter(fill, 1.6)
        edge = HOVER_EDGE if it.hovered and it.enabled else None
        d.rounded_rectangle(box, radius=min(18, h // 3), fill=fill, outline=edge, width=4 if edge else 0)
        size = int(h * {"stepper": 0.62, "toggle": 0.5}.get(it.kind, 0.42))
        f = _fit(d, it.label, box[2] - box[0] - 16, size, bold=it.style == "danger" or it.kind == "stepper")
        d.text(((box[0] + box[2]) / 2, (box[1] + box[3]) / 2), it.label, font=f, anchor="mm",
               fill=TEXT if it.enabled else DIM_TEXT)
        if it.progress > 0:                       # hold-to-confirm ring at the right end
            r = h * 0.32
            cx, cy = box[2] - h * 0.5, (box[1] + box[3]) / 2
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(0, 0, 0), width=7)
            d.arc([cx - r, cy - r, cx + r, cy + r], -90, -90 + 360 * it.progress, fill=RING, width=7)
    for rect, lines in pic.texts:
        box = _box(rect)
        if not lines:
            continue
        if len(lines) == 2 and box[3] - box[1] < 110:      # knob row: label left, value right
            h = box[3] - box[1]
            f = font(int(h * 0.5))
            d.text((box[0], (box[1] + box[3]) / 2), lines[0], font=f, anchor="lm", fill=TEXT)
            tone = {"AÇIK": "ok", "KAPALI": "dim"}.get(lines[1], "warn")
            d.text((box[2] - 8, (box[1] + box[3]) / 2), lines[1], font=font(int(h * 0.5), True), anchor="rm",
                   fill=TONES[tone])
            continue
        size = 30
        step = int(size * 1.3)
        n_fit = max(1, (box[3] - box[1]) // step)
        for k, line in enumerate(lines[:n_fit]):
            f = _fit(d, line, box[2] - box[0], size, bold=False)
            d.text((box[0], box[1] + k * step), line, font=f, fill=TEXT)
    for x, y, pressed in pic.cursors:
        cx, cy, r = _px(x), _px(y), 11
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(60, 220, 120) if pressed else None, outline=(255, 255, 255),
                  width=4)
    return _jpeg(img)


STRIP_H_PX = 56


def render_strip(pic: StripPicture) -> tuple[bytes, float]:
    """(JPEG, aspect width/height) of the one-line status strip."""
    f = font(int(STRIP_H_PX * 0.55), True)
    probe = ImageDraw.Draw(Image.new("RGB", (8, 8)))
    gap = 28
    widths = [probe.textlength(t, font=f) for t, _ in pic.segments]
    w = int(sum(widths) + gap * (len(widths) + 1))
    img = Image.new("RGB", (max(w, 64), STRIP_H_PX), BG)
    d = ImageDraw.Draw(img)
    x = gap
    for (text, tone), tw in zip(pic.segments, widths):
        d.text((x, STRIP_H_PX / 2), text, font=f, anchor="lm", fill=TONES.get(tone, TEXT))
        x += tw + gap
    return _jpeg(img), img.width / img.height

