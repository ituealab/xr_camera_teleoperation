"""Course geometry of the sim scene (PLAN §7): features of configs/sim_courses.yaml -> primitive parts in the world.

Pure (numpy only); `sim/scene_builder.py` turns the parts into MjSpec elements. A *feature* is one obstacle built in
its own frame: x is the direction you walk onto it, from its near edge at x = 0; y is centred on its middle; z = 0 is
the floor. A *course* is a list of placements of features:

    {feature: ramp5, name: ramp5, pos: [x, y], yaw_deg: 90}      # static obstacle, its frame put at (x, y, yaw)
    {feature: cube, atop: table_a, pos: [0.12, 0.0]}             # stacked: pos is in table_a's top frame

Feature types (their keys are the dataclass fields below, all in metres and degrees):
    ramp         up a slope, a plateau, down the same slope (three boxes)
    steps        `count` steps up, a landing, `count` steps down (stacked boxes)
    cross_slope  a lane whose surface is tilted sideways (the left edge high), eased in and out along x (heightfield)
    rough        random bumps up to `height`, smooth, zero at the border so there is no lip (heightfield)
    table        a top on four legs; the top surface is the frame children are stacked in
    cube         a free body (the graspable object) with its own contact parameters
    mark         a flat disc on the floor, visual only (no contact): where to stop, where a course starts
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields
from typing import Optional

import numpy as np

COLORS = {"ramp": (0.55, 0.62, 0.74, 1.0), "steps": (0.74, 0.63, 0.50, 1.0), "cross_slope": (0.64, 0.56, 0.74, 1.0),
          "rough": (0.52, 0.60, 0.46, 1.0), "table": (0.58, 0.43, 0.28, 1.0), "cube": (0.86, 0.18, 0.14, 1.0),
          "mark": (0.96, 0.80, 0.10, 1.0)}


@dataclass(frozen=True)
class Contact:
    """MuJoCo contact parameters of a geom (the contact uses the larger friction and condim of the two geoms)."""
    friction: tuple = (1.0, 0.005, 0.0001)      # sliding, torsional, rolling (MuJoCo's defaults)
    condim: int = 3
    solref: tuple = (0.02, 1.0)
    solimp: tuple = (0.9, 0.95, 0.001, 0.5, 2.0)


@dataclass(frozen=True)
class Part:
    """One MJCF element in the world. kind: box | disc | hfield | free_box (a body with a free joint)."""
    kind: str
    name: str
    pos: tuple                                  # world, of the geom centre (hfield: of its base plane centre)
    euler: tuple = (0.0, 0.0, 0.0)              # roll, pitch, yaw (rad) of Rz(yaw) Ry(pitch) Rx(roll)
    size: tuple = ()                            # box: half sizes; disc: radius, half height; hfield: rx, ry, top, base
    rgba: tuple = (0.6, 0.6, 0.6, 1.0)
    contact: Optional[Contact] = Contact()      # None: visual only
    elevation: Optional[np.ndarray] = None      # hfield: nrow x ncol in [0, 1], row 0 at -y, column 0 at -x
    mass: float = 0.0                           # free_box


@dataclass(frozen=True)
class Support:
    """Where children are stacked: the frame of a top surface (origin at its near edge centre, z on the surface)."""
    x: float
    z: float


# ---------------------------------------------------------------------------------------------------- features

@dataclass(frozen=True)
class Ramp:
    angle_deg: float
    rise: float
    plateau: float
    width: float
    thickness: float = 0.04

    @property
    def run(self) -> float:
        return self.rise / math.tan(math.radians(self.angle_deg))

    @property
    def length(self) -> float:
        return 2 * self.run + self.plateau

    def surface(self, x: float) -> float:
        r = self.run
        if x <= 0 or x >= self.length:
            return 0.0
        if x < r:
            return x / r * self.rise
        if x <= r + self.plateau:
            return self.rise
        return (self.length - x) / r * self.rise

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        th = math.radians(self.angle_deg)
        slope_half = self.rise / math.sin(th) / 2
        out = []
        for name, x0, sign in (("up", 0.0, 1.0), ("down", self.length, -1.0)):
            mid_top = np.array([x0 + sign * self.run / 2, 0.0, self.rise / 2])       # middle of the sloped surface
            normal = np.array([-sign * math.sin(th), 0.0, math.cos(th)])
            c = mid_top - normal * self.thickness / 2
            out.append(Part("box", name, tuple(c), (0.0, -sign * th, 0.0),
                            (slope_half, self.width / 2, self.thickness / 2), rgba, contact))
        out.append(Part("box", "plateau", (self.run + self.plateau / 2, 0.0, self.rise / 2), (0.0, 0.0, 0.0),
                        (self.plateau / 2, self.width / 2, self.rise / 2), rgba, contact))
        return out, None


@dataclass(frozen=True)
class Steps:
    rise: float
    count: int
    tread: float
    landing: float
    width: float

    @property
    def length(self) -> float:
        return 2 * (self.count - 1) * self.tread + self.landing

    def surface(self, x: float) -> float:
        if x <= 0 or x >= self.length:
            return 0.0
        k = min(int(min(x, self.length - x) // self.tread) + 1, self.count)
        return k * self.rise

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        out = []
        for k in range(1, self.count + 1):                 # layer k: one step higher, one tread shorter each end
            x0, x1 = (k - 1) * self.tread, self.length - (k - 1) * self.tread
            out.append(Part("box", f"layer{k}", ((x0 + x1) / 2, 0.0, (k - 0.5) * self.rise), (0.0, 0.0, 0.0),
                            ((x1 - x0) / 2, self.width / 2, self.rise / 2), rgba, contact))
        return out, None


def _ease(u: np.ndarray) -> np.ndarray:
    u = np.clip(u, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * u)


def _hfield_part(length: float, width: float, z: np.ndarray, base: float, rgba, contact) -> Part:
    top = float(z.max())
    return Part("hfield", "surface", (length / 2, 0.0, 0.0), (0.0, 0.0, 0.0), (length / 2, width / 2, top, base),
                rgba, contact, elevation=z / top)


@dataclass(frozen=True)
class CrossSlope:
    angle_deg: float
    length: float
    width: float
    lead: float                                  # ease in / out along x
    cell: float = 0.05
    base: float = 0.05

    def grid(self) -> tuple[np.ndarray, np.ndarray]:
        nx, ny = int(round(self.length / self.cell)) + 1, int(round(self.width / self.cell)) + 1
        return np.linspace(0.0, self.length, nx), np.linspace(-self.width / 2, self.width / 2, ny)

    def heights(self) -> np.ndarray:
        x, y = self.grid()
        along = _ease(x / self.lead) * _ease((self.length - x) / self.lead)
        across = math.tan(math.radians(self.angle_deg)) * (y + self.width / 2)       # 0 at the right edge (-y)
        return across[:, None] * along[None, :]

    def surface(self, x: float, y: float) -> float:
        a = float(_ease(np.array(x / self.lead)) * _ease(np.array((self.length - x) / self.lead)))
        return math.tan(math.radians(self.angle_deg)) * (y + self.width / 2) * a

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        return [_hfield_part(self.length, self.width, self.heights(), self.base, rgba, contact)], None


@dataclass(frozen=True)
class Rough:
    height: float                                # bumps from 0 to this
    length: float
    width: float
    cell: float = 0.025
    bump: float = 0.06                           # bump size (Gaussian smoothing sigma), m
    border: float = 0.2                          # bumps ease in from 0 at the edges over this
    seed: int = 1
    base: float = 0.05

    def heights(self) -> np.ndarray:
        nx, ny = int(round(self.length / self.cell)) + 1, int(round(self.width / self.cell)) + 1
        noise = np.random.default_rng(self.seed).standard_normal((ny, nx))
        sigma = self.bump / self.cell
        k = np.exp(-0.5 * (np.arange(-3 * int(sigma + 1), 3 * int(sigma + 1) + 1) / sigma) ** 2)
        k /= k.sum()
        smooth = np.apply_along_axis(lambda r: np.convolve(r, k, mode="same"), 1, noise)
        smooth = np.apply_along_axis(lambda c: np.convolve(c, k, mode="same"), 0, smooth)
        smooth = (smooth - smooth.min()) / (smooth.max() - smooth.min())
        x, y = np.linspace(0, self.length, nx), np.linspace(0, self.width, ny)
        wx = _ease(np.minimum(x, self.length - x) / self.border)
        wy = _ease(np.minimum(y, self.width - y) / self.border)
        z = smooth * wy[:, None] * wx[None, :]
        return self.height * z / z.max()

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        return [_hfield_part(self.length, self.width, self.heights(), self.base, rgba, contact)], None


@dataclass(frozen=True)
class Table:
    depth: float                                 # along x, from the near edge
    width: float
    height: float                                # of the top surface
    top_thickness: float = 0.04
    leg: float = 0.04
    leg_inset: float = 0.05

    def parts(self, rgba, contact) -> tuple[list[Part], Support]:
        t = self.top_thickness
        out = [Part("box", "top", (self.depth / 2, 0.0, self.height - t / 2), (0.0, 0.0, 0.0),
                    (self.depth / 2, self.width / 2, t / 2), rgba, contact)]
        legs_h = self.height - t
        xi, yi = self.leg_inset, self.width / 2 - self.leg_inset
        for k, (x, y) in enumerate(((xi, yi), (xi, -yi), (self.depth - xi, yi), (self.depth - xi, -yi))):
            out.append(Part("box", f"leg{k}", (x, y, legs_h / 2), (0.0, 0.0, 0.0),
                            (self.leg / 2, self.leg / 2, legs_h / 2), rgba, contact))
        return out, Support(0.0, self.height)


@dataclass(frozen=True)
class Cube:
    size: float
    mass: float

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        h = self.size / 2
        return [Part("free_box", "body", (0.0, 0.0, h), (0.0, 0.0, 0.0), (h, h, h), rgba, contact, mass=self.mass)], None


@dataclass(frozen=True)
class Mark:
    radius: float

    def parts(self, rgba, contact) -> tuple[list[Part], None]:
        return [Part("disc", "disc", (0.0, 0.0, 0.0015), (0.0, 0.0, 0.0), (self.radius, 0.0015), rgba, None)], None


TYPES = {"ramp": Ramp, "steps": Steps, "cross_slope": CrossSlope, "rough": Rough, "table": Table, "cube": Cube,
         "mark": Mark}


@dataclass(frozen=True)
class Feature:
    type: str
    shape: object
    rgba: tuple
    contact: Optional[Contact]


def feature(cfg: dict) -> Feature:
    """A feature of configs/sim_courses.yaml: {type, <shape keys>, [rgba], [contact: {friction, condim, ...}]}."""
    cfg = dict(cfg)
    kind = cfg.pop("type")
    if kind not in TYPES:
        raise ValueError(f"unknown feature type {kind!r}; known: {', '.join(TYPES)}")
    rgba = tuple(map(float, cfg.pop("rgba", COLORS[kind])))
    contact_cfg = cfg.pop("contact", {})
    unknown = set(contact_cfg) - {f.name for f in fields(Contact)}
    if unknown:
        raise KeyError(f"{kind}: unknown contact keys {sorted(unknown)}")
    contact = None if kind == "mark" else Contact(**{k: (tuple(v) if isinstance(v, list) else v)
                                                   for k, v in contact_cfg.items()})
    cls = TYPES[kind]
    unknown = set(cfg) - {f.name for f in fields(cls)}
    if unknown:
        raise KeyError(f"{kind}: unknown keys {sorted(unknown)}")
    return Feature(kind, cls(**cfg), rgba, contact)


# ---------------------------------------------------------------------------------------------------- placement

@dataclass(frozen=True)
class Frame:
    x: float
    y: float
    z: float
    yaw: float

    def apply(self, p) -> tuple:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c * p[0] - s * p[1], self.y + s * p[0] + c * p[1], self.z + p[2])


@dataclass(frozen=True)
class Placed:
    name: str
    feature: str
    frame: Frame
    parts: list = field(default_factory=list)
    support: Optional[Frame] = None


def place(course: list, features: dict) -> list[Placed]:
    """A course's placements -> parts in the world, in order; `atop` refers to an earlier placement."""
    placed: dict[str, Placed] = {}
    for i, entry in enumerate(course or []):
        entry = dict(entry)
        fname = entry.pop("feature")
        if fname not in features:
            raise KeyError(f"placement {i}: unknown feature {fname!r}")
        name = entry.pop("name", fname)
        if name in placed:
            raise ValueError(f"placement {i}: duplicate name {name!r} (give it a `name`)")
        x, y = map(float, entry.pop("pos", (0.0, 0.0)))
        yaw = math.radians(float(entry.pop("yaw_deg", 0.0)))
        on = entry.pop("atop", None)
        if entry:
            raise KeyError(f"placement {name}: unknown keys {sorted(entry)}")
        if on is None:
            frame = Frame(x, y, 0.0, yaw)
        else:
            if on not in placed or placed[on].support is None:
                raise KeyError(f"placement {name}: `atop: {on}` is not an earlier table")
            sup = placed[on].support
            wx, wy, wz = sup.apply((x, y, 0.0))
            frame = Frame(wx, wy, wz, sup.yaw + yaw)
        f = features[fname]
        local, support = f.shape.parts(f.rgba, f.contact)
        parts = [Part(p.kind, f"{name}_{p.name}", frame.apply(p.pos), (p.euler[0], p.euler[1], p.euler[2] + frame.yaw),
                      p.size, p.rgba, p.contact, p.elevation, p.mass) for p in local]
        sup_frame = None if support is None else Frame(*frame.apply((support.x, 0.0, support.z)), frame.yaw)
        placed[name] = Placed(name, fname, frame, parts, sup_frame)
    return list(placed.values())


def course_parts(cfg: dict, course: str) -> list[Part]:
    """All parts of a course of configs/sim_courses.yaml (`features`, `courses`)."""
    if course not in cfg["courses"]:
        raise KeyError(f"unknown course {course!r}; known: {', '.join(cfg['courses'])}")
    features = {k: feature(v) for k, v in (cfg.get("features") or {}).items()}
    return [p for pl in place(cfg["courses"][course], features) for p in pl.parts]
