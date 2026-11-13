"""F/T reading axes (SETUP > F/T + payload > Reading axes): which sensor axis each published axis reads.

For a sensor mounted turned on the flange: the broadcaster publishes a turned copy of the sensor's reading, so
the published Fx/Fy/Fz (and Tx/Ty/Tz) lie along the axes the robot model gives ft_fts_link. The robot model, TF
and everything that reads ~/wrench (the GUI, the payload fit, admittance, policies) then agree, with no other change.

force_torque_sensor_broadcaster (ros2_controllers 4.42) does it with its own parameters: interface_names.<axis>
(the sensor interface an axis reads; sensor_name empty) and multiplier.<axis> (±1). It publishes
(sensor value + offset) * multiplier. Rotations in 90° steps only: published X and Y each read one sensor axis
(±), Z = X × Y, the torques the same.

The mapping is kept in config_dir()/ft_axes.yaml, the broadcaster's parameters for its controller manager (no file:
the sensor's own axes). The launcher (ur7e_gui.sh) passes it to the stack as UR7E_FT_AXES_PARAMS; the stack loads
it after ft_controllers.yaml, so it applies at the next robot stack start.
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Tuple

FILE = "ft_axes.yaml"
XYZ = "xyz"
CHOICES = ["+x", "-x", "+y", "-y", "+z", "-z"]
SENSOR = "tcp_fts_sensor"                                # ft_controllers.yaml sensor_name
IDENTITY: List[Tuple[int, int]] = [(1, i) for i in range(6)]
# per published axis (Fx Fy Fz Tx Ty Tz): (sign, sensor axis index 0..5)
Mapping = List[Tuple[int, int]]
KINDS = ("force", "torque")


class FtAxesError(ValueError):
    pass


def _axis(s: str) -> Tuple[int, int]:
    s = str(s).strip().lower()
    if s not in CHOICES:
        raise FtAxesError(f"an axis is one of {', '.join(CHOICES)}, not '{s}'")
    return (1 if s[0] == "+" else -1), XYZ.index(s[1])


def mapping(x: str, y: str) -> Mapping:
    """Published X reads sensor axis x, published Y reads y (e.g. "-y", "+x"); Z completes a right-handed frame."""
    (sx, ix), (sy, iy) = _axis(x), _axis(y)
    if ix == iy:
        raise FtAxesError("X and Y must read two different sensor axes")
    ex = [0, 0, 0]
    ey = [0, 0, 0]
    ex[ix], ey[iy] = sx, sy
    ez = [ex[1] * ey[2] - ex[2] * ey[1], ex[2] * ey[0] - ex[0] * ey[2], ex[0] * ey[1] - ex[1] * ey[0]]
    iz = next(i for i in range(3) if ez[i])
    f = [(sx, ix), (sy, iy), (ez[iz], iz)]
    return f + [(s, i + 3) for s, i in f]


def label(m: Optional[Mapping]) -> Optional[Dict[str, str]]:
    """{"x": "-y", "y": "+x", "z": "+z"}: the sensor axis each published force axis reads."""
    if m is None:
        return None
    return {XYZ[k]: ("+" if m[k][0] > 0 else "-") + XYZ[m[k][1]] for k in range(3)}


def is_identity(m: Optional[Mapping]) -> bool:
    return m is not None and list(m) == IDENTITY


def param_names() -> List[str]:
    return (["sensor_name"] + [f"interface_names.{k}.{a}" for k in KINDS for a in XYZ]
            + [f"multiplier.{k}.{a}" for k in KINDS for a in XYZ])


def from_params(got: Optional[dict], sensor: str = SENSOR) -> Optional[Mapping]:
    """The mapping a broadcaster runs with, from its parameters (None: not readable or not a rotation)."""
    if got is None:
        return None
    mult = []
    for k in KINDS:
        for a in XYZ:
            v = got.get(f"multiplier.{k}.{a}")
            mult.append(1 if v is None else (1 if float(v) >= 0 else -1))   # older broadcaster: no multipliers
    if got.get("sensor_name"):
        src = list(range(6))
    else:
        src = []
        for j, (k, a) in enumerate((k, a) for k in KINDS for a in XYZ):
            name = str(got.get(f"interface_names.{k}.{a}") or "")
            tail = name.rsplit("/", 1)[-1]                  # "<sensor>/force.y"
            kind, _, ax = tail.partition(".")
            if kind not in KINDS or ax not in XYZ:
                return None
            src.append(KINDS.index(kind) * 3 + XYZ.index(ax))
    return [(mult[j], src[j]) for j in range(6)]


def apply(m: Mapping, raw: List[float], offset: Optional[List[float]] = None) -> List[float]:
    """What the broadcaster publishes: (sensor value + offset) * multiplier, per published axis."""
    off = offset or [0.0] * 6
    return [(float(raw[i]) + float(off[j])) * s for j, (s, i) in enumerate(m)]


def convert(bias: List[float], src: Optional[Mapping], dst: Optional[Mapping]) -> List[float]:
    """A bias (published units: published = sign * sensor - bias) made with mapping src, for mapping dst."""
    src, dst = src or IDENTITY, dst or IDENTITY
    if list(src) == list(dst):
        return list(bias)
    on_sensor = [0.0] * 6
    for j, (s, i) in enumerate(src):
        on_sensor[i] = s * float(bias[j])
    return [s * on_sensor[i] + 0.0 for s, i in dst]


def encode(m: Optional[Mapping]) -> Optional[list]:
    return None if m is None else [[int(s), int(i)] for s, i in m]


def decode(v) -> Optional[Mapping]:
    try:
        m = [(1 if int(s) > 0 else -1, int(i)) for s, i in v]
    except (TypeError, ValueError):
        return None
    return m if len(m) == 6 and sorted(i for _, i in m) == list(range(6)) else None


def yaml_text(m: Mapping, node: str, sensor: str = SENSOR, frame_id: str = "ft_fts_link") -> str:
    key = node.lstrip("/")
    lines = [f"# SETUP > F/T + payload > Reading axes (moveit_gui ft_axes.py): published X reads sensor "
             f"{label(m)['x']}, Y reads {label(m)['y']}, Z reads {label(m)['z']}",
             f"{key}:", "  ros__parameters:", f"    frame_id: {frame_id}", '    sensor_name: ""', "    interface_names:"]
    for k in KINDS:
        lines.append(f"      {k}:")
        for a in XYZ:
            s, i = m[KINDS.index(k) * 3 + XYZ.index(a)]
            lines.append(f"        {a}: {sensor}/{KINDS[i // 3]}.{XYZ[i % 3]}")
    lines.append("    multiplier:")
    for k in KINDS:
        lines.append(f"      {k}:")
        for a in XYZ:
            s, _ = m[KINDS.index(k) * 3 + XYZ.index(a)]
            lines.append(f"        {a}: {float(s):.1f}")
    return "\n".join(lines) + "\n"


def read_next(cfg: Path, node: str) -> Mapping:
    """The mapping the next stack start loads (identity without a file or with one this module cannot read)."""
    p = Path(cfg) / FILE
    try:
        import yaml
        d = yaml.safe_load(p.read_text()) or {}
        params = (d.get(node.lstrip("/")) or {}).get("ros__parameters") or {}
    except Exception:                                    # no file, or not one this module wrote
        return list(IDENTITY)
    flat = {"sensor_name": params.get("sensor_name")}
    for grp in ("interface_names", "multiplier"):
        for k in KINDS:
            for a in XYZ:
                flat[f"{grp}.{k}.{a}"] = ((params.get(grp) or {}).get(k) or {}).get(a)
    return from_params(flat) or list(IDENTITY)


def write_next(cfg: Path, m: Mapping, node: str):
    p = Path(cfg) / FILE
    p.parent.mkdir(parents=True, exist_ok=True)
    if is_identity(m):                                    # the sensor's own axes need no file
        try:
            p.unlink()
        except FileNotFoundError:
            pass
        return
    tmp = p.with_suffix(".tmp")
    tmp.write_text(yaml_text(m, node))
    tmp.replace(p)


def rotation_deg(m: Mapping) -> Optional[str]:
    """A short name for common mappings ("turned 90° about Z"), else None."""
    names = {("+y", "-x", "+z"): "turned −90° about Z", ("-y", "+x", "+z"): "turned +90° about Z",
             ("-x", "-y", "+z"): "turned 180° about Z", ("+x", "+y", "+z"): "the sensor's own axes"}
    lb = label(m)
    return names.get((lb["x"], lb["y"], lb["z"])) if lb else None

