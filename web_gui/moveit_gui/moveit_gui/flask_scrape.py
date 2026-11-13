"""Pure NumPy pose generation for scraping inside a round-bottom flask.

Tool TCPs are contact points. Straight-tool +Z points from shaft to tip. For a
bent tool +Z points from elbow to tip and the bend lies in local XZ. The shaft
points along ``-sin(a) X + cos(a) Z`` toward the elbow.

In SETUP > TCPs, create the named bent-tool TCP at the physical contact tip.
Its XYZ is the measured tip position in the flange/gripper TCP. For rotation,
measure unit vectors ``d`` along shaft-to-elbow and ``e`` along elbow-to-tip.
The TCP axes in the parent frame are ``Z=e``, ``X=(cos(a)e-d)/sin(a)``, and
``Y=Z cross X``. Enter that rotation as the SETUP roll/pitch/yaw convention.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Union

import numpy as np

from . import transforms as tf
from .flask_geometry import FlaskGeometry, T_base_flask
from .geometry import PoseSpec
from .program import Program, ProgramWaypoint, Step


class FlaskScrapeError(ValueError):
    pass


def _positive(value: float, name: str, zero: bool = False) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        raise FlaskScrapeError(f"{name} must be a number in metres") from None
    if not math.isfinite(out) or out < 0.0 or (out == 0.0 and not zero):
        raise FlaskScrapeError(f"{name} must be {'non-negative' if zero else 'positive'}")
    return out


@dataclass(frozen=True)
class StraightTool:
    tcp: str
    shaft_radius: float
    shaft_length: float

    def __post_init__(self):
        if not self.tcp:
            raise FlaskScrapeError("tool TCP name must not be empty")
        object.__setattr__(self, "shaft_radius", _positive(self.shaft_radius, "shaft radius"))
        object.__setattr__(self, "shaft_length", _positive(self.shaft_length, "shaft length"))


@dataclass(frozen=True)
class BentTool:
    """Rigid shaft and end; TCP +Z is elbow-to-contact-tip."""

    tcp: str
    shaft_radius: float
    shaft_length: float
    bend_length: float
    bend_angle_deg: float
    tip_radius: float = 0.0

    def __post_init__(self):
        if not self.tcp:
            raise FlaskScrapeError("tool TCP name must not be empty")
        object.__setattr__(self, "shaft_radius", _positive(self.shaft_radius, "shaft radius"))
        object.__setattr__(self, "shaft_length", _positive(self.shaft_length, "shaft length"))
        object.__setattr__(self, "bend_length", _positive(self.bend_length, "bend length"))
        object.__setattr__(self, "tip_radius", _positive(self.tip_radius, "tip radius", True))
        angle = float(self.bend_angle_deg)
        if not math.isfinite(angle) or not 0.0 < angle < 90.0:
            raise FlaskScrapeError("bend angle must be between 0 and 90 degrees")
        object.__setattr__(self, "bend_angle_deg", angle)

    @property
    def contact_radius(self) -> float:
        return self.tip_radius or self.shaft_radius

    @property
    def lateral_offset(self) -> float:
        return self.bend_length * math.sin(math.radians(self.bend_angle_deg))


ToolModel = Union[StraightTool, BentTool]


@dataclass(frozen=True)
class BendSuggestion:
    bend_length_range_m: tuple[float, float]
    bend_angle_range_deg: tuple[float, float]
    recommended_length_m: float
    recommended_angle_deg: float
    reachable_band_deg: Optional[tuple[float, float]]
    target_band_deg: tuple[float, float]
    meets_target: bool

    def tool(self, tcp: str, shaft_radius: float, shaft_length: float,
             tip_radius: float = 0.0) -> BentTool:
        return BentTool(tcp, shaft_radius, shaft_length, self.recommended_length_m,
                        self.recommended_angle_deg, tip_radius)


PATTERNS = ("down", "rings")      # down: strokes from high on the wall to the bottom, like the vial scrape


@dataclass
class ScrapePose:
    name: str
    ring: int                    # the group the pose is scraped in: a ring, or one top-to-bottom stroke
    theta_deg: float
    azimuth_deg: float
    pose: PoseSpec


@dataclass
class UnreachablePose:
    ring: int
    theta_deg: float
    azimuth_deg: float
    reason: str
    pose: Optional[PoseSpec] = None


@dataclass
class ScrapePlan:
    reachable: list[ScrapePose]
    unreachable: list[UnreachablePose]
    pivot: np.ndarray
    geometry: FlaskGeometry
    tool: ToolModel
    margin: float
    press_depth: float
    reference_rotation: np.ndarray = field(repr=False)

    @property
    def poses(self) -> list[ScrapePose]:
        return self.reachable


@dataclass
class FlaskScrape:
    geometry: FlaskGeometry
    tool: ToolModel
    theta_min_deg: float
    theta_max_deg: float
    rings: int
    step_deg: float
    press_depth_mm: float = 0.0
    margin_mm: float = 1.0
    pivot_depth_m: Optional[float] = None
    reference_orientation: Optional[np.ndarray] = None
    contact_preference: str = "normal"
    pattern: str = "down"

    def generate(self) -> ScrapePlan:
        return generate_scrape_poses(
            self.geometry, self.tool, self.theta_min_deg, self.theta_max_deg,
            self.rings, self.step_deg, press_depth_mm=self.press_depth_mm,
            margin_m=self.margin_mm / 1000.0, pivot_depth_m=self.pivot_depth_m,
            reference_orientation=self.reference_orientation,
            contact_preference=self.contact_preference, pattern=self.pattern)


def _flask_values(g: FlaskGeometry) -> tuple[np.ndarray, np.ndarray, float, float]:
    T = T_base_flask(g)
    rim = T[:3, :3].T @ (g.rim_center - g.sphere_center)
    return T, rim, g.neck_join_height, float(rim[2])


def pivot_point(geometry: FlaskGeometry, pivot_depth_m: Optional[float] = None) -> np.ndarray:
    """Pivot in ``flask``; default is the bulb-side end of the neck."""
    _, rim, join, rim_z = _flask_values(geometry)
    z = join if pivot_depth_m is None else rim_z - float(pivot_depth_m)
    if not math.isfinite(z):
        raise FlaskScrapeError("pivot depth must be finite")
    if z < join - 1e-9 or z > rim_z + 1e-9:
        raise FlaskScrapeError("pivot must lie inside the neck")
    return np.array([rim[0], rim[1], z], dtype=float)


def _tool_pivot(geometry: FlaskGeometry, tool: ToolModel,
                pivot_depth_m: Optional[float]) -> np.ndarray:
    # Put the RCM near the bulb while retaining a short constrained neck span.
    if pivot_depth_m is None and isinstance(tool, BentTool):
        pivot_depth_m = 0.90 * geometry.neck_length
    return pivot_point(geometry, pivot_depth_m)


def _rotation_value(value: Optional[np.ndarray]) -> np.ndarray:
    if value is None:
        return np.eye(3)
    if isinstance(value, PoseSpec):
        R = value.T()[:3, :3]
    else:
        a = np.asarray(value, dtype=float)
        if a.shape == (4, 4):
            R = a[:3, :3]
        elif a.shape == (3, 3):
            R = a
        elif a.shape == (4,):
            R = tf.quat_to_matrix(a)
        else:
            raise FlaskScrapeError("reference orientation must be a pose, rotation matrix, or quaternion")
    if not np.all(np.isfinite(R)) or not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise FlaskScrapeError("reference orientation is not a rotation")
    return np.array(R, dtype=float)


def _minimal_twist(z: np.ndarray, reference: np.ndarray) -> np.ndarray:
    z = np.asarray(z, dtype=float)
    z /= np.linalg.norm(z)
    for candidate in (reference[:, 0], reference[:, 1], np.eye(3)[0], np.eye(3)[1]):
        x = candidate - z * float(candidate @ z)
        if np.linalg.norm(x) > 1e-7:
            x /= np.linalg.norm(x)
            break
    else:
        raise FlaskScrapeError("cannot construct tool orientation")
    y = np.cross(z, x)
    y /= np.linalg.norm(y)
    return np.column_stack((np.cross(y, z), y, z))


def insertion_clearance_reason(geometry: FlaskGeometry, tool: ToolModel,
                               margin_m: float = 0.001) -> Optional[str]:
    """Check the axial insertion envelope before any in-bulb tilt."""
    if margin_m < 0.0 or not math.isfinite(margin_m):
        raise FlaskScrapeError("clearance margin must be non-negative")
    if isinstance(tool, BentTool):
        needed = (tool.lateral_offset + tool.shaft_radius + tool.contact_radius
                  + 2.0 * margin_m)
    else:
        needed = 2.0 * (tool.shaft_radius + margin_m)
    if needed <= 2.0 * geometry.neck_inner_radius + 1e-12:
        return None
    if isinstance(tool, BentTool):
        return ("The bent end cannot pass the neck bore: bend_length*sin(angle) "
                "plus both radii and margins exceeds the bore diameter.")
    return "The shaft is too wide for the neck clearance."


def _bent_parts(T: np.ndarray, tool: BentTool):
    tip = T[:3, 3]
    end_axis = T[:3, 2]
    angle = math.radians(tool.bend_angle_deg)
    shaft_axis = T[:3, :3] @ np.array([-math.sin(angle), 0.0, math.cos(angle)])
    return tip, tip - tool.bend_length * end_axis, shaft_axis, end_axis


def _neck_clearance(elbow: np.ndarray, axis: np.ndarray, geometry: FlaskGeometry,
                    tool: ToolModel, margin: float) -> Optional[str]:
    _, rim, join_z, rim_z = _flask_values(geometry)
    if abs(float(axis[2])) < 1e-9:
        return "The shaft cannot pass up through the neck."
    s_join = (elbow[2] - join_z) / axis[2]
    s_rim = (elbow[2] - rim_z) / axis[2]
    if s_rim < -1e-8 or s_rim > tool.shaft_length + 1e-9:
        return "The shaft is too short to span the neck."
    lo = max(0.0, min(s_join, s_rim))
    hi = min(tool.shaft_length, max(s_join, s_rim))
    allowed = geometry.neck_inner_radius - margin - tool.shaft_radius / abs(float(axis[2]))
    if allowed < 0.0:
        return "The shaft is too wide for the neck clearance."
    axis_xy = rim[:2]
    ends = ((elbow - lo * axis)[:2], (elbow - hi * axis)[:2])
    if max(float(np.linalg.norm(p - axis_xy)) for p in ends) > allowed + 1e-9:
        return "The shaft does not stay inside the neck bore."
    return None


def _segment_inside(a: np.ndarray, b: np.ndarray, radius: float,
                    sphere_radius: float, skip_end: float = 0.0) -> bool:
    length = float(np.linalg.norm(b - a))
    if length < 1e-12:
        return float(np.linalg.norm(a)) + radius <= sphere_radius + 1e-8
    direction = (b - a) / length
    usable = max(0.0, length - skip_end)
    for distance in np.linspace(0.0, usable, max(3, int(math.ceil(usable / 0.001)) + 1)):
        point = a + (b - a) * (distance / length)
        parallel = float(point @ direction)
        perpendicular = float(np.linalg.norm(point - parallel * direction))
        if math.hypot(parallel, perpendicular + radius) > sphere_radius + 1e-8:
            return False
    return True


def clearance_reason(T_flask_tcp: np.ndarray, geometry: FlaskGeometry, tool: ToolModel,
                     pivot: Optional[Sequence[float]] = None, margin_m: float = 0.001,
                     press_depth_m: float = 0.0) -> Optional[str]:
    """Return ``None`` when shaft, bend, neck, and bulb clear."""
    T = np.asarray(T_flask_tcp, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise FlaskScrapeError("tool pose must be a finite 4x4 transform")
    reason = insertion_clearance_reason(geometry, tool, margin_m)
    if reason:
        return reason
    pivot_v = _tool_pivot(geometry, tool, None) if pivot is None else np.asarray(pivot, dtype=float)
    if pivot_v.shape != (3,):
        raise FlaskScrapeError("pivot must have three values")
    if isinstance(tool, BentTool):
        tip, elbow, axis, _ = _bent_parts(T, tool)
    else:
        tip, elbow, axis = T[:3, 3], T[:3, 3], T[:3, 2]
    axis /= np.linalg.norm(axis)
    along = float((elbow - pivot_v) @ axis)
    miss = float(np.linalg.norm((elbow - pivot_v) - along * axis))
    if miss > 1e-5 or along < -1e-8:
        return "The shaft line does not pass through the pivot."
    reason = _neck_clearance(elbow, axis, geometry, tool, margin_m)
    if reason:
        return reason
    _, _, join_z, _ = _flask_values(geometry)
    s_open = max(0.0, (elbow[2] - join_z) / axis[2])
    sphere_limit = geometry.R_inner + max(0.0, press_depth_m)
    shaft_start = elbow
    if isinstance(tool, StraightTool):
        shaft_start = elbow - min(tool.shaft_radius, s_open) * axis
    if s_open > 1e-9 and not _segment_inside(
            shaft_start, elbow - s_open * axis, tool.shaft_radius, sphere_limit):
        return "The shaft touches the bulb wall away from the tip."
    if isinstance(tool, BentTool):
        if not _segment_inside(elbow, tip, tool.shaft_radius, sphere_limit,
                               max(tool.contact_radius, tool.shaft_radius)):
            return "The bent end touches the bulb wall away from the tip."
    return None


check_clearance = clearance_reason


def _bent_orientations(target: np.ndarray, normal: np.ndarray, pivot: np.ndarray,
                       tool: BentTool, reference: np.ndarray, preference: str) -> list[np.ndarray]:
    delta = target - pivot
    distance = float(np.linalg.norm(delta))
    lateral = tool.lateral_offset
    if distance <= lateral + 1e-10:
        return []
    rhat = delta / distance
    beta = math.asin(lateral / distance)
    e1 = normal - rhat * float(normal @ rhat)
    if np.linalg.norm(e1) < 1e-9:
        e1 = reference[:, 0] - rhat * float(reference[:, 0] @ rhat)
    if np.linalg.norm(e1) < 1e-9:
        e1 = _minimal_twist(rhat, reference)[:, 0]
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(rhat, e1)
    angle = math.radians(tool.bend_angle_deg)
    q = math.sqrt(distance * distance - lateral * lateral) - tool.bend_length * math.cos(angle)
    if q < -1e-9:
        return []
    candidates = []
    # Five-degree roll samples are deterministic and much finer than tool data.
    for roll in np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False):
        side = math.cos(roll) * e1 + math.sin(roll) * e2
        shaft = math.cos(beta) * rhat + math.sin(beta) * side
        elbow = pivot + max(0.0, q) * shaft
        end = (target - elbow) / tool.bend_length
        end /= np.linalg.norm(end)
        x = (math.cos(angle) * end - shaft) / math.sin(angle)
        x /= np.linalg.norm(x)
        y = np.cross(end, x)
        y /= np.linalg.norm(y)
        R = np.column_stack((x, y, end))
        contact = float(end @ normal)
        contact_cost = -contact if preference == "normal" else abs(contact)
        continuity = 3.0 - float(np.trace(reference.T @ R))
        candidates.append((contact_cost, continuity, roll, R))
    candidates.sort(key=lambda item: item[:3])
    return [item[3] for item in candidates]


def _target_pose(target, normal, pivot, tool, reference, preference, geometry, margin, press):
    if isinstance(tool, StraightTool):
        direction = target - pivot
        candidates = [] if np.linalg.norm(direction) < 1e-9 else [_minimal_twist(direction, reference)]
    else:
        candidates = _bent_orientations(target, normal, pivot, tool, reference, preference)
    if not candidates:
        return None, "The tool cannot geometrically reach the target."
    first = None
    for R in candidates:
        T = tf.from_rotation(R, target)
        reason = clearance_reason(T, geometry, tool, pivot, margin, press)
        if reason is None:
            return T, None
        first = first or reason
    return None, first or "No clear tool orientation reaches the target."


def generate_scrape_poses(geometry: FlaskGeometry, tool: ToolModel,
                          theta_min_deg: float, theta_max_deg: float, rings: int,
                          step_deg: float, *, press_depth_mm: float = 0.0,
                          margin_m: float = 0.001, pivot_depth_m: Optional[float] = None,
                          reference_orientation: Optional[np.ndarray] = None,
                          contact_preference: str = "normal", pattern: str = "down") -> ScrapePlan:
    """Generate the contact poses, ``rings`` polar angles from theta_min to theta_max, ``step_deg`` apart around.

    ``down`` (default), like the vial scrape: one stroke per azimuth, from the highest polar angle down to the
    lowest (toward the bottom), named stroke_<azimuth>_<level>. ``rings``: alternating rings around the axis from
    the bottom up, named ring_<ring>_<index>.

    Normal contact is the default because it gives a predictable force and
    avoids scraping on a tip edge. ``tangent`` is available for blade-like ends.
    """
    if not isinstance(tool, (StraightTool, BentTool)):
        raise FlaskScrapeError("tool must be a StraightTool or BentTool")
    if isinstance(rings, bool) or int(rings) != rings or int(rings) < 1:
        raise FlaskScrapeError("number of rings must be a positive whole number")
    rings = int(rings)
    if not all(math.isfinite(float(v)) for v in
               (theta_min_deg, theta_max_deg, step_deg, press_depth_mm, margin_m)):
        raise FlaskScrapeError("scrape dimensions must be finite")
    if not 0.0 <= theta_min_deg <= theta_max_deg <= 180.0:
        raise FlaskScrapeError("polar angles must satisfy 0 <= minimum <= maximum <= 180 degrees")
    if step_deg <= 0.0 or step_deg > 360.0:
        raise FlaskScrapeError("ring step must be in (0, 360] degrees")
    if press_depth_mm < 0.0 or margin_m < 0.0:
        raise FlaskScrapeError("press depth and clearance margin must be non-negative")
    if contact_preference not in ("normal", "tangent"):
        raise FlaskScrapeError("contact preference must be 'normal' or 'tangent'")
    if pattern not in PATTERNS:
        raise FlaskScrapeError(f"pattern must be one of {', '.join(PATTERNS)}")
    press = float(press_depth_mm) / 1000.0
    radius = geometry.R_inner + press
    pivot = _tool_pivot(geometry, tool, pivot_depth_m)
    reference = _rotation_value(reference_orientation)
    azimuths = list(np.arange(0.0, 360.0 - 1e-10, float(step_deg)))
    reachable, unreachable = [], []
    previous = reference
    insertion = insertion_clearance_reason(geometry, tool, margin_m)
    thetas = [float(t) for t in np.linspace(theta_min_deg, theta_max_deg, rings)]
    if pattern == "rings":
        groups = [("ring", [(t, a) for a in (azimuths if ring % 2 == 0 else azimuths[::-1])])
                  for ring, t in enumerate(thetas)]
    else:                         # one stroke per azimuth, top (largest polar angle) to bottom
        groups = [("stroke", [(t, a) for t in thetas[::-1]]) for a in azimuths]
    for ring, (prefix, order) in enumerate(groups):
        for index, (theta_deg, azimuth_deg) in enumerate(order):
            theta = math.radians(theta_deg)
            phi = math.radians(float(azimuth_deg))
            normal = np.array([math.sin(theta) * math.cos(phi),
                               math.sin(theta) * math.sin(phi), -math.cos(theta)])
            target = normal * radius
            if insertion:
                T, reason = None, insertion
            else:
                T, reason = _target_pose(target, normal, pivot, tool, previous,
                                         contact_preference, geometry, margin_m, press)
            if reason:
                unreachable.append(UnreachablePose(ring, float(theta_deg), float(azimuth_deg), reason))
                continue
            pose = PoseSpec.from_T("flask", tool.tcp, T)
            name = f"{prefix}_{ring + 1:02d}_{index + 1:03d}"
            reachable.append(ScrapePose(name, ring, float(theta_deg), float(azimuth_deg), pose))
            previous = T[:3, :3]
    return ScrapePlan(reachable, unreachable, pivot, geometry, tool, margin_m, press, reference)


generate = generate_scrape_poses


def reachable_band(geometry: FlaskGeometry, tool: ToolModel, *, margin_m: float = 0.001,
                   pivot_depth_m: Optional[float] = None, resolution_deg: float = 0.25,
                   azimuth_step_deg: float = 15.0,
                   contact_preference: str = "normal") -> Optional[tuple[float, float]]:
    """Largest polar band for which every sampled azimuth is reachable."""
    if resolution_deg <= 0.0 or azimuth_step_deg <= 0.0:
        raise FlaskScrapeError("band sampling steps must be positive")
    ok = []
    for theta in np.arange(0.0, 180.0 + resolution_deg * 0.5, resolution_deg):
        plan = generate_scrape_poses(geometry, tool, theta, theta, 1, azimuth_step_deg,
                                     margin_m=margin_m, pivot_depth_m=pivot_depth_m,
                                     contact_preference=contact_preference)
        if plan.reachable and not plan.unreachable:
            ok.append(float(theta))
    if not ok:
        return None
    groups: list[list[float]] = [[ok[0]]]
    for value in ok[1:]:
        if value - groups[-1][-1] <= resolution_deg * 1.5:
            groups[-1].append(value)
        else:
            groups.append([value])
    best = max(groups, key=lambda group: group[-1] - group[0])
    return best[0], best[-1]


def suggest_bend(geometry: FlaskGeometry, *, target_band_deg=(0.0, 70.0),
                 shaft_radius=0.001, shaft_length=0.25, tip_radius=0.0,
                 margin_m=0.001, pivot_depth_m=None) -> BendSuggestion:
    """Search insertion-safe rigid bends and report ranges plus the best band."""
    lo, hi = map(float, target_band_deg)
    if not 0.0 <= lo <= hi <= 180.0:
        raise FlaskScrapeError("target band must lie between 0 and 180 degrees")
    shaft_radius = _positive(shaft_radius, "shaft radius")
    contact_radius = _positive(tip_radius, "tip radius", True) or shaft_radius
    lateral_max = (2.0 * geometry.neck_inner_radius - shaft_radius
                   - contact_radius - 2.0 * margin_m)
    if lateral_max <= 0.0:
        raise FlaskScrapeError("shaft, tip, and margin do not fit through the neck bore")
    # Larger bulbs can use more of the bore envelope without the end fouling
    # the opposite wall at the bottom. Thirty degrees supports normal contact.
    angle = 30.0
    use = min(0.95, max(0.70, 0.75 + 15.0 * (geometry.R_inner - 0.031)))
    length = use * lateral_max / math.sin(math.radians(angle))
    if length > 0.050:
        length = 0.050
        angle = math.degrees(math.asin(min(1.0, 0.98 * lateral_max / length)))
    tool = BentTool("suggested_bent_tip", shaft_radius, shaft_length, length, angle, tip_radius)
    band = reachable_band(geometry, tool, margin_m=margin_m, pivot_depth_m=pivot_depth_m,
                          resolution_deg=1.0, azimuth_step_deg=90.0)
    angle_max = min(75.0, math.degrees(math.asin(min(1.0, lateral_max / 0.006))))
    length_max = min(0.050, lateral_max / math.sin(math.radians(10.0)))
    meets = band is not None and band[0] <= lo + 0.5 and band[1] >= hi - 0.5
    return BendSuggestion((0.006, length_max), (10.0, angle_max), length, angle,
                          band, (lo, hi), meets)


def _insertion_pose(centre: np.ndarray, tool: ToolModel, reference: np.ndarray) -> PoseSpec:
    if isinstance(tool, StraightTool):
        R = _minimal_twist(np.array([0.0, 0.0, -1.0]), reference)
        tip = centre
    else:
        shaft = np.array([0.0, 0.0, -1.0])
        angle = math.radians(tool.bend_angle_deg)
        x = reference[:, 0].copy()
        x[2] = 0.0
        x = np.array([1.0, 0.0, 0.0]) if np.linalg.norm(x) < 1e-9 else x / np.linalg.norm(x)
        end = math.cos(angle) * shaft + math.sin(angle) * x
        local_x = (math.cos(angle) * end - shaft) / math.sin(angle)
        local_x /= np.linalg.norm(local_x)
        local_y = np.cross(end, local_x)
        R = np.column_stack((local_x, local_y, end))
        shift = 0.5 * (tool.lateral_offset + tool.contact_radius - tool.shaft_radius)
        elbow = centre - shift * x
        tip = elbow + tool.bend_length * end
    return PoseSpec.from_T("flask", tool.tcp, tf.from_rotation(R, tip))


def _retracted_pose(target: ScrapePose, pivot: np.ndarray, tool: ToolModel,
                    geometry: FlaskGeometry) -> PoseSpec:
    T = target.pose.T()
    if isinstance(tool, StraightTool):
        tip = pivot
    else:
        angle = math.radians(tool.bend_angle_deg)
        shaft = T[:3, :3] @ np.array([-math.sin(angle), 0.0, math.cos(angle)])
        inner_join = math.sqrt(max(0.0, geometry.R_inner ** 2
                                   - geometry.neck_inner_radius ** 2))
        q = (inner_join - tool.shaft_radius - pivot[2]) / shaft[2]
        elbow = pivot + max(0.0, q) * shaft
        tip = elbow + tool.bend_length * T[:3, 2]
    return PoseSpec.from_T("flask", tool.tcp, tf.from_rotation(T[:3, :3], tip))


def build_program(scrape: ScrapePlan | FlaskScrape, name: str, robot: str, *,
                  approach_m: float = 0.05, velocity: float = 0.05,
                  contact_velocity: float = 0.02, force_n: Optional[float] = None,
                  motion_mode: str = "position", force_mode: str = "admittance") -> Program:
    """Build named flask waypoints, retracting through the safe pose per ring or stroke: in at the group's first
    pose (the top of a stroke), scrape, back out at its last."""
    plan = scrape.generate() if isinstance(scrape, FlaskScrape) else scrape
    if not isinstance(plan, ScrapePlan):
        raise FlaskScrapeError("build_program needs a ScrapePlan or FlaskScrape")
    if not plan.reachable:
        raise FlaskScrapeError("the scrape has no reachable poses")
    if not math.isfinite(approach_m) or approach_m <= 0.0:
        raise FlaskScrapeError("approach height must be positive")
    if not 0.0 < velocity <= 1.0 or not 0.0 < contact_velocity <= 1.0:
        raise FlaskScrapeError("velocity scalings must be in (0, 1]")
    if force_n is not None and (not math.isfinite(force_n) or force_n <= 0.0):
        raise FlaskScrapeError("scrape force must be positive")
    _, rim, _, rim_z = _flask_values(plan.geometry)
    reference = plan.reachable[0].pose.T()[:3, :3]
    safe_pose = _insertion_pose(plan.pivot, plan.tool, reference)
    approach_pose = _insertion_pose(np.array([rim[0], rim[1], rim_z + approach_m]),
                                    plan.tool, reference)
    waypoints = {
        "approach_neck": ProgramWaypoint("approach_neck", pose=approach_pose),
        "pivot_safe": ProgramWaypoint("pivot_safe", pose=safe_pose),
    }
    for target in plan.reachable:
        waypoints[target.name] = ProgramWaypoint(target.name, pose=target.pose)
    rings = []
    for ring in sorted({target.ring for target in plan.reachable}):
        targets = [target for target in plan.reachable if target.ring == ring]
        group = targets[0].name.rsplit("_", 1)[0]          # ring_01 / stroke_01
        entry = f"{group}_entry_safe"
        exit_name = f"{group}_exit_safe"
        waypoints[entry] = ProgramWaypoint(
            entry, pose=_retracted_pose(targets[0], plan.pivot, plan.tool, plan.geometry))
        waypoints[exit_name] = ProgramWaypoint(
            exit_name, pose=_retracted_pose(targets[-1], plan.pivot, plan.tool, plan.geometry))
        rings.append((targets, entry, exit_name))
    active_mode = force_mode if force_n is not None else motion_mode
    steps = [Step("comment", {"text": "Round-bottom flask scrape"}),
             Step("controller", {"mode": active_mode}),
             Step("move_pose", {"target": "approach_neck", "cartesian": False,
                                "velocity": velocity}),
             Step("move_pose", {"target": "pivot_safe", "cartesian": True,
                                "velocity": contact_velocity})]
    force_active = False
    for ring_index, (targets, entry, exit_name) in enumerate(rings):
        steps.append(Step("move_pose", {"target": entry, "cartesian": True,
                                         "velocity": contact_velocity}))
        for target in targets:
            steps.append(Step("move_pose", {"target": target.name, "cartesian": True,
                                             "velocity": contact_velocity}))
            if force_n is not None and not force_active:
                steps.append(Step("force", {
                    "wrench": [0.0, 0.0, float(force_n), 0.0, 0.0, 0.0], "frame": "tool"}))
                force_active = True
        if force_active:
            steps.append(Step("force_off", {}))
            force_active = False
        steps.append(Step("move_pose", {"target": exit_name, "cartesian": True,
                                         "velocity": contact_velocity}))
        if ring_index + 1 < len(rings):
            steps.append(Step("move_pose", {"target": "pivot_safe", "cartesian": True,
                                             "velocity": contact_velocity}))
    steps.extend([Step("move_pose", {"target": "pivot_safe", "cartesian": True,
                                     "velocity": contact_velocity}),
                  Step("move_pose", {"target": "approach_neck", "cartesian": True,
                                     "velocity": contact_velocity})])
    if force_n is not None and force_mode != motion_mode:
        steps.append(Step("controller", {"mode": motion_mode}))
    return Program(name=name, robot=robot, steps=steps, waypoints=waypoints)
