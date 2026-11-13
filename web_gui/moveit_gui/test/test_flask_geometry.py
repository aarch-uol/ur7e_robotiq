import math

import numpy as np
import pytest

from moveit_gui.flask_geometry import (
    FlaskGeometry,
    FlaskGeometryError,
    T_base_flask,
    closest_standard_size,
    fit_flask,
)


def _basis(axis):
    z = np.asarray(axis, float)
    z /= np.linalg.norm(z)
    x = np.array([1.0, 0.0, 0.0])
    x -= z * (x @ z)
    x /= np.linalg.norm(x)
    return x, np.cross(z, x), z


def _synthetic(od=0.085, neck_outer=0.027, neck_length=0.060, tip=0.002,
               centre=(0.42, -0.08, 0.22), tilt_deg=2.0, seed=4):
    rng = np.random.default_rng(seed)
    axis = np.array([math.sin(math.radians(tilt_deg)), 0.0, math.cos(math.radians(tilt_deg))])
    x, y, z = _basis(axis)
    centre = np.asarray(centre, float)
    radius = od / 2.0
    join = math.sqrt(radius * radius - neck_outer * neck_outer)
    rim = centre + z * (join + neck_length)
    angles = np.linspace(0.0, 2.0 * math.pi, 16, endpoint=False)
    rim_points = [rim + (neck_outer + tip) * (x * math.cos(a) + y * math.sin(a)) for a in angles]
    dirs = rng.normal(size=(80, 3))
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    bulb_points = centre + dirs * (radius + tip)
    side = []
    for a in angles[::2]:
        side.append(rim - z * neck_length * 0.5 + (neck_outer + tip) * (x * math.cos(a) + y * math.sin(a)))
    noise = lambda p: np.asarray(p) + rng.normal(scale=0.0002, size=np.asarray(p).shape)
    return noise(rim_points), noise(bulb_points), noise(side), centre, radius, axis, rim


@pytest.mark.parametrize("od,neck", [(0.051, 0.020), (0.085, 0.027), (0.131, 0.037)])
def test_noisy_tilted_flasks_fit_with_tip_compensation(od, neck):
    rim, bulb, side, centre, radius, axis, _ = _synthetic(od=od, neck_outer=neck)
    g = fit_flask(rim, bulb, side, r_tip=0.002, wall_thickness=0.0015)
    assert g.sphere_center == pytest.approx(centre, abs=0.0005)
    assert g.R_outer == pytest.approx(radius, abs=0.0005)
    assert g.R_inner == pytest.approx(radius - 0.0015, abs=0.0005)
    assert g.neck_axis == pytest.approx(axis, abs=0.01)
    assert g.neck_inner_radius == pytest.approx(neck - 0.0015, abs=0.0005)
    assert g.neck_length == pytest.approx(0.060, abs=0.0005)
    assert g.rim_rms < 0.0005 and g.sphere_rms < 0.0005


def test_fixture_is_at_sphere_with_z_up_neck_and_x_near_base_x():
    rim, bulb, side, centre, _, axis, _ = _synthetic(tilt_deg=6.0)
    g = fit_flask(rim, bulb, side, r_tip=0.002)
    T = T_base_flask(g)
    assert T[:3, 3] == pytest.approx(centre, abs=0.0005)
    assert T[:3, 2] == pytest.approx(axis, abs=0.01)
    assert T[0, 0] > 0.99
    assert "tilted" in " ".join(g.warnings)


def test_override_serialization_and_closest_size():
    rim, bulb, side, _, _, _, _ = _synthetic()
    g = fit_flask(rim, bulb, side, r_tip=0.002, neck_inner_diameter=0.020)
    assert g.neck_inner_radius == pytest.approx(0.010)
    again = FlaskGeometry.from_dict(g.to_dict())
    assert again.to_dict() == g.to_dict()
    assert closest_standard_size(0.086, 0.024) == "250 ml"


def test_too_few_degenerate_and_nonsense_inputs_are_rejected():
    rim, bulb, _, _, _, _, _ = _synthetic()
    with pytest.raises(FlaskGeometryError, match="at least 3"):
        fit_flask(rim[:2], bulb)
    with pytest.raises(FlaskGeometryError, match="at least 4"):
        fit_flask(rim, bulb[:3])
    with pytest.raises(FlaskGeometryError, match="collinear"):
        fit_flask([[0, 0, 0], [0.01, 0, 0], [0.02, 0, 0]], bulb)
    with pytest.raises(FlaskGeometryError, match="coplanar"):
        fit_flask(rim, [[0, 0, 0], [0.01, 0, 0], [0, 0.01, 0], [0.01, 0.01, 0]])
    with pytest.raises(FlaskGeometryError, match="wall thickness"):
        fit_flask(rim, bulb, r_tip=0.002, wall_thickness=0.1)


def test_off_axis_and_bad_residuals_warn():
    rim, bulb, side, _, _, _, _ = _synthetic()
    rim[:, 0] += 0.003
    bulb[0] += [0.004, 0.004, 0.004]
    bad_side = np.vstack((side, side[0] + [0.005, 0, 0]))
    g = fit_flask(rim, bulb, bad_side, r_tip=0.002)
    text = " ".join(g.warnings)
    assert "off the neck axis" in text
    assert "RMS residual" in text
