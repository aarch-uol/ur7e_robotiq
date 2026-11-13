"""sim_ft_reference.py: the sim F/T reference made from a MuJoCo model (skipped without mujoco)."""
import json

import pytest

mujoco = pytest.importorskip("mujoco")

from moveit_gui import ft_match as FM          # noqa: E402
from moveit_gui import sim_ft_reference as SR  # noqa: E402

# A wrist on one fixed body, a tool 0.2 m below the sensor, pointing down. The sensor frame is the world frame
# turned 90 deg about z (sensor x = world y), so the reference must show it.
ARM = """
<mujoco>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 0 1"/>
      <geom type="box" size=".05 .05 .05" mass="1"/>
      <body name="hand" pos="0 0 -0.1" quat="0.7071068 0 0 0.7071068">
        <site name="ft_site"/>
        <geom type="box" size=".02 .02 .02" mass="0.5"/>
        <body name="tool" pos="0 0 -0.2"><geom type="sphere" size=".01" mass="0.1"/><site name="spatula_site"/></body>
      </body>
    </body>
  </worldbody>
  <sensor>
    <force name="ft_force" site="ft_site"/>%s
  </sensor>
  <keyframe><key name="start" qpos="0"/></keyframe>
</mujoco>
"""


def _xml(tmp_path, torque=True):
    p = tmp_path / "arm.xml"
    p.write_text(ARM % ('\n    <torque name="ft_torque" site="ft_site"/>' if torque else ""))
    return p


def test_pushes_and_twist_in_the_sensor_frame(tmp_path):
    ref = SR.make(_xml(tmp_path))
    assert ref["pose"] == "keyframe start" and ref["tip_site"] == "spatula_site" and ref["torque_sensor"] == "ft_torque"
    px, py, pz = (ref["pushes"][a] for a in ("+X", "+Y", "+Z"))
    # MuJoCo reports the force the child (tool side) puts on the parent: the push itself, in the sensor frame
    assert abs(px[1]) == pytest.approx(10, abs=1e-3) and abs(px[0]) < 1e-6      # world x = sensor -y
    assert abs(py[0]) == pytest.approx(10, abs=1e-3) and abs(py[1]) < 1e-6      # world y = sensor x
    assert abs(pz[2]) == pytest.approx(10, abs=1e-3)
    assert abs(px[3]) == pytest.approx(2.0, abs=1e-3)                            # 10 N, 0.2 m below the sensor
    assert abs(ref["twist"][5]) == pytest.approx(1, abs=1e-3) and max(map(abs, ref["twist"][:5])) < 1e-6


def test_without_a_torque_sensor_the_torque_comes_from_the_body(tmp_path):
    a = SR.make(_xml(tmp_path, torque=True))
    (tmp_path / "b").mkdir()
    b = SR.make(_xml(tmp_path / "b", torque=False))
    assert "none" in b["torque_sensor"]
    for k in ("+X", "+Y", "+Z"):
        assert b["pushes"][k] == pytest.approx(a["pushes"][k], abs=1e-3)
    assert b["twist"] == pytest.approx(a["twist"], abs=1e-3)


def test_the_file_loads_as_a_reference_and_bad_input_is_named(tmp_path):
    out = tmp_path / "sim_ft_reference.json"
    assert SR.main([str(_xml(tmp_path)), "--pose", "0.3", "-o", str(out)]) == 0
    ref = FM.load_reference(out)
    assert ref["twist"] is not None and json.loads(out.read_text())["pose"].startswith("joints 0.3")
    with pytest.raises(SR.SimReferenceError, match="no keyframe 'home'.*start"):
        SR.make(_xml(tmp_path), pose="home")
    with pytest.raises(SR.SimReferenceError, match="no site 'tip'"):
        SR.make(_xml(tmp_path), tip="tip")
    with pytest.raises(SR.SimReferenceError, match="cannot load"):
        SR.make(tmp_path / "missing.xml")
