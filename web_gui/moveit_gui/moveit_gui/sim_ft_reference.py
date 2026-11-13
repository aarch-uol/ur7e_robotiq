"""Make the sim F/T reference (sim_ft_reference.json, read by ft_match.py) from the policy's MuJoCo model.

The model is held still at a pose (a keyframe, or joint angles) and loaded at the tool tip, one load at a time:

    +X, +Y, +Z   PUSH_N newtons along the model's world +X, +Y, +Z
    twist        TWIST_NM newton metres clockwise, looking down from above (a torque along world -Z)

Each entry is the change of the model's F/T sensor [fx, fy, fz, tx, ty, tz] from the unloaded reading, in the
sensor's frame, as the policy saw it in training (MuJoCo reports the reaction, so signs may look backwards: that
is what the real sensor is matched to). Static: qvel = qacc = 0 and mj_inverse, so no controller is needed.
A model without a torque sensor gets the torque from the sensor body's internal force (what MuJoCo's torque
sensor reports).

Needs the mujoco Python package, so it runs where the policy was trained, not on the robot computer:

    python3 -m moveit_gui.sim_ft_reference <model.xml> [--pose <keyframe>] [--tip <site>] -o sim_ft_reference.json

Put the file next to the policy's robot_config, or name it in the manifest as sim_ft_reference.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import List, Optional, Sequence, Union

import numpy as np

PUSH_N = 10.0
TWIST_NM = 1.0
TIP_SITES = ("spatula_site", "tcp", "tool_tip", "attachment_site")
FORCE_SENSORS = ("ft_force", "ft_sensor_force")
TORQUE_SENSORS = ("ft_torque", "ft_sensor_torque")


class SimReferenceError(ValueError):
    pass


def _find(model, kind, names: Sequence[str], what: str, given: Optional[str]):
    import mujoco
    obj = {"site": mujoco.mjtObj.mjOBJ_SITE, "sensor": mujoco.mjtObj.mjOBJ_SENSOR}[kind]
    for n in ([given] if given else names):
        i = mujoco.mj_name2id(model, obj, n)
        if i >= 0:
            return i, n
    if given:
        raise SimReferenceError(f"the model has no {kind} '{given}' ({what})")
    return -1, None


def _pose(model, data, pose: Union[None, str, Sequence[float]]) -> str:
    import mujoco
    if pose is None:
        if model.nkey:
            mujoco.mj_resetDataKeyframe(model, data, 0)
            return f"keyframe {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, 0)}"
        mujoco.mj_resetData(model, data)
        return "the model's default pose"
    if isinstance(pose, str):
        k = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, pose)
        if k < 0:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i) for i in range(model.nkey)]
            raise SimReferenceError(f"the model has no keyframe '{pose}' (it has: {', '.join(names) or 'none'})")
        mujoco.mj_resetDataKeyframe(model, data, k)
        return f"keyframe {pose}"
    q = [float(x) for x in pose]
    if not q or len(q) > model.nq:
        raise SimReferenceError(f"a pose needs 1 to {model.nq} joint values, not {len(q)}")
    mujoco.mj_resetData(model, data)
    data.qpos[:len(q)] = q
    return "joints " + " ".join(f"{x:.4f}" for x in q)


def make(xml: Union[str, Path], pose: Union[None, str, Sequence[float]] = None, tip: Optional[str] = None,
         force_sensor: Optional[str] = None, torque_sensor: Optional[str] = None,
         push_n: float = PUSH_N, twist_nm: float = TWIST_NM) -> dict:
    """The reference dict: {"pushes": {"+X", "+Y", "+Z": [6]}, "twist": [6], "no_contact": {"0": [6]}, ...}."""
    try:
        import mujoco
    except ImportError:
        raise SimReferenceError("the mujoco Python package is not installed here: run this where the policy was "
                                "trained (pip install mujoco)") from None
    xml = Path(xml).expanduser()
    try:
        model = mujoco.MjModel.from_xml_path(str(xml))
    except Exception as e:                                          # noqa: BLE001 (mujoco raises ValueError)
        raise SimReferenceError(f"cannot load {xml}: {e}") from None
    data = mujoco.MjData(model)
    where = _pose(model, data, pose)
    site, tip = _find(model, "site", TIP_SITES, "the tool tip", tip)
    if site < 0:
        raise SimReferenceError(f"no tool tip site found (tried {', '.join(TIP_SITES)}): name one with --tip")
    fs, fname = _find(model, "sensor", FORCE_SENSORS, "the force sensor", force_sensor)
    if fs < 0:
        raise SimReferenceError(f"no force sensor found (tried {', '.join(FORCE_SENSORS)}): name one")
    if model.sensor_type[fs] != mujoco.mjtSensor.mjSENS_FORCE:
        raise SimReferenceError(f"sensor '{fname}' is not a force sensor")
    ts, tname = _find(model, "sensor", TORQUE_SENSORS, "the torque sensor", torque_sensor)
    ft_site = int(model.sensor_objid[fs])
    ft_body = int(model.site_bodyid[ft_site])
    body = int(model.site_bodyid[site])
    qpos = data.qpos.copy()

    def read(force=np.zeros(3), torque=np.zeros(3)) -> np.ndarray:
        data.qpos[:] = qpos
        data.qvel[:] = 0
        data.qacc[:] = 0
        data.xfrc_applied[:] = 0
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        lever = data.site_xpos[site] - data.xipos[body]           # xfrc_applied acts at the body's centre of mass
        data.xfrc_applied[body, :3] = force
        data.xfrc_applied[body, 3:] = torque + np.cross(lever, force)
        mujoco.mj_inverse(model, data)
        f = data.sensordata[model.sensor_adr[fs]:model.sensor_adr[fs] + 3].copy()
        if ts >= 0:
            t = data.sensordata[model.sensor_adr[ts]:model.sensor_adr[ts] + 3].copy()
        else:                                                     # what a torque sensor at the site reports
            c = data.cfrc_int[ft_body]                             # [torque, force] about the root's subtree com
            com = data.subtree_com[model.body_rootid[ft_body]]
            tw = c[:3] - np.cross(data.site_xpos[ft_site] - com, c[3:])
            t = data.site_xmat[ft_site].reshape(3, 3).T @ tw
        return np.concatenate([f, t])

    base = read()
    pushes = {ax: read(force=float(push_n) * np.eye(3)[i]) - base for i, ax in enumerate(("+X", "+Y", "+Z"))}
    twist = read(torque=np.array([0.0, 0.0, -float(twist_nm)])) - base
    mujoco.mj_kinematics(model, data)
    r = lambda v: [round(float(x), 4) + 0.0 for x in v]          # noqa: E731
    return {
        "pushes": {ax: r(v) for ax, v in pushes.items()},
        "twist": r(twist),
        "no_contact": {"0": r(base)},
        "model": str(xml), "pose": where, "tip_site": tip, "force_sensor": fname,
        "torque_sensor": tname or f"(none: from the internal force of body {model.body(ft_body).name})",
        "push_n": float(push_n), "twist_nm": float(twist_nm),
        "tip_position": r(data.site_xpos[site]),
        "note": "pushes: sensor change for push_n newtons at the tip along world +X/+Y/+Z; twist: twist_nm "
                "newton metres clockwise seen from above",
        "made": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Make sim_ft_reference.json from a MuJoCo model (see the module doc).")
    ap.add_argument("xml", help="the policy's MuJoCo model (e.g. mujoco_menagerie/franka_emika_panda/panda.xml)")
    ap.add_argument("--pose", help="keyframe name, or comma-separated joint angles (default: the first keyframe)")
    ap.add_argument("--tip", help=f"tool tip site (default: the first of {', '.join(TIP_SITES)})")
    ap.add_argument("--force-sensor")
    ap.add_argument("--torque-sensor")
    ap.add_argument("--push-n", type=float, default=PUSH_N)
    ap.add_argument("--twist-nm", type=float, default=TWIST_NM)
    ap.add_argument("-o", "--out", help="write here (default: print)")
    a = ap.parse_args(argv)
    pose = a.pose
    if pose:
        try:
            pose = [float(x) for x in pose.split(",")]       # joint angles; else a keyframe name
        except ValueError:
            pass
    try:
        ref = make(a.xml, pose, a.tip, a.force_sensor, a.torque_sensor, a.push_n, a.twist_nm)
    except SimReferenceError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    text = json.dumps(ref, indent=1) + "\n"
    if a.out:
        Path(a.out).write_text(text)
        print(f"wrote {a.out} ({ref['pose']}, tip {ref['tip_site']})")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
