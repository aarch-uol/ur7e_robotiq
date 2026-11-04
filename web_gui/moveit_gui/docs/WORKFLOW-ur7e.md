# Running a scraping experiment on the UR7e with the GUI

This is the GUI version of the FR3 workflow in `~/franka_ros2/sample_scraping`:
`estimation.py` (touch the rim, fit the vial), then `policy_scraping.py` (go above the vial, and at each angle
around the rim run the policy for a few seconds). Here the robot-specific parts come from `~/ur7e_robotiq`,
and the policy still runs in the existing runner (`mujoco_multiagent_rl.policy_runner`). The GUI never runs
inference.

| FR3 script | GUI |
|---|---|
| `estimation.py`: low impedance, Enter ×3 on the rim | SETUP › Vial Geometry: **Freedrive**, then **Capture** ×3 |
| `circle_from_points`, bottom = lowest point | the same maths (`geometry_calibration.estimate_vial`), saved as fixture `vial` |
| `target_xyz`, `vial_radius`, `vial_height` parameters | the `vial` fixture and one taught contact pose |
| `_create_scrape_quat_xyzw`: RPY constants for `fr3_hand_tcp` | none: the taught pose is turned about the vial axis |
| `ANGLE_STEP_DEGREES = 10`, forward + inverted pass | **Angle step**, **Angles** and **Then sweep back** |
| `_run_policy_at_angle`: policy for `policy_duration` | a `policy` step (it runs the runner for `duration` seconds) |
| `ft_sensor_calibration` | SETUP › F/T + Payload, then **Apply to admittance** |

## 0. Start

```bash
~/ur7e_gui.sh                # robot stack + web GUI (http://<tailscale ip>:8180/); --desktop for this window
~/ur7e_gui.sh -- servo:=true # also start MoveIt Servo, for hold-to-jog and servo_admittance policies
```
The script runs everything in the `ur7e_robotiq` container on ROS domain 42 (another lab PC runs a Gazebo UR
on the default domain 0). Other terminals that talk to the robot need `export ROS_DOMAIN_ID=42`.
On the pendant, External Control must be running. The header should show `RUNNING · NORMAL` and
`All connected`. If it shows `PROGRAM STOPPED`, the External Control program stopped (a protective stop, the
pendant, a dropped connection): the UR driver then deactivates the arm controller and the GUI refuses motion.
Press Play on the pendant; the driver re-activates the controller. A program stop can also stop the F/T bridge;
the next switch to Admittance brings it back.

## 1. Tool (once per tool): SETUP › TCPs
Select or create the TCP of the tool that touches the vial (for example `spatula_tip`), then click **Set active**.
Every pose you record is stored *with* its TCP, so record with the TCP you will scrape with.

## 2. Payload, mass and CoG (once per tool): SETUP › F/T + Payload
1. Switch to **Position** mode, and check that the seven poses are clear of the cell.
2. Click **▶ Collect all poses**. The arm visits each pose, settles, and averages the ATI readings.
3. Read the result: it should say PASS. Mass and CoG are fitted in `ft_fts_link` by `calibration_math`.
4. Click **Save result**, which writes its own JSON file under `~/.config/moveit_gui/ft_payload/`.
5. Click **Apply to admittance**. This sets `gravity_compensation.CoG.pos` and `CoG.force`, the weight
   mass × 9.80665 N, on the running `/admittance_controller`, then reads the values back.
   - The GUI refuses while Admittance is active, because a change in compensation would move the arm.
   - The values last until the controller restarts. To keep them, use **Copy YAML** and paste the snippet into
     `gravity_compensation:` in `ur_controllers_gui.yaml` (and `ur_controllers_admittance.yaml`).
   - The controller has no offset term. Keep `ft_hardware_biasing:=false` (the default), and do not re-zero
     the ATI after calibrating. The software F/T bias (web SETUP › F/T + Payload) must stay as it was while
     the poses were collected; the GUI refuses to apply a fit after it changed.

## 3. Vial, every time it moves: SETUP › Vial Geometry
1. Click **Freedrive** and drag the tool tip to the rim.
2. Click **Capture** for Rim 1, then do the same for Rim 2 and Rim 3. Spread the points around the rim; more
   than 90° apart is best.
3. Optionally touch the bottom, then click **Capture** next to Bottom. Without it, Z is the lowest rim point.
4. Check that the radius is plausible and the rim height spread is small. A large spread means the rim is not level or a touch was off.
5. Click **End freedrive**, name the fixture `vial`, and click **Save as fixture**.

The fixture is base-aligned, with its origin at the bottom centre, so its Z axis is the vial axis.
This is the same level-rim, vertical-vial assumption the FR3 script made. It is not a 3D fit.

## Keep-out zones (table, vial holder): SETUP › Keep-out zones
Boxes MoveIt must plan around. They change what MoveIt plans; they never move the robot. Edit them only while
the robot is idle.
1. **Table** (once): touch the table top with the TCP (Freedrive), choose frame `base_link`, enter a size that
   covers the table (e.g. 1200 × 800 × 20 mm), click **Under TCP** (centre under the TCP, top face 5 mm below
   it), then **Save box**. Keep the top a few mm below the robot base: a box that contains the TCP, the flange
   or the base origin is refused, because MoveIt would start in collision and every plan would fail.
2. **Vial holder**: after SETUP › Vial Geometry, choose frame `vial` (origin at the vial bottom centre) and enter
   the holder size and centre in that frame, e.g. 120 × 90 × 60 mm at 0, 0, 30 mm. When the vial is re-taught,
   the box moves with it automatically.
3. Check the status line: **✓ MoveIt: n box(es) in MoveIt's planning scene** means move_group was read back and
   holds them. ⚠ means it does not (MoveIt down, a box skipped because the robot is inside it, a fixture
   missing): read the message, fix it, **Re-apply**.
4. Verify: OPERATE 3D view (Zones on) shows the boxes in red where you expect them. In RViz (MotionPlanning ›
   Scene Objects) they appear as `gui_keepout/<name>`. Then **Plan** a Cartesian goal on the far side of a box:
   the blue ghost must go around it; a goal inside a box must fail to plan. Only then Execute.

Untick a box to take it out of MoveIt's scene without deleting it. Other planning-scene objects (not named
`gui_keepout/...`) are never touched.

If the robot itself ends up inside an enabled box (freedrive, a re-taught fixture), an amber banner says so and
failed moves name the box: MoveIt refuses every plan from inside one. While idle the GUI also re-reads MoveIt's
scene every 8 s and applies the boxes again if move_group lost them (for example after a fast restart).

## Cameras: SETUP › Cameras
The page lists the cameras in the profile (`cameras:` in `profiles/ur7e_lab.yaml`): RealSense colour and depth, ZED
left and depth, and the scraping vision node's annotated and midpoints images. A green dot means the topic has a
publisher; a grey dot means nothing publishes it (the driver is not running, or the topic name is wrong). Click a
camera to preview it, with resolution, encoding, rate and `frame_id`. Pick or type any other image topic
(including `.../compressed`) under **Any image topic**. Only the previewed topic is subscribed, and only while the
page is on screen. The ZED topic names are the zed-ros2-wrapper defaults and have not been checked against your
install. If they differ when the ZED arrives, change them in the profile.

## 4. The policy: a manifest in `~/moveit_gui_policies/`
Copy `policies/_template.yaml.example` and set `model`, `normalization`, `training_config` (the PPO
run folder) and `robot_config`, which is the UR file `ur7e_policy_deployment.yaml`, never the FR3 `robot_config.yaml`. Then choose the backend:

| backend | required_mode | how the runner commands the arm |
|---|---|---|
| `servo_admittance` | `admittance` | dx/dy/dz → MoveIt Servo twists, force → `/admittance_controller/wrench_reference` |
| `impedance` | `impedance` (direct torque) | reference pose → `/cartesian_impedance_controller/reference_pose`, wrench → `set_cartesian_wrench` |

With `impedance`, note that the UR controller has `wrench.apply_wrench: false`. The policy's force
actions are therefore ignored, and only its pose deltas act. To use them, set `apply_wrench: true` in
`ur_controllers_cartesian_impedance.yaml` after a low-force test.

The runner takes its start pose and force baseline (50 samples) where the arm is when the step starts, so the
program must put the tool on the rim first. AUTOMATION shows whether a policy is ready: files, mode, TCP,
Servo, and the deployment commissioning.

## 5. The program: PROGRAMS › File › New scraping program…
1. Select **Vial frame** `vial`.
2. Click **Freedrive**, put the tool on the rim exactly as scraping should start (position *and* tilt), then click **Record contact pose**.
   Then click **End freedrive**.
3. Select the **Policy**, and set **Per angle** (seconds), **Angle step**, **Angles**, and optionally **Then sweep back**.
   **Approach height** is how far above the contact pose each move starts.
4. Click **Create program**. Each angle becomes the steps below, with named waypoints `above_###` and `touch_###`:

   ```
   move_pose above_030            (first one planned, then linear)
   move_pose touch_030  linear 3%
   [controller impedance]         only if the policy's mode cannot run MoveIt
   policy  <name>  8 s
   [controller position]
   move_pose above_030  linear
   ```
   For a `servo_admittance` policy everything runs in Admittance, with no switching.
5. Check the program: select a few `touch_` steps, then run **Run step** on the first two at low speed. Click **File › Check program**, then **Save**.
6. Click **▶ RUN**. **Pause** and **■ STOP** act between steps, and the header **■ STOP** also stops a running policy.

## Teaching by hand (any program)
In PROGRAMS, click **Freedrive**, drag the arm, then click **Record pose**. That adds a MovePose step to the current
TCP pose in the frame shown on the right. Choose `vial` there to make the pose follow the vial when it is
re-calibrated. **+ Add step › MoveJ — current joints** records joints instead. Click **End freedrive**, and
the arm returns to the mode it was in before.

You can write steps by hand too (see `programs/example_scrape.yaml`). Every pose is
`{frame, tcp, position [m], quat_xyzw}`.

## Jogging: OPERATE › Jog
- **Step** (0.5 / 1 / 5 / 10 mm or °): each press is one short MoveIt move. Cartesian steps are straight lines;
  rotations turn about the active TCP. Works in Position and Admittance.
- **Continuous (Servo)**: six spring-centred −100…100 sliders map linearly to MoveIt Servo velocity, capped by
  the OPERATE Velocity %. The robot PC streams Servo input commands at 100 Hz. It works at
  50 mm/s or 15 °/s. It needs `servo:=true` and works in **Position and Admittance**. In Position, the GUI
  transactionally switches `scaled_joint_trajectory_controller` → `joint_trajectory_controller`, and the
  dedicated Position Servo node (`/position_servo_node`) streams trajectories to it, the same way `/servo_node`
  streams to `admittance_trajectory_controller` in Admittance. On release/deadman/STOP it restores the scaled
  trajectory controller. (Not `forward_position_controller`: MoveIt 2.12 Servo starts every position-array step
  from the measured state, and the real UR's tracking lag then leaves the arm almost still.) Release returns the slider to zero; the core stops 0.5 s after the last
  screen renewal, while Servo itself also has a 0.1 s incoming-command timeout.
- **Cartesian · base** jogs along the base axes, **Cartesian · tool** along the active TCP's axes, and **Joint**
  maps the six sliders to the six UR joints.
- In Admittance the arm sits slightly off its commanded position (residual force ÷ stiffness: 5 N at 250 N/m is
  2 cm). `/servo_node` therefore reads `/servo_reference_state/joint_states`, the measured joint states with the
  arm joints replaced by `admittance_trajectory_controller`'s reference. Fed the measured state, it jumped by that
  offset at every jog start (and at every `servo_admittance` policy start).
- Every jog ends with a `Servo jog summary` line in the GUI log: how many commands Servo sent to its controller,
  Servo's status, and how far the arm moved. "0 commands" means Servo did not run; commands but no motion
  means the controller or driver did not act on them.

## 3D view and planning: OPERATE
- The 3D view draws the robot from `/robot_description` with its meshes, posed from the joint states, like RViz.
  Red arrow: measured F/T (at the sensor). Green arrow: the force being applied (Force tab). Drag to orbit,
  right-drag to pan, scroll to zoom.
- **Plan** (Joint or Cartesian tab) plans without moving; the blue ghost plays the plan in a loop. **Execute**
  runs exactly that plan, once, if the arm is still where the plan starts. **MoveJ / MovePose** plan and execute
  in one go.
- **Goal** shows a gizmo for the TCP (Move / Rotate). Dragging it fills the Cartesian tab (base frame); typing
  there moves it. Then Plan, check the ghost, Execute.

## Applying a force: OPERATE › Force (Admittance or Impedance)
- Enter Fx Fy Fz [N] and Tx Ty Tz [Nm] in the tool (active TCP) or base axes: the wrench the tool exerts on the
  environment. **Apply** ramps it in (10 N/s); **Change** retargets while applying; **Ramp down** ramps it out;
  **Zero now** and STOP zero it at once. Limits: 20 N, 2 Nm (profile `force`).
- In free air the arm moves along the force until the controller's stiffness balances it (admittance x:
  250 N/m, so 5 N is 2 cm); against a surface it presses with it. **Test the sign first**: 2 N in free air, and
  check that the arm moves the way you asked.
- Admittance only responds along its `selected_axes` (now tool X only): the tab says so. Impedance takes all six.
- Neither controller times out a wrench, so the robot PC streams it while this page renews it. Losing control,
  closing or hiding the tab, or the network zeroes it within 0.5 s. So does a mode change.

## Before the first real run
- `ur7e_policy_deployment.commissioning.yaml` says `ready: false`, so real policy runs are refused until the listed values
  are measured and you set `ready: true`.
- `spatula_tip` exists only as a GUI TCP. The runner's `servo_admittance` backend looks it up in TF, so publish it
  (URDF or a static transform) before running that backend.
- Impedance (direct torque) is enabled in `profiles/ur7e_lab.yaml`. Every switch still checks robot state, interface
  ownership and the reference pose, and rolls back on failure. It needs PolyScope ≥ 5.25.1 and the
  patched `cartesian_impedance_controller`.
