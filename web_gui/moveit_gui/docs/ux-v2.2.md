# MoveIt GUI v2.2: UX, information architecture and control-stack model

This pass restructures the interface. It does not add new robot features. The screenshots in
[`docs/ux/v2.2/`](ux/v2.2/) come from the fake robot (`moveit_gui --fake`), so every number in them is invented.

**Not inspected:** `~/ur7e_robotiq` and `~/franka_ros2` could not be read in this session (no link to the
lab computer). Every UR, Robotiq, policy-runner, vial and F/T-calibration detail below is taken from
the task description and is marked as unverified. Nothing here was run on hardware.

## 1. Audit of v2.1

| Area | v2.1 | Problem |
|---|---|---|
| Navigation | 8 equal horizontal tabs: Robot, Control mode, Manual, Gripper, Program, Policies, Frames & Calibration, Diagnostics; calibration had 3 more sub-tabs | Mirrors software subsystems; daily use needs 4 tabs (Robot, Control, Manual, Gripper) |
| Control mode | `POSITION \| ADMITTANCE \| IMPEDANCE` as three equal buttons | Suggests equivalent presets; hides that Impedance claims EFFORT (direct torque) and takes no MoveIt trajectories |
| Mode model | `activate`, `supports_moveit`, `experimental` only | No hardware interface, command-source type, MoveIt controller or domain; nothing verified interface ownership |
| Gripper | Full tab; shown as if part of the arm | Runs on its own controller manager; a "reactivate" button pointed at an unverified service |
| Programs | Table + 10 equally prominent side buttons; YAML dialog the only editor | Clutter; editing a pose meant editing YAML |
| Policies | Table of manifests + "Reload" | No backend / hardware semantics, no execution feedback, nothing about the runner's F/T baseline |
| Header | Single row, ~10 items | STOP button was clipped at ~1250 px; the IP was as prominent as the safety state |
| Log | Permanent ~160 px panel under every tab | ~20% of the height on 768-px screens |
| Sizing | `resize(1180, 820)`; no scroll areas anywhere | Taller than 768 px: the calibration and program pages could not fit on 1366×768 |
| Bugs found | `RosBridge.snapshot()` re-took its own `Lock` → **GUI freeze on the real robot**; `wait_until_executed() is None` counted as success | Both fixed in this pass (strictly necessary), with regression tests |

## 2. Navigation

Left rail with 5 entries (Ctrl+1…5): **OPERATE · PROGRAMS · AUTOMATION · SETUP · DIAGNOSTICS**.

* **Header (always visible):** robot, robot state, safety state, speed, TCP selector, arm mode · hardware
  interface, STOP GUI MOTION.
* **Second header row:** compact connectivity lights (ROS, UR, MoveIt, ATI, Robotiq, External Control),
  then the IP and profile in grey.
* **Log drawer:** collapsed `Logs ▸` with an unread count. It turns red on refusals or failures and shows
  the latest line. The full log lives under DIAGNOSTICS. There is no status bar.

## 3. Control-stack state model

Each mode in the profile now carries its semantics. The loader rejects inconsistent combinations, such as
a direct-torque mode that claims to accept MoveIt.

| | Position | Admittance | Impedance |
|---|---|---|---|
| `command_interface` | position | position | **effort** |
| `command_source` | trajectory | trajectory | cartesian_reference |
| `moveit_controller` | scaled_joint_trajectory_controller | admittance_trajectory_controller | — |
| `chain` | — | admittance_trajectory_controller → admittance_controller | — |
| `requires_ft` | no | yes | no |
| `direct_torque` / domain | no / position | no / position | **yes / direct_torque** |
| status | enabled | enabled | `allow_activation: false` |

The `domain` is derived from `direct_torque`. Position ↔ Admittance stays in one domain. Anything ↔
Impedance **crosses the direct-torque boundary**.

`transition_steps()` gives the ordered checklist the dialog shows. Crossing the boundary adds these steps:
release the old interfaces, activate STRICT, verify EFFORT ownership, initialise the reference at the
current pose.

After every switch, `ControlModeManager` now checks **claimed interfaces**: each arm joint must be claimed
with exactly the mode's interface. For a direct-torque target, claims that can't be read count as a
failure.

Unchanged: preflight, STRICT switching, verified standstill, `allow_activation: false` for Impedance.

The **gripper** is modelled outside this: `gripper.controller_manager: /robotiq_controller_manager`
(unverified), listed separately in the bridge snapshot and never touched by mode switching.

## 4. OPERATE ([1366](ux/v2.2/1366_operate_admittance.png), [1920](ux/v2.2/1920_operate.png), [1.5× DPI](ux/v2.2/hidpi150_operate.png))

1. **Robot strip:** Power On, Brake Release, PolyScope ▶ ⏸ ■. Everything else is under `More ▾`: unlock
   protective stop, restart safety, popups, resend External Control, load program, speed slider, DO0–7,
   power off, zero the UR's built-in F/T.
2. **Manual motion:** Joint / Cartesian / Jog tabs. Jog is a placeholder: Servo is not wired. The Cartesian
   target is *the active TCP in a chosen frame*.
3. **Current state:** TCP pose in a chosen frame, plus joints.
4. **Arm control:** mode, hardware interface, accepted command source, command path, ATI status, and what
   is commanding the arm now. `Change…` opens the control-stack dialog
   ([screenshot](ux/v2.2/1366_stack_dialog_impedance.png)): the position domain and the direct-torque domain
   are separated by a labelled **DIRECT TORQUE BOUNDARY**, and the dialog shows the transition checklist
   and live preflight. The impedance apply button is amber, not red.
5. **Gripper card:** Ready / Fault / Unavailable, % closed, Open / Close, target + effort.
   `Advanced ▸` shows the raw position, velocity and effort, the controller manager, the controller
   states and the action type. The reactivation button only appears when a verified service is configured.
6. **ATI F/T strip:** read-only. The ATI box is never biased; a software bias is set in web SETUP › F/T + payload (see WEB.md).

## 5. PROGRAMS ([1366](ux/v2.2/1366_programs.png))

* **Table** of steps. Mode steps show `Admittance · POSITION`, pose steps show `waypoint: frame / tcp`.
* **Inspector** with a form per step type. MovePose has frame, TCP, XYZ in mm, RPY in degrees, linear,
  velocity, and *Record current pose* using that step's frame and TCP. MoveJ has a waypoint selector and
  *Record current joints*. The other types each get a small form, and YAML remains a fallback.
* **Secondary actions** (↑ ↓, Duplicate, YAML, Delete) moved to the inspector footer, the right-click menu
  and shortcuts (Ctrl+D, Alt+↑/↓, Del, Ctrl+Enter, Ctrl+S).
* **Run bar:** ▶ Run (primary), Pause, Stop (danger), Step, from selected, loop.
* **"+ Add step"** includes *MoveJ — current joints* and *MovePose — current pose*. The record frame
  is chosen next to it, and the active TCP is stated.

## 6. AUTOMATION ([preparing](ux/v2.2/1366_automation_preparing.png), [ready](ux/v2.2/1366_automation_ready.png))

* **Policy detail:** algorithm, rate, backend with its command path (`impedance` or `servo_admittance`),
  required control mode, the resulting **hardware interface**, duration, model / normalizer / training
  config.
* **Requirements:** ✓ / ✗ / ? per item.
  * Shown as `?`, not checked: observation shape (the runner's job) and whether MoveIt Servo is running.
  * The required mode must already be active. The GUI never switches modes for a policy; the page
    says so.
* **Recording:** the checkboxes are shown but disabled. They will become part of the RunPolicy goal.
* **Execution:** Prepare → the runner's F/T startup baseline progress (`33 / 50`), the wrench, and the
  pose source, observation and normalizer checks. When it finishes: `✓ F/T baseline locked`. Then
  ▶ Run and ■ Stop policy.
* **Interface boundary:** `policy_action.py` defines RunPolicyGoal / Feedback / Result, mirroring the
  intended `RunPolicy.action`.
  * `MockPolicyRunner` drives the demo.
  * `SubprocessPolicyRunner` wraps the existing launcher and states that it gives no feedback.
  * The GUI contains no inference code.

## 7. SETUP

A sub-navigation list with one section at a time, each in its own scroll area.

* **TCPs** ([screenshot](ux/v2.2/1366_setup_tcps.png)): the library card, then the pivot calibration card.
  The math is unchanged.
* **Fixtures** ([screenshot](ux/v2.2/1366_setup_fixtures.png)): a wizard. Method: *Teach 3 points* or
  *Manual transform*. Step 1/2/3 with ✓ marks, *Capture current TCP*, a result preview, then Save.
* **Vial geometry** ([screenshot](ux/v2.2/1366_setup_vial.png)): **placeholder**. It must reuse the
  `vial_estimation.py` geometry, which I could not read. The planned limits are stated: XY circle,
  base-aligned, bottom from sampled Z.
* **Vision baselines** ([screenshot](ux/v2.2/1366_setup_vision.png)): the ground-truth source is either a
  *probe touch (position only)*, a *known fixture/frame*, or *manual entry*. "Correct: Position" is always
  on. "Orientation" is disabled for probe touches. The math is now position-only by default, so a probe
  orientation can never leak into the correction (tested).
* **F/T + payload:** **placeholder**. It must wrap `ft_sensor_calibration/calibration_math.py` with UR
  poses, and must never write controller YAML.
* **Robot profile** ([screenshot](ux/v2.2/1366_setup_profile.png)): read-only facts, the mode table, and a
  deliberate *Reconnect…* for changing robot. This moved here from the header.

## 8. DIAGNOSTICS ([1366](ux/v2.2/1366_diagnostics.png), [1920](ux/v2.2/1920_diagnostics.png))

* **Arm control:** mode and hardware command; the chain `MoveIt FollowJointTrajectory ↓
  admittance_trajectory_controller [active] ↓ admittance_controller [active] ↓ <joint>/position …`;
  a ✓ / ✗ check of the claimed interfaces; the direct-torque warning; and every arm controller with its
  state and claims.
* **Gripper:** its own controller manager, controller states, action, interface, joint, and whether the
  action server is up.
* **State:** joint-state age, velocity availability, stationarity, F/T source, command source, the policy
  runner type with its feedback ability, active TCP, planning link, and the last refusal.
* **Log:** the full log.

## 9. Mapping: v2.1 → v2.2

| v2.1 | v2.2 |
|---|---|
| Robot tab: Power, Brake, Play/Pause/Stop | OPERATE robot strip |
| Robot tab: unlock, restart safety, popups, resend, load, speed slider, DO0–7, zero UR F/T | OPERATE `More ▾` |
| Robot tab: robot / safety / program / speed state | Header |
| Control tab: three mode buttons | OPERATE Arm-control card + control-stack dialog |
| Control tab: freedrive + record joints | OPERATE Arm-control card (Freedrive); record in PROGRAMS + Add step |
| Control tab: ATI values | OPERATE F/T strip |
| Manual tab | OPERATE Manual card (Joint / Cartesian / Jog) |
| Gripper tab | OPERATE Gripper card + `Advanced ▸` |
| Program tab | PROGRAMS (table + inspector + run bar) |
| Policies tab | AUTOMATION |
| Frames & Calibration → TCPs / Fixtures / Vision | SETUP → TCPs / Fixtures / Vision baselines |
| Header profile combo | SETUP → Robot profile → Reconnect… |
| Diagnostics tab (controllers, freshness) | DIAGNOSTICS (+ control chain, claims, gripper CM, full log) |
| Bottom log panel | Collapsed log drawer; full log in DIAGNOSTICS |

## 10. Behind "Advanced" or menus

* Dashboard extras: protective-stop unlock, restart safety, popups, resend / load program, speed slider,
  digital outputs, power off, zeroing the UR sensor.
* Gripper raw joint values, controller manager, controller states, action type; reactivation (hidden
  until verified).
* Program step move / duplicate / YAML; raw controller claims (DIAGNOSTICS).

## 11–12. Screenshots and sizes

Rendered offscreen with the fake robot in [`docs/ux/v2.2/`](ux/v2.2/), at three sizes:

| Window | Result |
|---|---|
| **1366×768** | Fits without clipping. The window's minimum size is now 985×536 (was 1293×907). |
| **1920×1080** | Joint and Cartesian rows stay compact instead of spreading over the extra height. |
| **1280×720 logical at 1.5× scale** (a 1920×1080 high-DPI screen) | Fits; dense SETUP sections scroll inside their own areas. |

## Tracked issues (not solved here)

* MoveIt routing during live Position ↔ Admittance switching. Switching ros2_control controllers is not
  enough on its own. `moveit_controller` per mode is now explicit in the profile, but how `move_group`
  targets it still needs to be designed and tested on the real stack.
* The unified GUI controller launch for `ur7e_robotiq`. It is blocked until the repository can be read.
* The Robotiq close position (0.79, from the task description) and the reactivation service. Both need
  checking against the live stack.
* Vial geometry and F/T + payload calibration. Both need the `~/franka_ros2` sources.
* RunPolicy action server around `mujoco_multiagent_rl.policy_runner`; recording settings.
* MoveIt Servo jogging.
* **Fixed in this pass:** the snapshot deadlock, `None` counted as success, the fixture parent
  validation, and probe orientation leaking into vision baselines.
