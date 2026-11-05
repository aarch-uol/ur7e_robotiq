# Web front end (over Tailscale)

The same workstation as the desktop app, in a browser on any PC on your tailnet. The robot PC runs a small
server; the browser only shows state and sends requests. Every rule that keeps motion safe lives on the robot
PC (`moveit_gui/core.py`, shared with the desktop app), not in the browser.

## Run it

On the robot PC:

```bash
~/ur7e_gui.sh        # builds what changed, starts the robot stack and the web GUI in the ur7e_robotiq container
```

Open `http://<robot-pc tailscale ip>:8180/` from the other PC (the script prints it). The server refuses
`--host 0.0.0.0` / `::`. It also checks the `Host` header (anti DNS-rebinding): an IP address, `localhost`, a short
single-label name or `*.ts.net` (Tailscale MagicDNS) is accepted, any other DNS name needs `--allow-host NAME`
(repeatable). A POST whose `Origin` is not the page's own host:port is refused, and the page cannot be framed
(`X-Frame-Options: DENY`, CSP `frame-ancestors 'none'`). The server listens only on
the Tailscale address, so only your tailnet reaches it, and Tailscale encrypts the traffic. The header shows the
operator's IP. Never listen on `0.0.0.0`: there is no login. **Never use `tailscale funnel`**: it publishes to the
internet.

The stack and the GUI use ROS domain 42 (`UR7E_ROS_DOMAIN_ID`), apart from other ROS machines on the lab
network.

Try it without a robot: `ros2 run moveit_gui moveit_gui_web --fake`, then open `http://127.0.0.1:8180/`.

Do not run the desktop app and the web server against the same robot at the same time. Each has its own
program runner and lease, and they do not know about each other.

## Control, and what happens when the network drops

- **Viewing is free; controlling needs the lease.** Click **Take control** in the header. Only one browser holds
  control. Others see "Viewing · <name> has control", and all their robot buttons are disabled.
- **■ STOP works from every browser, with or without control** (and Esc). It stops the GUI program, MoveIt motion and the
  running policy. It is not an emergency stop: the pendant e-stop is.
  A STOP pressed while MoveIt is still planning (or while the move waits to start) also counts: that move is never sent.
- **Heartbeat.** The controlling browser checks in every 500 ms (one at a time, 2 s timeout), and every request that
  carries the control token (commands, jog and force renewals, hold-to-run) counts as one too. If the robot PC hears
  nothing for 2.5 s (closed laptop, Wi-Fi gone, tab killed), it **stops everything** and ends freedrive. If the robot
  was idle, it simply drops the lease. The robot-PC log says why and for how long it was silent ("lost contact with
  <name> (no heartbeat for 2.6 s, limit 2.5 s)").
- **Lost control, or just a slow link?** A page declares "You lost control" **only when the robot PC answers that it no
  longer holds the lease** (it timed out, or someone took over). If the answer does not arrive (network hiccup, the
  browser's connection limit busy with camera streams) the page shows a yellow "Connection problem" banner, keeps
  trying, and control continues the moment an answer comes back in time.
- **Control token.** Take control makes the robot PC issue a random secret token, sent only to that browser. Every
  command, heartbeat and release must carry it (STOP and the read-only status calls need none). The state everyone
  watches shows only the holder's name and a public id, which is no credential. The token lives in the tab's
  `sessionStorage`: **reloading the tab keeps control** (if the reload takes less than 2.5 s), a duplicated tab does
  not inherit it. Closing the tab drops control after 2.5 s.
- **Take over** from another browser is only possible while the robot is idle. The browser that loses control
  gets a message that stays until dismissed ("You lost control: <name> took over").
- **Operator names.** The first Take control asks for your name (kept in that browser); the header then shows
  "<name> @ <ip>". Click your name in the header to change it.
- **Header ACTIVITY** shows what the robot is doing from any page: PROGRAM ▶ (with the step), PAUSED, POLICY,
  PREPARING POLICY, FORCE, FREEDRIVE, JOGGING, MOVING, PAYLOAD CAL. Click it to open the page that owns it.
  Force and freedrive also show an amber banner.
- **Messages.** Errors from your own actions stay bottom right until dismissed; job results fade after 3 s. The
  bottom line shows the latest log line (errors stay there for 5 s).
- **Gripper** commands are refused while a program, a policy or the payload calibration runs.
- **GUI SPEED** (header slider, 5–100 %) scales every motion started from the GUI: MoveJ/MovePose, Plan, jog
  (step and Servo), Move here and program moves. Policies and the payload calibration poses are not scaled.
  It is shared by all browsers and resets to 100 % when the GUI restarts. The UR pendant slider (PENDANT SPEED)
  applies on top.
- **Move here** (PROGRAMS) is hold-to-run: the arm moves only while the button is held. The page renews the hold
  every 150 ms and the robot PC stops the move 0.4 s after the last renewal, so a quick tap, a dropped connection
  or a closed tab cannot run the whole move; letting go also sends STOP at once.
- **Two browsers, one program.** Every program change bumps a revision. Pages reload the program when it changes
  elsewhere, and an edit sent from a page that still shows the old program is refused ("the program was
  changed…") instead of hitting the wrong step.
- The active TCP can only be changed while the robot is idle. GUI SPEED also scales the payload calibration poses.
- **Continuous Servo jogging** uses six spring-centred sliders and is renewed by the page every 100 ms; the
  robot PC stops the jog 0.5 s after the last renewal, well before the 2.5 s lease timeout. Releasing a slider,
  losing pointer capture, blurring/hiding the tab or losing the lease stops it. Position mode adds a local
  0.5 s core deadman. Position switches to `joint_trajectory_controller` for the jog (Servo streams trajectories to it) and
  restores the scaled trajectory controller on release; Admittance uses its separate Servo trajectory-output path.
- Motion requests are refused for the same reasons as on the desktop: busy, CONTROL STATE UNKNOWN, a mode
  without MoveIt. **Confirm moves** is on by default for a real robot.
- **MoveJ targets follow the arm.** The joint and Cartesian fields show where the arm is until you type into
  one; edited fields are highlighted and only they change. The MoveJ confirmation lists the joints that move
  with their change, and a joint move above 30° always asks, even with Confirm moves off. **Reset to current**
  drops your edits.
- Requests must be JSON POSTs, so another website open in the same browser cannot send commands. Program
  files can only be plain names in the programs folder.

## Pages

| Page | In the browser |
|---|---|
| OPERATE | MoveJ (joints, waypoints), MovePose (any frame, linear), Plan → preview → Execute, Jog (step, or hold-to-jog through Servo), Force (Admittance / Impedance), 3D view (robot, goal gizmo, plan preview, F/T and force arrows, TF Frames overlay (checklist in localStorage, polled at 2 Hz via `tf_frames`), planned TCP path line, 10 s TCP Trail, Top/Front/Side/Iso and Follow TCP camera controls), control mode switch, freedrive, gripper, ATI F/T, Joint monitor (bar per joint across its URDF limits, amber within 10°, red within 3°), live F/T chart (last 20 s, Pause/Clear, torque toggle, commanded-force line) | With **Goal** on, an **orange goal-state arm** follows the goal marker live (as in RViz MotionPlanning): IK in the page from the robot model, seeded from the current joints, within joint limits, retried from other seeds near singular poses; nothing is sent to the robot and nothing is collision-checked (Plan does that). A goal it cannot reach says so and keeps the last pose that worked. Screenshot: `docs/ux/web_operate_goal_state.png`.
| PROGRAMS | Insert tab with the **jog panel and gripper** (jog, then MoveJ/MoveL (here)); waypoint rename and coordinates; **3D preview** beside the tree: the program's TCP path (loops and relative moves included) and its waypoints (selected one orange, joint targets as a ghost arm); typed waypoints and a Go to list; loop progress in the run bar; UR-style tree (Loop, Folder, Comment, …); Insert / Command / Teach tabs; edits apply when you leave a field; **Undo / Redo** (also Ctrl+Z / Ctrl+Y); unsaved changes are marked `*` and New/Open ask first; Set waypoint here, Move here; run from a selected step, step, pause, stop; The 3D preview's **▶ Play** runs a ghost arm through the whole program from where the robot is (loops included; MoveJ / non-linear MovePose in joint space, linear moves in <= 10 mm steps with IK); nothing moves, no planning, no contact. A **Move until contact** is drawn to where it meets the wall or floor of a vial of known size (a fixture with a radius: taught by touch, a vial profile, Use example vial, or the example vial while assumed), else to its full distance; the step line says which. A fixture the program uses but that is not taught yet is previewed at the profile's `example_fixtures` position (`vial` = scraping.py's target_xyz [525, 16, 40] mm in base_link, what `robot_demo` runs), else under the tool now; the note says which. SETUP > Vial Geometry > **Use example vial** saves the example as the real fixture on request. **Step through** under the view: |◀ ◀ slider ▶ ▶| picks one move (loops unrolled); the ghost repeats only that move, it is orange on the path, and the line under it says which step and loop turn it is (e.g. `Move 6 / 79 · step 8.5 · Loop 8 · 2/37 · Move relative [0, 0, -41] mm in vial`). The path is drawn for the **active TCP**: a taught pose puts its own TCP at the target (as the runner does) and the line shows where the active one goes; changing or editing the active TCP redraws it. A pose's TCP can be `active` (whichever TCP is selected when it runs; the Franka baseline example uses it) or a named TCP (TCP dropdown in the waypoint editor). A path that goes below the robot base (z < 0) is flagged in red under the preview. Screenshots: `docs/ux/web_programs_play.png`, `docs/ux/web_programs_step.png`. |
| AUTOMATION | policy list, readiness with **Fix** buttons (use the TCP, switch mode, apply the trained gains, open SETUP/DIAGNOSTICS), Prepare (shows PREPARING…, Cancel), Run, STOP POLICY, recording on/off, live feedback (when the runner gives it) |
| SETUP | TCPs, Parts, Vial Geometry and Keep-out zones share a **3D view** beside their forms (TCP frames; touched points and the fitted rim; boxes, with the box being typed drawn dashed before it is saved); F/T + Payload has a live F/T chart beside the bias table. F/T + Payload calibrates **one pose at a time**: select a pose (click it or ◀ ▶; the blue ghost shows it, nothing moves), **Move & collect** goes to that pose only and samples it, then the next uncollected pose is selected. **Collect all poses** still does the whole set in one go (starts over, visits every pose in order, then evaluates). Skip any pose that is not safe; **Evaluate** fits from the poses collected so far (3 or more), **Start over** clears them. A bias change between poses is refused (Start over). The ghost is the target, not MoveIt's path. The F/T bias **survives a robot stack restart**: the broadcaster starts with no offset, and the GUI puts the last bias back by itself within a few seconds, once nothing uses the reading (logged; `ft_bias.auto_restore: false` turns this off). With a payload applied, **Admittance (and any F/T mode, so also a program's controller step and a policy that needs it) is refused** while the F/T bias does not match it: bias lost after a stack restart, no bias (raw), or the last zero was Zero all / taken before the payload was applied. Re-apply the bias or press Zero now. A zero taken with the tool on (bias ~ the tool weight) does not bend the fit: the constant bias is estimated linearly first, then calibration_math fits the rest; a large bias is a reminder to press Zero now after Apply, not a FAIL. The fit uses ~/franka_ros2's calibration_math when it is there, else the copy shipped in moveit_gui/vendor. **Zero now** keeps the payload loaded on the admittance controller in the reading and removes only the drift; **Zero all** zeroes the tool weight too (Admittance would then subtract it twice). A **Saved (kept across restarts)** block shows what survives a restart even when nothing was evaluated this session: the payload the GUI restores on the admittance controller (mass, CoG, when, and whether the controller differs), the last calibration file (`ft_payload/<profile>_<date>.json`: PASS/FAIL, mass, poses) and the last F/T bias (in use or not, how and when, restored when). Screenshots: `docs/ux/web_setup_payload_ghost.png`, `docs/ux/web_setup_payload_saved.png`. **Vial profiles** (SETUP › Vial Geometry): several named vials (centre of the bottom in base_link, radius, height), kept in `config_dir()/vials/<profile>.yaml`; **baseline** is built in (scraping.py's target_xyz, from the profile's `example_fixtures`). **Use** makes one the fixture programs use (`vial` by default, or e.g. `vial_2` via *Use as*; refused while a program runs); add one typed, from the touched rim (*Save as vial profile…*) or from the current fixture. Screenshot: `docs/ux/web_setup_vial_profiles.png`. An untaught fixture is one preflight line saying what to do. TCPs (set active; **create / edit / delete virtual TCPs** as an offset from a TCP or link), Vial Geometry, **Keep-out zones** (named boxes MoveIt plans around: list with enable / edit / delete, form for name, frame (base or a fixture such as `vial`), size, centre, rotation, **Under TCP** helper, Re-apply; applied to MoveIt's planning scene as `gui_keepout/<name>` at startup, on every change, when move_group comes back and when a fixture they hang on moves; a box around the current TCP / flange / base is refused; stored in `~/.config/moveit_gui/calibrations/<profile>.keepout.yaml`; drawn red in the 3D views with a **Zones** toggle; apply result on DIAGNOSTICS › State), F/T + Payload (Apply to admittance: set, read back and saved), **Controller gains** (see below), **Drivers & cameras** (start / stop camera drivers and pipelines, live picture, log; see below) |
| DIAGNOSTICS | controllers, state, **recording** (rosbag2/MCAP, see below), **run history** (every program and policy run: when, how long, result, policy log folder; kept in `~/.config/moveit_gui/history/<profile>.jsonl`), log with filter, "problems only" and download |

Still desktop-only: pivot calibration of TCPs, the fixture wizard, the detailed control-stack
transition view, and the other UR dashboard commands (unlock protective stop, restart safety, popups). Play / pause /
stop of the robot program are on OPERATE › Robot.

## OPERATE layout

* Left: **Manual** (Joint / Cartesian / Jog / Force; target fields with the current value next to them; why a move
  is refused shows as a banner above Plan / Move) and **ATI F/T** with its chart. The chart takes any of Fx Fy Fz |F|
  Tx Ty Tz |T| (forces on the left axis in N, torques on the right in Nm), a 5 / 10 / 20 / 60 s window, and gets
  the sensor at 100 Hz (`ft_samples` in each state), not one value per state. The choice is kept per browser.
  A BIASED chip shows when a software F/T bias is set.
* Right: **Robot** (state line, then the three steps Power on → Release brakes → ▶ Play program with the next one
  highlighted; the steps fold away once the External Control program runs), **Arm control** (the active mode,
  Switch, Freedrive), **Gripper** (status chip; Reactivate only appears with a fault; **⟳ Reconnect gripper** appears when the gripper is not connected: the Robotiq gets its power from the robot's tool flange, so if the stack started while the robot was powered off its hardware failed to configure and its controllers never started. With the robot powered on, Reconnect activates the `robotiq_2f85` hardware and loads / configures / starts the four gripper controllers, the same as the launch does; parts that are already up are left alone. Refused while the robot is powered off. Screenshot: `docs/ux/web_operate_gripper_reconnect.png`), **Joints**.
* A plan is shown only while it can still be executed: running any other motion, or the arm moving away from the
  plan's start (program, jog, pendant), drops it and its preview. **▶ Execute plan** sits right under Plan / MoveJ.
* PENDANT SPEED in the header is the UR speed slider. The driver publishes it in percent; it was shown ×100
  (100 % read "10000%").

Screenshots: `docs/ux/web_operate_v24_running.png`, `docs/ux/web_operate_v24_power_off.png`.

## Robot power (OPERATE › Robot)

* **Power off…** — any browser that has control. GUI motion is stopped first, then the robot powers off and the
  brakes engage. It asks first.
* **Power on** and **Release brakes** — any browser that has control, after confirming that you can see the robot
  and its workspace is clear. Releasing the brakes moves the joints slightly. The log says who and whether it came
  from the robot PC or a remote browser.
  * `MOVEIT_GUI_LOCAL_POWER_ONLY=1` (environment of `ur7e_gui.sh`, passed into the container) limits them to a
    browser **on the robot PC**: other browsers do not get these buttons, and the server refuses the commands.
  * The server decides what counts as "on the robot PC", not the page: the connection must come from this machine
    (loopback or the address the server listens on), carry no proxy header (`tailscale serve`, `X-Forwarded-For`,
    …), and the page's `Origin` must be that same address and port. An SSH port forward to this PC cannot be told
    apart; it needs a login on this PC anyway.
  * Brake release is only sent in robot mode IDLE (powered on, brakes engaged); Power on only when the robot is off.
* All three go through the UR Dashboard (`/dashboard_client`). The pendant must be in **Remote Control**; in Local
  Control the robot refuses them and the reason shows in the log. Every use is logged with the operator's name.
* Once the program runs: **⏸ Pause program** / **▶ Resume program** and **■ Stop program** (Dashboard pause / play /
  stop). Power and brakes stay as they are. The state line says PLAYING / PAUSED / STOPPED (Dashboard
  `program_state`, polled every 3 s).
  * A GUI **program run pauses** with it instead of ending: Resume program carries it on with the step it was on
    (a relative move finishes at the target it had when it started, not one offset further). Pausing the robot
    program on the pendant, a stop, or a protective stop also pauses the run. A force the run applied is zeroed at
    once and re-applied (ramped) on resume. A jog, a manual move, a manual force or a policy is stopped.
  * Paused from the GUI: ▶ Resume program also resumes the run. Paused on the pendant: press Play there, then
    Resume in PROGRAMS (it is refused while the robot program is not playing).
* The driver's Dashboard client does not reconnect by itself after the robot drops the connection ("Failed to send
  request to dashboard server. Are you connected to the Dashboard Server?"). The GUI now reconnects it (quit,
  connect, retry once) when a command or the 3 s poll finds it lost.

## Controller gains (SETUP › Controller gains)

One table per mode that the profile lists under `gains:` (UR7e: Admittance and Impedance). The values are read from
the running controller node (`/admittance_controller`, `/cartesian_impedance_controller`). Edited cells are
highlighted; **Apply** first shows old → new and asks.

- **Limits.** Values are checked against the profile's min / max before anything is sent.
- **Mode not active.** The values are set in one call and take effect the next time the mode is switched on.
- **Mode active.** The change ramps in over 1 s (10 steps). A value may change at most 2× per apply. Some rows
  (which axes are compliant) are refused. Nothing may be running: no program, policy, motion, jog or applied force.
  The impedance controller re-applies its parameter wrench (zero) on every parameter change, so it would cut a
  streamed force. STOP during a ramp leaves the gains where the ramp got to, and the page says so.
- **Read back.** Every apply is read back; a value the controller did not take is an error.
- **Load-time parameters.** Parameters the controller reads only when it loads (impedance `delta_tau_max`, filters)
  are shown but not editable.

**Saved automatically, no YAML to copy.** Every successful apply, and the payload's *Apply to admittance*, is
saved in `<config>/moveit_gui/gains/<profile>.json` (`~/ur7e_data/config/...` on the host). The GUI puts the
saved values back by itself:

- when it starts;
- whenever it finds the controller with other values while its mode is not active (checked every 3 s, e.g.
  after a controller or robot-PC restart);
- just before every switch into the mode (the switch is refused if that fails).

An **active** controller is never changed behind the operator's back. A difference there is shown in red, with
**Restore saved**, which ramps like any live apply. **Forget saved** drops them; the controller then keeps its
current values until it restarts with its YAML.

Presets (named tables) are stored in the same file. Loading one only fills the table.

**Policies.** A manifest may list the gains it was trained with:

```yaml
controller_gains:
  admittance.stiffness: [250, 200, 180, 20, 20, 20]
```

AUTOMATION then shows a **Controller gains** readiness check (2 % tolerance). It fails on a mismatch, with
**Apply trained gains** as its fix.

## Force work in programs (PROGRAMS › Insert › Force & contact)

| Step | What it does |
|---|---|
| Zero F/T | software F/T bias at the current load, arm still for 1 s. Refused in Admittance or with a force applied: put it before them, in free air |
| Move until contact | linear move along a direction (tool, base or fixture axes) for at most N mm; stops when the force changes by \|ΔF\| ≥ the threshold from its value at the start. No contact: fail (default) or continue |
| Force | the wrench the tool exerts (tool or base axes), ramped in; the step ends when it is reached. It **stays applied** through the following steps |
| Force off | ramps it out |
| Move relative | the TCP moved by an offset / rotation in tool, base or fixture axes (e.g. a stroke along the vial axis) |

* A force needs a mode that takes one (Admittance; Impedance has no MoveIt moves). Limits are the profile `force:`
  limits, checked when the program is checked.
* A force never outlives the program: it is ramped out at the end of a run, zeroed at once by STOP or a failed step.
  If it ends by itself (mode change, controller stopped), the program fails at once instead of carrying on unloaded.
* **New scraping program › force baseline (scripted)** builds the non-learned baseline from the taught contact pose:
  Position, approach, Zero F/T in free air, Admittance, then per angle (one folder each): near the wall, search it,
  press with the force, N strokes down and up along the vial axis, Force off, lift. Folders start folded; every
  number is an ordinary step parameter.

* More for force work: a Force step can **go on without waiting** for its ramp (a preload during the next move; a
  Wait step gives it time). Move relative can **rotate about the frame's origin** (an orbit: with the vial fixture,
  rotate Z 10° goes to the next scraping angle, the tool facing the wall the same way; the travel is limited like an
  offset). Motion steps can **skip on failure** (`on_fail: continue`; the run ends "finished, N skipped").
* **Moves under a force (Admittance) start at the controller's reference.** A force pushes the arm off the
  trajectory controller's reference (about F / stiffness: 5 N at 250 N/m = 20 mm in free air). MoveIt plans from
  the measured joints, so a move sent as planned would jump the reference to the pushed pose and the admittance
  controller would add the push again: the push doubled at the start of every move (reproduced on the real
  controller chain). The GUI therefore shifts every plan in Admittance so it starts at the reference
  (`/admittance_trajectory_controller/controller_state`) and sends it to that controller directly; the push
  stays as it is and the path keeps its shape. A move is refused when the arm is more than 5° (any joint) off its
  reference, or the reference is not fresh (profile: `reference_start`).
  **Pause and STOP** do the same: cancelling the goal would make the controller hold at the measured (pushed)
  joints and double the push at every pause, so in Admittance they replace the move with a hold at the reference
  (the arm stops where it is; STOP then releases the force and the arm eases back by the push).
  Checked on the real controller chain by an opt-in test (mock UR arm, simulated F/T, isolated container):
  `MOVEIT_GUI_DOCKER_TESTS=1 pytest test/test_admittance_chain_docker.py` (image `ur7e_robotiq:buildtest`, or
  `MOVEIT_GUI_TEST_IMAGE`). Before the fix: 20 mm jump per move start and per pause, 200 mm walk after 5 pause
  cycles; after: under 2 mm.
* **Typed waypoints**: the editor of a MoveJ / Move to pose step shows its waypoint's numbers as fields (joints in
  degrees; or frame + XYZ mm + RPY ° of the TCP, e.g. in the vial frame) and a **Go to** list to point the step at
  another waypoint (e.g. back to `home`). Teaching (Set waypoint here) still works.
* The run bar shows the **loop iteration** ("Loop 9 · 3 / 37") and how many steps were skipped.
* **3D preview** beside the program: the TCP path of the whole program from where the robot is now, loops and
  relative moves included (kinematics from the URDF; no planning, no forces: Move until contact stops at the wall / floor of a
  vial of known size, else it is drawn to its full distance), and the waypoints.
* `programs/example_franka_scraping_baseline.yaml` is the Franka `sample_scraping` baseline (`scripts/scraping.py`)
  rebuilt with these steps: 37 angles 10° apart, preload, press Fx 5 N + Ty 2 Nm, scrape down, relax, up and orbit.
  Differences: relax torque 2 Nm (the profile limit, Franka used 3), Zero F/T before Admittance, no Franka pose-error
  compensation. Check the tool orientation of `approach` / `rim_start` and the Admittance compliant axes first.

Screenshots: `docs/ux/web_programs_scrape_baseline.png`, `docs/ux/web_programs_force_step.png`,
`docs/ux/web_programs_franka_baseline.png`, `docs/ux/web_programs_orbit_step.png`.

## F/T bias (SETUP › F/T + Payload)

A software bias on the ATI reading. The ATI box is never biased (keep `ft_hardware_biasing:=false`). The GUI sets
the `offset.*` parameters of `/ft/force_torque_sensor_broadcaster`. The broadcaster re-reads them every cycle and
adds them to the reading, so the GUI, payload calibration, policies and the admittance controller (through
TopicSensorBridge) all see the same values at once. Reading = raw sensor − bias.

* **Zero now** averages the reading for 1 s with the arm still and makes the current load read 0. It is refused
  if the arm moves, or if the reading moves more than 1 N / 0.1 Nm (someone touching the tool).
* **Clear (raw)** sets the bias to 0: the sensor's own values.
* **Set these values** sets any six values (limits in the profile, `ft_bias:`). **Save fields as set…** stores a
  named set (not applied); **Use** applies it.
* Refused while anything uses the reading: Admittance (or any `requires_ft` mode), an applied force, a program,
  policy or motion. Every change is read back and logged.
* The broadcaster starts with no bias every time the robot stack starts. The GUI then shows that the last bias
  is gone. With `ft_bias.auto_restore: true` (the default, `moveit_gui/ft_bias.py`) the GUI then puts the last
  bias back **by itself** as soon as nothing uses the reading (checked every `poll_s`) and logs it; with
  `auto_restore: false` it only shows that the bias is gone and offers **Re-apply**. While it waits, SETUP >
  F/T + Payload says why (e.g. a program runs). At every GUI start the log says, once, what the broadcaster has
  against the saved bias ("F/T bias on ...: the saved one" or "... NOT the saved one ...; re-applying it"), and
  once if the broadcaster does not answer yet. `scripts/ur7e_gui.sh` keeps the GUI's output in
  `~/ur7e_data/logs/gui_<time>.log` (next to the stack's `launch_<time>.log`). Parameter calls use one client
  per node, kept (a new client per call had to find the service again within 0.5 s every time).
  With **Zero now** (payload kept) the reading at rest still shows the tool weight (Fz ~10 N on the UR7e tool):
  that is the payload the admittance controller removes, not a lost bias. Opt-in check on the real broadcaster
  (docker, no network): `MOVEIT_GUI_DOCKER_TESTS=1 pytest test/test_ft_bias_restart_docker.py`.
* The payload calibration holds only for the bias it was collected with: **Apply to admittance** is refused after
  the bias changed (collect again, or set the bias back). Zeroing with the gripper on and gravity compensation
  set makes Admittance subtract the gripper weight twice.

Screenshots: `docs/ux/web_setup_ft_bias_raw.png`, `docs/ux/web_setup_ft_bias_zeroed.png`.

## Recording (DIAGNOSTICS › Recording)

**● Record** writes a rosbag2 (MCAP) of the ticked topics. The header shows **● REC m:ss** while it runs; click it to
come back here. Recording never moves the robot. Starting needs control, but anyone can stop it. The big STOP does
*not* end a recording, because the moments after a STOP are often the ones you want on record.

- **What gets recorded** is opt-in: presets (Robot state, Force/torque, Gripper, Policy, Planning, Cameras compressed,
  Cameras raw, ROS log), resolved against what is published now, plus single topics ticked or unticked in the list.
  Image and point-cloud topics are only ever picked by the camera presets. Raw cameras are about 30 MB/s per
  640×480 camera. The choice is saved per robot profile.
- **Limits:** a recording stops by itself after the time or size limit (default 10 min / 4000 MB), or when less than
  1 GB of disk is free. It won't start with less than 2 GB free.
- **Auto-record every program / policy run:** one recording per run, named after it and stopped when it ends. The run
  history links the bag folder. A recording you started yourself is left alone.
- **Files:** `<programs dir>/../recordings/<date>_<name>/` holds `bag/` (the MCAP and metadata.yaml), `gui_meta.json`
  (topics, who or what started it, the linked run, why it ended) and `record.log`. With `~/ur7e_gui.sh` that is
  `~/ur7e_data/recordings/` on the host. Open the `.mcap` in PlotJuggler 4 or Lichtblick/Foxglove, or replay it
  with `ros2 bag play`.
- Closing the GUI finishes the bag cleanly (SIGINT to `ros2 bag record`).

## Parts, camera calibration, setups, restart (SETUP)

The SETUP menu is in three groups, in the order a new user needs them: **Robot** (Setups, Parts, TCPs),
**Calibrate** (Camera calibration, F/T + Payload, Vial Geometry) and **Cell** (Drivers & cameras, Keep-out zones,
Controller gains). Each item shows a short status: the setup in use (or *changed*), how many parts / TCPs / keep-out boxes, ✓
when a camera, payload or vial is calibrated. One bar at the top of every SETUP page says when saved parts (or
calibrated cameras, or a base switch) are not in the robot model yet, with **Restart robot stack…**.

**TCPs** (SETUP › TCPs): one list (Edit, Delete, Set active) and one editor: name and parent once, then
*Type the offset* or *Measure the tip (pivot)*: hold the tip on one fixed point in freedrive, tilt, Capture 6-12
times, Solve, Save TCP. The pivot gives the position only; the rotation is typed (usually 0 0 0).

**Parts** (SETUP › Parts): things bolted to the robot or the cell, without editing URDF. A part is a box, cylinder,
sphere, mesh (.stl, .obj or .dae upload) or a bare frame, on a parent (a robot link, or another part) at an offset in
mm / deg. A shape sits on its frame and grows along +Z, so **+** on a part (or *on far face*) stacks the next piece
where the previous one ends, like the real tool. Presets: spatula on the gripper, wrist camera bracket, RealSense
D435, table, frame only.

The editor shows an unsaved part as a translucent **draft** in the SETUP 3D view. Move / Rotate uses the same
drag gizmo as OPERATE's Goal TCP; Local / Parent chooses the axes and Snap can use 1 mm / 1° or 5 mm / 5° steps.
Typing XYZ / RPY moves the draft too. **Import mesh…** starts a new mesh draft immediately, places it standing on
its frame and uses **auto** collision by default; nothing becomes TF / MoveIt geometry until **Save part**.
Cancel removes the draft and restores the saved part when editing.

For a mesh, **auto (recommended)** builds a few light convex collision meshes automatically: it keeps the CAD mesh
for the visual (so a .dae keeps its colours), but MoveIt and the generated URDF use the simplified pieces. The
collision line reports piece / triangle count and volume excess; Advanced can change the piece limit, triangles per
piece and tolerance, or regenerate. **Collision** in the 3D view shows the same generated pieces, including an
unsaved draft once its preview is ready. Existing saved mesh parts without a collision setting keep the old
single-convex-hull behaviour.

* **Live at once:** a static TF frame (parent → part), a MoveIt collision object `gui_part/<name>` (attached to the
  arm link the part rides on, with that link's neighbours as touch links, so plans carry it; a world object for
  parts on the base), and the 3D views on every page. Sent only while the robot is idle (a scene change can abort a
  running trajectory), re-checked every 10 s, re-sent if move_group restarted. The status line says what MoveIt
  confirmed.
* **Robot model after a restart:** every save also writes `parts/generated/robot_parts.urdf.xacro` and
  `robot_parts.srdf.xacro` (collision pairs: each part against what it is bolted to and the links one joint away).
  `ur7e_gui.sh` passes them to the robot's xacro (`UR7E_USER_PARTS_URDF` / `_SRDF`), so after **Restart robot
  stack** robot_description, RViz, recordings and MoveIt's self-collision have the parts. The table's status says
  *live* (not in the robot model yet), *in model*, *restart* (changed since) or *removed*; the banner offers the
  restart. Once the robot model has a part, the GUI drops its live copy.
* **TCP at its tip…** makes a virtual TCP at the part's far face (on the planning link). It follows the part when
  you edit it. Refused for a part on something that moves relative to the flange (a finger).
* **Meshes:** STL and OBJ are in whatever unit the CAD used. The upload guesses it from the size (mm / m / inch can be
  changed; the size shown in mm must match the real part). A .dae carries its unit and keeps its own colours. *Place
  it* puts the mesh's origin where you want it on the part frame: *as exported*, *centred*, or *standing on the
  frame* (centred in X / Y, bottom on the frame, so it stacks like the other shapes). Up to 20 MB and 400 000
  triangles; textures are not used.
* **Collision** per part: new meshes default to *auto (recommended)*, which generates a small deterministic set of
  conservative convex pieces under `parts/generated/collision/`. *Full visual mesh*, *single convex hull*,
  *bounding box* and *none* remain available; saved mesh parts that omit `collision` still use the historical
  single-hull behaviour. The same collision geometry goes live in MoveIt and into the generated URDF.
* **Visual / Collision** (bottom right of every 3D view): show the robot's visual meshes, its collision shapes
  (translucent green: what MoveIt checks, robot links and parts), or both. Every 3D view follows the choice.
* A new tool changes the payload: redo SETUP › F/T + Payload.

Screenshot: `docs/ux/web_setup_parts.png`.

**Camera calibration** (SETUP › Camera calibration): hand-eye with a ChArUco board (7×5 squares, 35 / 26 mm,
DICT_5X5_100 by default; **Print board…** opens it at true size; print at 100 %, measure a square, enter it, *Save
board*). Eye-in-hand (camera on the tool, board still in the cell: gives flange → camera) or eye-to-hand (camera on
a stand, board on the tool: gives base → camera).

The page is four steps and the next step's button is the blue one (Start, Collect all…, Solve, Save + add to robot).
The board is one line (*7 × 5 squares of 35 mm …*); *Change the board* opens its fields.

1. Pick the camera and **Start**: intrinsics come from its CameraInfo (RealSense / ZED are factory calibrated).
2. **Look** shows what the detector sees (corners, reprojection error, distance). Then either **Capture here**
   after each freedrive / jog, or **Plan 12 poses** (±tilt about the board, or about the tool tip for eye-to-hand)
   and **Collect all…**: MoveIt moves to each (collision-aware, GUI speed applies, STOP stops it) and samples
   where the board is seen.
3. **Solve**: OpenCV calibrateHandEye (Park; Tsai, Horaud, Andreff, Daniilidis shown as a cross-check). The
   verdict comes from where the samples put the board: good ≤ 2 mm / 0.3°, usable ≤ 5 mm / 1°. Outlier samples are
   named; remove them with × and solve again.
4. **Save + add to robot**: kept in `cameras/<profile>.yaml` (with the samples) and added as a camera part. The part
   is the camera driver's own root frame (e.g. `camera_link` above `camera_color_optical_frame`), placed so the
   optical frame is where the calibration says. So the driver's TF tree hangs on the robot; nothing else may
   publish a parent for that frame. Editing the part's offset afterwards drops its *calibrated* mark.

MoveIt Calibration (the RViz plugin) is not released for Jazzy; this uses the same board type and OpenCV's solvers.
Screenshot: `docs/ux/web_setup_handeye.png`.

**Base** (SETUP › Setups): what is on the flange. These are fixed parts of the robot model, built by the robot's own
xacro: *F/T sensor + gripper* (default), *Gripper* (coupling straight on tool0), *F/T sensor* (your own tool goes on
it as parts), *Bare arm*. The GUI never edits a base: parts go on top of it. **Switch…** (idle, Position) writes
`config_dir()/base` and restarts the robot stack: URDF, SRDF (MoveIt's groups and collision pairs) and controllers
all change together (`UR7E_EE_CONFIG`). On a base without the F/T sensor, Admittance is disabled. Without the
gripper, the gripper controls and Impedance (its end effector is robotiq_tcp) are disabled. If the active TCP hangs
on hardware the base does not have, the GUI falls back to tool0. Parts on links the base does not have stay saved
but are left out of the model (listed as problems). `ur7e_gui.sh --base <name>` does the same from the shell.

**Setups** (SETUP › Setups): one physical configuration by name (e.g. *scraping spatula*, *pick + wrist cam*): parts
and their meshes and generated URDF, TCPs and fixtures, keep-out zones, F/T bias, gains and payload, vial profiles,
camera calibrations, the last payload calibration, and the base it was saved on. Programs and policies are shared.
**Load…** (idle, Position) puts the files in place and the setup's base back, and restarts the GUI, and the robot
stack too when the setup's parts or base differ from the running robot model. So the usual way to change tools is:
build it in Parts, save it as a setup, and load setups to switch; the files in use before are kept as an automatic backup (`_before_…`, newest 5). Stored in
`config_dir()/setups/<name>/`.

The top of the page says which setup is **in use** (the last one saved or loaded) and what changed since
(*Changed since saved: TCPs + fixtures*), with **Save into "<name>"**. Under the save form, *Saves:* lists what a
save would hold. **Load…** first shows only what differs from the files in use, a base change, and what restarts.
The setup in use is kept in `config_dir()/setups/current.yaml`; the comparison leaves out the generated URDF/SRDF.

**Restart** (SETUP › Setups, and the bar on every SETUP page): *Restart GUI* (exit code 76) or *Restart robot stack* (75: the
container stops and starts again with the parts, about a minute; press Play on the pendant again unless headless).
`ur7e_gui.sh` does the restart; refused while anything moves, runs or pushes. Every browser takes control again.

## Drivers & cameras (SETUP › Cell)

One list of the ROS processes the GUI starts and stops for you: camera drivers now, vision pipelines later.
**+ Add…** asks *what is it?* (Intel RealSense, USB / webcam, Stereolabs ZED, a ROS launch file, a ROS node),
then a name (it becomes the ROS namespace) and a few settings. Each row has a status dot and **Start / Stop**;
selecting it shows the live picture, its topics (green = publishing) and the log.

- **Ready** = every topic it should make publishes. *No data* = it runs, but a topic is missing after 25 s (see the
  log). *Failed* = it stopped by itself. *Not installed* = its ROS package is not in the robot image.
- **Needs**: a pipeline can need a camera. Start starts the camera first and the pipeline once the camera is
  ready; Stop stops the pipeline first. Programs will use the same "ready" check.
- **Start with the GUI** starts it whenever the GUI starts (so after a robot stack restart too).
- Stop sends Ctrl-C (SIGINT), then SIGTERM after 8 s, then SIGKILL. A GUI-only restart leaves the drivers running
  (the new GUI takes them over); closing the GUI or restarting the robot stack stops them.
- Nothing here moves the robot. The command is `ros2 launch …` / `ros2 run …` without a shell; it is shown under
  the log. The list is `config_dir()/services/<profile>.yaml`, the logs `config_dir()/services/logs/` (newest 5
  per driver). Calibrated camera colour topics appear first in Camera calibration.
- The robot image needs the drivers (`ros-jazzy-realsense2-camera`, `ros-jazzy-usb-cam` in `docker/Dockerfile`)
  and the container needs the USB devices: `-v /dev:/dev --device-cgroup-rule='c 81:* rmw'
  --device-cgroup-rule='c 189:* rmw'` on the launcher's `docker run`. ZED needs the ZED SDK (CUDA): not in the
  image yet.

## Cameras over the tailnet

Only one image topic is subscribed, and only while someone looks at a picture in SETUP › Drivers & cameras. It is sent as JPEG at
up to 10 frames/s. If two browsers watch different topics, the last one to open wins.

## Files

`moveit_gui/web/server.py` (HTTP, standard library only), `moveit_gui/web/service.py` (lease, commands, state;
tested without HTTP in `test/test_web.py`), `moveit_gui/web/workcell.py` (Parts, Camera calibration, Setups, restart; `test/test_workcell.py`), `moveit_gui/web/static/` (the app: HTML, CSS, `app.js`, `workcell.js`, `drivers.js`, `view3d.js`, no build
step). Logic without the web: `moveit_gui/parts.py`, `handeye.py`, `setups.py`, `services.py` (drivers; `test/test_services.py`). Screenshots: `docs/ux/web_*.png` (`tools/web_screenshots.py`).
