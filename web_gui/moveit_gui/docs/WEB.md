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
- **Several users, also from one IP** (`web/users.py`). Each browser is one user, however many tabs it has open.
  It keeps a random id in its localStorage, so browsers on one PC, or everyone coming in through `tailscale serve`
  (all 127.0.0.1), are still told apart. The people icon in the header shows how many browsers have the GUI open.
  Click it for the list: name, where they connect from, who has control. **Change my name** is at the bottom of the
  list. The name is asked once per browser. Same names get (2), (3). The browser's own id is never shown to the
  others; they see a short id derived from it.
- **Ask for control.** When someone else has control, **Ask** sends them a pop-up: **Give** or **Keep** (it closes
  by itself after 45 s, and the question ends after 60 s). Give hands control straight to the one who asked, only
  while the robot is idle. For 15 s nobody else can take it. Keep tells the one who asked, who can ask again
  after 20 s. **Take over** still works without asking, but only while the robot is idle.
- **Roles** (`web/roles.py`, SETTINGS › Roles). **Viewer** watches only and cannot take control. **Operator** runs the
  robot: moves, programs, gripper, force, policies, recording. **Admin** also changes the setup: TCPs, calibrations,
  keep-out zones, gains, parts, drivers, hand-eye, setups and roles. A person is their Tailscale login when the page
  comes through `tailscale serve`, else their IP address. A browser on the robot PC is always Admin, so the lab can
  never be locked out. Roles are kept in `~/.config/moveit_gui/roles.yaml`. Without that file everyone is Admin (as
  before roles). The people list shows each person's role. STOP works for everyone.
- **Top bar.** Left to right: the robot (with an amber **FAKE** tag and an amber line along the top bar when no
  robot is connected), its state, the active TCP, the control mode, what the GUI is doing, warnings, the GUI speed,
  **● Record**, then one chip for control (who has the GUI open, **View only** while you watch, who has control,
  **Take control** / **Ask** / **Release**), and STOP. In a narrow window the labels shorten (● alone for Record, the
  eye alone for View only) so STOP always stays in view.
- **■ STOP works from every browser, with or without control** (and Esc). It stops the GUI program, MoveIt motion and the
  running policy. It is not an emergency stop: the pendant e-stop is. It is the only filled red button. The others
  stop one thing and are outlined in red: **■ Stop program** (PROGRAMS run bar) and **■ Stop policy** (AUTOMATION).
  On OPERATE › Robot, **Pause pendant** and **Stop pendant** act on the program on the UR pendant.
  A STOP pressed while MoveIt is still planning (or while the move waits to start) also counts: that move is never sent.
- **Heartbeat.** The controlling browser checks in every 500 ms (one at a time, 2 s timeout), and every request that
  carries the control token (commands, jog and force renewals, hold-to-run) counts as one too. If the robot PC hears
  nothing for 2.5 s (closed laptop, Wi-Fi gone, tab killed) while the robot moves, it **stops everything** and ends
  freedrive. If the robot was idle, the holder is only **away** (a tab in the background, a short gap): it keeps
  control for 2 minutes and is back with its next heartbeat. The header and the people list show "(away)". An away
  holder never stops the robot, and anyone can take over from it. After 2 minutes away, control is dropped. The robot-PC log says why and for how long it was silent ("lost contact with
  <name> (no heartbeat for 2.6 s, limit 2.5 s)").
- **Lost control, or just a slow link?** A page declares "You lost control" **only when the robot PC answers that it no
  longer holds the lease** (it timed out, or someone took over). If the answer does not arrive (network hiccup, the
  browser's connection limit busy with camera streams) the page shows a yellow "Connection problem" banner, keeps
  trying, and control continues the moment an answer comes back in time.
- **Control token.** Take control makes the robot PC issue a random secret token, sent only to that browser. Every
  command, heartbeat and release must carry it (STOP and the read-only status calls need none). The state everyone
  watches shows only the holder's name and a public id, which is no credential. The token lives in the tab's
  `sessionStorage`: **reloading the tab keeps control**, a duplicated tab does not inherit it. Closing the tab with
  the robot idle drops control after 2 minutes (or at once when someone takes over).
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
- **Linear speed (mm/s).** OPERATE › Manual › Pose with **Straight line** on shows a **mm/s** field next to Speed %:
  the TCP moves at this **constant** speed (scraping needs that), with short ramps at the start and the end. Empty or
  0 = Speed % only, as before. The limit is the profile's `moveit.max_cartesian_speed_mm_s` (250 mm/s). Linear
  program steps take the same speed as **Max mm/s** in the step editor (`speed_mm_s` in the file: MoveL, Move
  relative when linear, Move until contact); the step line shows it. MoveIt ignores its own Cartesian speed fields,
  so the GUI retimes the planned path itself, for the TCP point (a virtual TCP too, from the URDF): the path stays
  the same, the ramps use `moveit.cartesian_accel_mm_s2` (500) × Accel %, and the joints stay within their URDF
  velocity × Speed % and `moveit.joint_accel_limit_rad_s2` × Accel %. If those limits need it, the whole move is
  slower and the log says so. GUI SPEED applies on top. The move can be slower than set, never faster.
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
  files can only be plain names in the programs folder (or in a profile's folder inside it).

**Programs per profile.** PROGRAMS shows the programs of the profile selected next to the file list (at first
your browser's profile, else *Shared*). A profile's programs are plain files in `<programs dir>/<profile>/`
(batch files beside them). *Shared* is the programs folder itself: the programs that were here before
profiles, and the examples. Everyone can open and run them. Save and Save as put the program into your own profile
(or Shared): another person's programs cannot be changed. **Copy from…** copies a program of another profile (or Shared) into the shown one; the copy is
independent. Commands take `owner` (`program_list`, `program_open`, `program_save`) and `program_copy` takes
`source_owner`, `owner`, `new_name`, `replace`. Policies, batches and the history refer to a program by its file name.

**Settings per profile** (`worksets.py`). The files in use (setups.py `members()`: TCPs, fixtures, keep-out zones,
F/T bias and limit, gains + payload, vials, flasks, parts, camera calibrations, recording settings, the F/T match
simulation result) belong to one profile at a time, named in `profiles/active.yaml`. The robot runs the settings of the one in control. Taking control
while another profile's settings are in use asks to switch (pop-up; `profile_activate`): the files in use are copied
back to their profile (`profiles/<name>/live/`), the new person's copy is put in place, and the GUI restarts (the
robot stack too when the parts or the base differ); the page then takes control again by itself. A profile without
settings of its own starts from a copy of the ones in use or from a setup (`start: "setup"`, `owner`, `name`).
Settings that are nobody's yet (from before profiles) become the first taker's without a restart. Until the switch,
every command that needs control is refused, so nobody changes another person's settings. Setups and programs are
written only into the profile of the one in control (or Shared); loading someone's setup into your settings keeps the
backup in your profile. A profile whose settings are in use cannot be deleted.

The settings prompt uses the lease response immediately, even if the live state stream is briefly behind it. A stale
queued prompt cannot cancel a newly acquired lease by itself; an explicit Cancel still releases control. Refused web
commands are written to the GUI log as `web: REFUSED <command>: <reason>` so a failed handoff is diagnosable.

## Pages

| Page | In the browser |
|---|---|
| OPERATE | MoveJ (joints, waypoints), MovePose (any frame, linear), Plan → preview → Execute, Jog (step, or hold-to-jog through Servo), Force (Admittance / Impedance), 3D view (robot, goal gizmo, plan preview, F/T and force arrows, TF Frames overlay (checklist in localStorage, polled at 2 Hz via `tf_frames`), planned TCP path line, 10 s TCP Trail, Top/Front/Side/Iso and Follow TCP camera controls), control mode switch, freedrive, gripper, ATI F/T, Joint monitor (bar per joint across its URDF limits, amber within 10°, red within 3°), live F/T chart (last 20 s, Pause/Clear, torque toggle, commanded-force line) | With **Goal** on, an **orange goal-state arm** follows the goal marker live (as in RViz MotionPlanning): IK in the page from the robot model, seeded from the current joints, within joint limits, retried from other seeds near singular poses; nothing is sent to the robot and nothing is collision-checked (Plan does that). A goal it cannot reach says so and keeps the last pose that worked. Screenshot: `docs/ux/web_operate_goal_state.png`.
| PROGRAMS | Insert tab with the **jog panel and gripper** in a drawer, closed at first so the commands start at the top (it stays as you leave it; jog, then MoveJ/MoveL (here)); the tree fills the page height; waypoint rename and coordinates; **3D preview** beside the tree: the program's TCP path (loops and relative moves included) and its waypoints (selected one orange, joint targets as a ghost arm); typed waypoints and a Go to list; loop progress next to the run controls; UR-style tree (Loop, Folder, Comment, …); Insert / Command / Teach & more / **Batch** tabs; **Teach by hand** (waypoints, or a path replayed as taught or as MoveL points) and **batch runs** (see below); edits apply when you leave a field; **Undo / Redo** (also Ctrl+Z / Ctrl+Y); unsaved changes are marked `*` and New/Open ask first; Set waypoint here, Move here; run from a selected step, step, pause, stop; The 3D preview's **▶ Play** runs a ghost arm through the whole program from where the robot is (loops included; MoveJ / non-linear MovePose in joint space, linear moves in <= 10 mm steps with IK); nothing moves, no planning, no contact. A **Move until contact** is drawn to where it meets the wall or floor of a vial of known size (a fixture with a radius: taught by touch, a vial profile, Use example vial, or the example vial while assumed), else to its full distance; the step line says which. A fixture the program uses but that is not taught yet is previewed at the profile's `example_fixtures` position (`vial` = scraping.py's target_xyz [525, 16, 40] mm in base_link, what `robot_demo` runs), else under the tool now; the note says which. SETUP > Vial > **Use example vial** saves the example as the real fixture on request. **Step through** under the view: |◀ ◀ slider ▶ ▶| picks one move (loops unrolled); the ghost repeats only that move, it is orange on the path, and the line under it says which step and loop turn it is (e.g. `Move 6 / 79 · step 8.5 · Loop 8 · 2/37 · Move relative [0, 0, -41] mm in vial`). The path is drawn for the **active TCP**: a taught pose puts its own TCP at the target (as the runner does) and the line shows where the active one goes; changing or editing the active TCP redraws it. A pose's TCP can be `active` (whichever TCP is selected when it runs; the Franka baseline example uses it) or a named TCP (TCP dropdown in the waypoint editor). A path that goes below the robot base (z < 0) is flagged in red under the preview. Screenshots: `docs/ux/web_programs_play.png`, `docs/ux/web_programs_step.png`. |
| AUTOMATION | policy list, readiness with **Fix** buttons (use the TCP, switch mode, apply the trained gains, open SETUP/DIAGNOSTICS), Prepare (shows PREPARING…, Cancel), Run, ■ Stop policy, recording on/off, live feedback (when the runner gives it) |
| SETUP | TCPs, Parts, Vial Geometry, Flask Geometry and Keep-out zones share a **3D view** beside their forms (TCP frames; touched points and the fitted rim; boxes, with the box being typed drawn dashed before it is saved); F/T + Payload has a live F/T chart beside the bias table. F/T + Payload calibrates **one pose at a time**: select a pose (click it or ◀ ▶; the blue ghost shows it, nothing moves), **Move & collect** goes to that pose only and samples it, then the next uncollected pose is selected. **Collect all poses** still does the whole set in one go (starts over, visits every pose in order, then evaluates). Skip any pose that is not safe; **Evaluate** fits from the poses collected so far (3 or more), **Start over** clears them. A bias change between poses is refused (Start over). The ghost is the target, not MoveIt's path. The F/T bias **survives a robot stack restart**: the broadcaster starts with no offset, and the GUI puts the last bias back by itself within a few seconds, once nothing uses the reading (logged; `ft_bias.auto_restore: false` turns this off). With a payload applied, **Admittance (and any F/T mode, so also a program's controller step and a policy that needs it) is refused** while the F/T bias does not match it: bias lost after a stack restart, no bias (raw), or the last zero was Zero all / taken before the payload was applied. Re-apply the bias or press Zero now. A zero taken with the tool on (bias ~ the tool weight) does not bend the fit: the constant bias is estimated linearly first, then calibration_math fits the rest; a large bias is a reminder to press Zero now after Apply, not a FAIL. A **FAIL** fit can still be saved or used: Save / Use for compliance list the failed checks and ask first; the file then says `accepted_despite_failed_checks: true` and the result shows *FAIL · used anyway*. The fit uses ~/franka_ros2's calibration_math when it is there, else the copy shipped in moveit_gui/vendor. **Zero now** keeps the payload loaded on the admittance controller in the reading and removes only the drift; **Zero all** zeroes the tool weight too (Admittance would then subtract it twice). A **Saved (kept across restarts)** block shows what survives a restart even when nothing was evaluated this session: the payload the GUI restores on the admittance controller (mass, CoG, when, and whether the controller differs), the last calibration file (`ft_payload/<profile>_<date>.json`: PASS/FAIL, mass, poses) and the last F/T bias (in use or not, how and when, restored when). Screenshots: `docs/ux/web_setup_payload_ghost.png`, `docs/ux/web_setup_payload_saved.png`. **Vial profiles** (SETUP › Vial): several named vials (centre of the bottom in base_link, radius, height), kept in `config_dir()/vials/<profile>.yaml`; **baseline** is built in (scraping.py's target_xyz, from the profile's `example_fixtures`). **Use** makes one the fixture programs use (`vial` by default, or e.g. `vial_2` via *Use as*; refused while a program runs); add one typed, from the touched rim (*Save as vial profile…*) or from the current fixture. Screenshot: `docs/ux/web_setup_vial_profiles.png`. An untaught fixture is one preflight line saying what to do. TCPs (set active; **create / edit / delete virtual TCPs** as an offset from a TCP or link), Vial Geometry, **Keep-out zones** (named boxes MoveIt plans around: list with enable / edit / delete, form for name, frame (base or a fixture such as `vial`), size, centre, rotation, **Under TCP** helper, Re-apply; applied to MoveIt's planning scene as `gui_keepout/<name>` at startup, on every change, when move_group comes back and when a fixture they hang on moves; a box around the current TCP / flange / base is refused; stored in `~/.config/moveit_gui/calibrations/<profile>.keepout.yaml`; drawn red in the 3D views with a **Zones** toggle; apply result on DIAGNOSTICS › State), F/T + Payload (Apply to admittance: set, read back and saved), **Controller gains** (see below), **Drivers & cameras** (start / stop camera drivers and pipelines, live picture, log; see below) |
| DIAGNOSTICS | controllers, state, **recording** (rosbag2/MCAP, see below), **run history**, **activity** (see below), log with filter, "problems only" and download |

Still desktop-only: pivot calibration of TCPs, the fixture wizard, the detailed control-stack
transition view, and the other UR dashboard commands (unlock protective stop, restart safety, popups). Play / pause /
stop of the robot program are on OPERATE › Robot.

Questions ("Delete …?", "Power on the robot?", names) are the GUI's own pop-ups (`static/dialog.js`), not the
browser's. Enter answers OK, Esc or a click outside answers Cancel. A question that deletes, forgets or clears has a
red button named after it.

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
* A saved pose picked in the Joints tab shows as a **ghost arm** in the 3D view before you move. It stays while
  the fields hold that pose, gives way to a plan preview, and goes when the arm gets there, after the MoveJ, or on
  Use current.
* A Position hold-to-jog runs on its own controller (the hidden `servo_position` mode) and switches back on release.
  Arm control keeps showing Position meanwhile and offers no switch. If switching back fails, the real mode shows.
* PENDANT SPEED in the header is the UR speed slider. The driver publishes it in percent; it was shown ×100
  (100 % read "10000%").
* **Layouts** (`static/views.js`). Layout **1 Main** is the page above and is fixed: Manual, F/T sensor and its
  chart on the left, the 3D view in the middle, the robot cards on the right. Nothing in it moves or resizes.
  In every layout, **−** in a card's title folds it to its title. The chart's own button moves it under the 3D view
  and back (kept per browser). Make your own layouts in SETTINGS › Layouts (New, Duplicate, rename, Edit, Delete). Two ready-made ones
  come with it: **2 Force** (chart, one camera, Gripper minimised) and **3 Cameras** (two cameras under the
  3D view).
  * With two or more layouts, their numbers show in the left menu under OPERATE (only on OPERATE) to switch. In your
    own layouts the column buttons under the numbers hide or show the side columns. While you edit a layout, its
    edit bar is in the top bar.
  * In your own layouts:
    * Drag any line between windows to resize, including the line under the 3D view.
    * Windows always fill their space. There are no gaps and no own window sizes.
    * **Fit to picture** (the frame button in a camera's bar, on by default): the camera window takes its
      picture's shape. Under the 3D view the row height follows the cameras' width. Beside it the stack width
      follows the cameras' height. In a side column the height follows the column width. While it fits, the line
      it sets is off. It falls back to the line (and the picture letterboxes) if fitting would leave the 3D view too
      little room (the 3D view always keeps at least half the height and half the width; one camera alone under
      it would be too tall), or with fit off.
    * The F/T chart has no number line under the plot; the F/T sensor card shows the numbers.
    * **−** minimises a window to its title bar, in place. Nothing else moves.
    * F/T sensor › **Chart** opens the chart right under the card (press again to close it). That chart stays
      attached to the card: it cannot be minimised or dragged, and it moves with the card.
  * **Edit** (from SETUP) puts a bar in the top bar: name, **+ View**, Reset, Done.
    * Drag a window by ⠿ to move it above or below another window. Dropping it on the right or bottom edge of the
      3D view starts a stack there.
    * Each view is a camera (with its topic menu), the F/T chart, a **Motion planner**, a **Joint chart**, a
      **Program run**, a **Log** or empty. **×** closes it. There is one of each, except cameras and empty views.
      * Motion planner: plans Manual's target (its Joints or Pose tab, Speed, Accel), Preview again, Clear,
        Execute, plus the plans of this session (`static/panels.js`).
      * Joint chart: the joint angles or speeds over the last 20 s. Click J1–J6 to show or hide one.
      * Program run: the running program, its step and loop, why it paused, Pause / Resume / Stop.
      * Log: the last lines of the GUI log, with a filter and "problems only".
    * The 3D view is pinned: it never moves, minimises or closes.
    * Window bars show only while editing. Outside edit mode a view looks like any other card.
  * A small browser window narrows the side columns first, down to 280 px. After that they fold to a strip by
    themselves and come back when there is room.
  * Layouts are kept per browser (localStorage `op-layouts1`).

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

## Controller gains (SETUP › Gains)

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

## Record and photo steps (PROGRAMS › Insert › Record & photo)

Three steps that never move the robot. **Record start** begins a recording (preset "Everything for a run", or the
topics ticked in DIAGNOSTICS) with an optional note; the folder is named after the program and `meta.json` carries
the program name and the operator. If a recording already runs (yours, or auto-record) the step carries on with it.
**Record stop** finishes the recording that runs. A recording that a program started always ends with that program,
whether it finishes, fails or is stopped; one you started yourself is left alone. **Photo** saves one full-size JPEG
from the chosen camera as `<date>_<time>_<label>.jpg` (label empty: `stepN`) into the `photos/` folder of the recording
that runs now, else into `photos/` next to the recordings. It fails the step if the camera gives no frame, and
the check before a run refuses a camera topic that publishes nothing. In fake mode the photo is a generated image. Older programs load as before.

## Teach by hand (PROGRAMS › Teach & more)

Press **● Record**, then move the arm by hand (freedrive; jogging works too). Nothing moves by itself while you teach.
Teaching changes the program, so it needs control; it ends when control changes hands. A program run or a batch
cannot start while you teach. New waypoints are named `taught_001`, `taught_002`, … and are stored in the frame chosen
next to Freedrive (`base_link`, or a fixture such as `vial`, so they follow the fixture). New steps go where the Insert
tab's Place says, at the selected step.

- **Waypoints**: every **Mark** adds a waypoint and a move to it (MoveL, MoveJ or MoveJ to pose, picked before
  Record). With **Mark when still 1 s** it also marks by itself when the arm has stood still for about a second
  after moving (not at the start). Each mark is one ordinary edit: Undo takes it back. **Finish** ends teaching.
- **Path**: while you move the arm the joint states are recorded about every 10 ms, with their time stamps (and
  the TCP pose 20 times a second, for the view; at most 10 minutes). **Finish** shows the taught motion in the 3D
  view (blue). **Insert as** then offers:
  - **Replay (as taught)**, the default: one **Replay** step that runs the recorded joint motion back as it was
    moved, with its timing (**Speed %** of the taught timing, 100 = as taught; the GUI speed scales it too). The
    samples are smoothed a little (20 ms) against time-stamp jitter, the standstill before and after the motion is
    cut off, and samples are left out only where every recorded sample stays within 0.03° per joint of the replayed
    line, so pauses keep their length. The points are stored in the step (`points: [[t, q1..q6], ...]`, s and rad).
    When it runs, the arm first goes to the first point with an ordinary planned MoveJ (the step's MoveJ %), then
    the whole motion is sent as one trajectory to the active mode's controller, the same way a planned move is.
    Before anything moves it is refused if a point leaves the URDF joint limits or the TCP or a joint origin enters
    an enabled keep-out zone (the same check runs again when the step starts). If the motion would exceed a joint
    speed (URDF) or acceleration limit (`moveit.joint_accel_limit_rad_s2`), it is slowed down as a whole until it
    does not, and the log says by how much. STOP stops it; Pause stops it and Resume carries on from about where it
    stopped (a short MoveJ back onto the path first). The 3D preview draws the replayed TCP path from the joints.
  - **MoveL points**: the path as a few points: no TCP sample is further than the **Tolerance** (mm, and degrees
    of tool rotation) from the straight MoveL line between two points (the panel shows the point count and the
    largest deviation; the grey markers). Insert adds a folder "Taught path HH:MM": a MoveJ to pose to the start,
    then one MoveL per point. **Max mm/s** sets the MoveL speed (suggested: your average speed while moving; empty:
    no limit). This is what taught paths were before Replay existed; such programs load and run unchanged.

  **Discard** throws the path away.
- **Press N** (MoveL points only, optional): replay the path pressing on the surface with this force. The folder
  then switches to Admittance, goes to 10 mm before the start, moves until contact, applies the force (along the
  tool's Z or straight down), follows the path, lets go, backs off 10 mm and goes back to the default mode. The force
  is limited by the profile's `max_force_n`.

A taught path runs as a normal program run, with the usual checks.

## Batch runs (PROGRAMS › Batch)

Runs the open program several times with one thing changed per run:

- **Nothing (repeat)**: the same program N times.
- **Vial profile**: each ticked vial profile becomes the fixture the program uses (Use as fixture, default `vial`)
  for one run. Afterwards the fixture is put back as it was before the batch (or removed again if there was none),
  also after STOP or a failure.
- **One step parameter**: pick a step, one of its numbers (e.g. Seconds of a Wait, Fz of a Force, Max mm/s of a
  MoveL, Times of a Loop) and a list of values (`2, 3, 4`). Each run uses a copy of the program with that value; the
  open program never changes. Every value is checked before the first run starts.

**Ask before each next run** shows "Next vial ready?" (or "Ready for the next run?") to the person in control
before every run after the first; Stop batch ends it there. **On failure**: stop, or go on with the next run. A batch
counts as one program run: nothing else moves, the vial cannot be changed and control cannot be taken over while it
runs, also during the question. STOP (and ■ Stop program or ■ Stop batch on PROGRAMS) ends the whole batch. The table lists
each run: what changed, the result, how long it took and its recording folder (when auto-record is on). The batch
also goes into the run history. **Save** keeps the batch next to the program as `<program>.batch.yaml` (not listed as
a program); **Load saved** reads it back.

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
  another waypoint (e.g. back to `up`). Teaching (Set waypoint here) still works.
* The run controls in the PROGRAMS toolbar (state chip, **Run**, **Pause**, **Stop**, and a **⋯** menu with Step, From selected and Loop) show the **loop iteration** ("Loop 9 · 3 / 37") and how many steps were skipped.
* **3D preview** beside the program: the TCP path of the whole program from where the robot is now, loops and
  relative moves included (kinematics from the URDF; no planning, no forces: Move until contact stops at the wall / floor of a
  vial of known size, else it is drawn to its full distance), and the waypoints.
* `programs/example_franka_scraping_baseline.yaml` is the Franka `sample_scraping` baseline (`scripts/scraping.py`)
  rebuilt with these steps: 37 angles 10° apart, preload, press Fx 5 N + Ty 2 Nm, scrape down, relax, up and orbit.
  Differences: relax torque 2 Nm (the profile limit, Franka used 3), Zero F/T before Admittance, no Franka pose-error
  compensation. Check the tool orientation of `approach` / `rim_start` and the Admittance compliant axes first.
* `programs/example_vial_scrape_speed.yaml` is that baseline with a set TCP speed on every straight move: to the rim
  15 mm/s, scrape down 5 mm/s, back up and orbit 15 mm/s, final lift 30 mm/s.
* `programs/example_rbf_scrape_speed.yaml` scrapes a round-bottom flask (fixture `flask`, saved in SETUP › Flask)
  like the vial: 24 strokes 15° apart all the way round, each following the round wall from 75° (as high as a
  straight tool reaches through its 40 mm neck) down to the bottom in 5° steps; per stroke: onto the wall, press 3 N, scrape
  down, release, out. The press is along the wall's normal at each point, so it follows the curve. Onto and down the
  wall 5 mm/s; into the neck, between strokes and out 15 mm/s. Its poses fit a flask with an 82 mm bulb and a 40 mm
  bore, and a 300 mm straight spatula; for another flask or tool, use Make program in SETUP › Flask.
  Until a flask is saved, the PROGRAMS preview shows it on the profile's example flask (`example_fixtures: flask`,
  500 mm out, 200 mm right of the example vial; preview only, never saved as the fixture).
* The examples are copied into the programs folder on first start. A newer shipped version replaces a copy nobody
  edited (`.examples.json` there keeps what was copied); an edited copy is kept.

Screenshots: `docs/ux/web_programs_scrape_baseline.png`, `docs/ux/web_programs_force_step.png`,
`docs/ux/web_programs_franka_baseline.png`, `docs/ux/web_programs_orbit_step.png`.

## Force limit (SETUP › F/T + payload)

A soft and a hard limit on |F| (N) and |T| (Nm) of the F/T reading as the GUI shows it (after the bias), kept in
`~/.config/moveit_gui/ft_limit.yaml` (`moveit_gui/ft_guard.py`). Off by default; only an Admin can change it.
Every F/T sample (about 500 Hz) is checked, on a 5-sample average so one noisy sample does not trip it.

* **Soft**: the chip on the OPERATE F/T card turns amber. Nothing stops.
* **Hard**: the chip turns red and the GUI's motion stops at once: the MoveIt motion is cancelled, the program
  stopped, the policy cancelled, the jog stopped and an applied force zeroed (an admittance controller stays on, so
  it gives way). Everyone gets a message. It fires once per event and re-arms when the reading is below the soft
  limit (or 80 % of the hard one) for 0.5 s, so you can move away from the contact.
* In freedrive it only warns: your hands are on the arm.
* With **Zero now** the reading at rest still holds the tool weight (about 10 N on the UR7e tool): set the limits
  above it, or use Zero all.
* It is a GUI guard for scraping contacts, not a safety function. The robot's own limits and the e-stop stay the
  safety.

## Match simulation (SETUP › F/T + payload)

## Policy imports (SETUP › Policies)

An imported checkpoint is copied into its own folder under `--policies-dir` (on the robot
`~/ur7e_data/policies`, mounted as /data/policies), together with its normaliser and available training/robot configuration. The manifest
uses relative paths, so the folder can move; renaming a policy moves the whole folder. The card command is optional
(`--policy-card-command`); without it the import remains usable when a normaliser is found and reports that no card
was made. A model without a normaliser must be explicitly marked **no normaliser (model has none)**. Admins can upload
a policy folder from the browser; supported files are checkpoint, normaliser and config formats up to 300 MB total,
and the uploaded folder is then an allowed import root.

On the robot computer start the GUI with:

```text
--policy-import-root /ws/scraping/policies
--policy-card-command "python3 /ws/scraping/export_policy_card.py"
--policy-run-command "python3 /ws/scraping/run_policy.py --model {model} --normalization {normalization} --config {training_config} --robot-config {robot_config} --robot-controller servo_admittance"
```

An imported policy needs a `robot_config.yaml` in its folder (or next to the checkpoint when importing); without
one the policy is listed but refuses to start.

A trained policy observes the F/T as `[fx, fy, fz, tx, ty, tz]` in the simulation's axes. When the sensor is mounted
turned (on the UR7e: scraping along base +X is now -Y of the sensor), Match simulation finds how the sensor's axes
map to the policy's, instead of hand-written sign tables (`moveit_gui/ft_match.py`, `web/ftmatch_cmds.py`).

* Needs control, Position mode (the robot holds still) and a live F/T reading. Nothing is commanded to move.
* Pick the policy whose sim reference to use and press **Start**. The reference is the manifest's
  `sim_ft_reference`, else `sim_ft_reference.json` next to its `robot_config` (made by `tools/sim_ft_reference.py`
  in the policy repo). Without one the card says so and does not start.
* For base +X, +Y and +Z in turn the 3D view shows an arrow at the spatula tip. Keep hands off the tool for a
  moment (the no-contact reading), then push the tip along the arrow, about 10 N, and hold 2 s. A push is taken by
  itself when it is over 5 N and steady for 1 s; **Record** takes it at once. **Again** repeats one push.
* The fit tries all 48 signed permutations M and picks the one with M s/|s| closest to the sim's r/|r| for the
  three pushes. It refuses a push under 5 N or more than 15° off M, and names the push to repeat. It also shows how
  far the best free rotation is from M. M is matched to the simulation, not to physics: MuJoCo reports the
  reaction, so its signs may be opposite to the push, and that is what the policy saw in training.
* Torque signs are not assumed to follow the forces (the FR3 needed `ft_obs_scale` [-1,-1,-1, 1,1,1]). A push at
  the tip also twists the sensor, so tx and ty get their sign from the same pushes: M t compared with the sim's
  torque (the pushes whose sim torque on that axis is at least 0.3 Nm). A torque change under 0.3 Nm, or pushes
  that disagree, are refused with the push to repeat. tz is not measured (a push through the tip hardly twists
  about z): it keeps the robot config's tz sign relative to its forces when they all share one sign, else it
  takes the tx / ty sign. The card says which.
* The result: `ft_obs_axes = M` and `ft_obs_scale = [1, 1, 1, s, s, s_z]` (forces 1: their signs are in M),
  explained in words ("policy fx = sensor fy", "policy tx = -sensor ty"), with each push (force and torque) as the
  policy sees it now, with the match, and in the simulation, and the torque check. A suggested spatula_tip
  orientation (tip x = the sensor axis that becomes policy x) is shown, never applied.
* **Save** (Admin) keeps the result in `config_dir()/ft_match/<robot>.json`, with the no-contact reading at the
  pose (raw sensor, reading and bias). It is one of each profile's settings, so it switches with control like
  gains and payload. **Remove** deletes it.
* Policy runs: with a saved result, each run gets an effective robot config, the manifest's `robot_config` with
  `observations.ft_obs_axes` and `ft_obs_scale` from the result, written to `config_dir()/policy_runs/` (the newest
  20 kept). Its path replaces the manifest's on the generated command line and in a hand-written command's
  `{robot_config}`. The log names the profile whose result was used. Without a result the manifest's file is used
  as it is and the policy's readiness list shows **F/T axes** as a warning (not a blocker).

## F/T bias (SETUP › F/T + payload)

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
- **● Record in the header** is the one-button way: it records everything useful for a run into one folder (joints, TCP
  pose via TF, F/T, gripper, controller states and the camera topics that are published, compressed only, the preset
  "Everything for a run"). It asks for an optional note; while recording the button reads **■ Stop** and the red pill shows
  the elapsed time and size. The folder also gets a short `meta.json`: start and stop time, who started it (the operator
  name), the program that was running, the control mode, the note, the topics. The ● Record button on DIAGNOSTICS still
  records only the topics ticked there.

## Flask Geometry and flask scrape (SETUP › Calibrate › Flask Geometry)

For scraping inside a round-bottom flask. The page fits the flask from touches, saves it, and makes a scrape program.

**Calibrate a flask** (Admin):

1. Make the TCP the centre of the ball tip (SETUP › TCPs) and set it active. Type the **tip radius** (mm) and the
   **wall thickness** (default 1.5 mm). Type the **neck inner diameter** if you measured it (caliper); otherwise it is
   the outer neck minus the wall.
2. Freedrive. Touch the flask from outside and press **Capture** for each point: **Rim** 3 or more, spread around the
   outside of the rim; **Bulb** 4 or more (6–10 is better), all around and down to the bottom, not only one side;
   **Neck side** (optional) on the outside of the neck. The bold line says where to touch next (the largest gap
   around the flask, and the height band with the fewest points); with a fit it is the orange dot in the 3D view.
   × on a point removes it.
3. The **Flask** card shows the fit: bulb outer diameter, inner radius (bulb minus the wall), neck bore, neck length,
   the closest typical size (50–1000 ml; a hint, not a check) and the RMS of each fit (how far the touches are from
   the fitted shape).
4. **Save flask** with a name. It becomes a **flask profile** and the fixture **`flask`** (origin at the bulb centre,
   Z up the neck, X close to base X), which scrape programs use. Profiles are kept per robot in
   `config_dir()/flasks/<profile>.yaml` and are part of a setup. **Use** makes a profile the fixture again (Operator
   too; refused while a program runs); **Delete** (Admin) keeps the fixture.

**Warnings** (Save asks before it saves with them):

- *The sphere centre is more than 2 mm off the neck axis*: the bulb and the rim do not line up. Usually a bad touch
  or bulb points on one side only. Add points on the other side, or remove the odd one.
- *The neck axis is tilted more than 3 degrees from vertical*: the flask is not upright in its holder, or a rim
  point is off. Straighten the flask or re-touch the rim.
- *A fitted surface has more than 1 mm RMS residual*: some touches are not on the surface (the tip slipped, the
  flask moved, a wrong tip radius). Remove the odd points or start over.
- *The fitted sizes are far from a common round-bottom flask size*: check the tip radius and the neck diameter; an
  unusual flask is fine.

**Flask scrape** (Operator, same page): choose the tool (**Straight**: shaft radius and length; **Bent**: also the
length and angle of the bent end; **Suggest bend** proposes a bend that passes the neck, fills it in and says which
polar band it reaches and whether that covers your polar range),
the TCP, the polar angles (0° = the bottom of the bulb), the points per stroke, the step around, the press depth
(how far past the inner wall the tip aims), an optional force (N, in the force mode) and the speed (mm/s, default
10, set on every linear move). The scrape is like the vial's: one stroke every step ° all the way round (360°), each
following the wall from the highest polar angle down to the lowest, so Z goes down along the stroke. A straight
tool's TCP X faces the wall at each angle and the force presses into the wall along its normal at each point (a
force step before each move: mostly along the tool high up, straight down at the bottom); a bent tool's end points
into the wall and the force is along +Z. **Pitch ° top / bottom** (straight tool): its tilt from vertical, toward
the wall, at the top and the bottom of each stroke (linear in between; one value: the same all the way). Empty =
automatic: the shaft passes through the middle of the neck (orange dot), which reaches highest; where that line
is blocked, through the middle of the neck off the axis (the robot then moves the tool sideways and up / down
through the neck, not only about one point). Only the spatula's end (up to 10 mm) may lie on the glass. Either way the neck
(margin kept at both ends) and the glass are checked, and Check reach shows the poses they block in red. A bent tool
pivots near the bottom of the neck. **Check reach** shows the polar band reached all the way round, how many poses are reachable and why the
others are not, and draws them green / red in the 3D view. **Make program** builds the program (approach above the
neck, Zero F/T there, the force mode, down to the pivot, then per stroke: down the shaft line to 5 mm short of the
wall, onto it, press, 0.5 s, scrape down, release, 0.25 s, back to the pivot; out) and opens it in PROGRAMS **unsaved**: review it, then save. It never
runs it.

**Straight-tool limit:** a straight tool must pass the neck, so it only reaches the lower part of the bulb. With
an 82 mm bulb, 50 mm neck, 3 mm shaft radius and 1 mm margin it reaches from the bottom up to 37° through a 24 mm
bore, 53° through 30 mm, 78° through 40 mm and 109° (past the widest point) through 60 mm: above that no straight
line through the neck reaches the wall (a shorter neck reaches a little higher). The rest of the
bulb needs a **bent tool**: its end goes in along the neck, then the tool tilts so the bent end reaches the side
of the bulb. Check reach with a bent tool takes a few seconds. On the simulated robot, **Fake touches** puts touches on a 250 ml
flask 5 cm below the TCP to try the page.

## Parts, camera calibration, setups, restart (SETUP)

The SETUP menu is in three groups, in the order a new user needs them: **Robot** (Setups, Parts, TCPs),
**Calibrate** (Camera calibration, F/T + Payload, Vial Geometry, Flask Geometry) and **Cell** (Drivers & cameras, Keep-out zones,
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
* A new tool changes the payload: redo SETUP › F/T + payload.

Screenshot: `docs/ux/web_setup_parts.png`.

**Camera calibration** (SETUP › Camera): hand-eye with a ChArUco board (7×5 squares, 35 / 26 mm,
DICT_5X5_100 by default; **Print board…** opens it at true size; print at 100 %, measure a square, enter it, *Save
board*). Eye-in-hand (camera on the tool, board still in the cell: gives flange → camera) or eye-to-hand (camera on
a stand, board on the tool: gives base → camera).

The page is four steps and the next step's button is the blue one (Start, Collect all…, Solve, Save + add to robot).
The board is one line (*7 × 5 squares of 35 mm …*); *Change the board* opens its fields.

1. Pick the camera and **Start**: intrinsics come from its CameraInfo (RealSense / ZED are factory calibrated).
   **End effector** and **Base** pick the frames, as in MoveIt's hand-eye plugin: any robot link or part (default:
   the planning link and the base). Eye-in-hand gives end effector → camera, eye-to-hand base → camera; planned
   poses are converted to the planning link for MoveIt.
2. **Live** (on by default) shows the camera about twice a second while the page is open, with the ArUco markers
   outlined (blue), the board corners (green) and its axes; a chip says *board found · N markers, M corners*,
   *not enough* or *no ArUco markers*. It pauses during Collect all. **Look** grabs one frame. Then either
   **Capture here** after each freedrive / jog (the count beside it says *at least 5, 12 recommended*), or **Plan 12 poses** (±tilt about the board, or about the tool tip for eye-to-hand)
   and **Collect all…**: MoveIt moves to each (collision-aware, GUI speed applies, STOP stops it) and samples
   where the board is seen.
3. **Solve**: OpenCV calibrateHandEye. **solver** picks which one is used (Park by default); the others are shown
   as a cross-check (*vs park*). Changing it solves again. The
   verdict comes from where the samples put the board: good ≤ 2 mm / 0.3°, usable ≤ 5 mm / 1°. Outlier samples are
   named; remove them with × and solve again.
   **Check in 3D**: the SETUP 3D view sits above the solve card and shows the camera's depth as a coloured point
   cloud (about 1–2 per second, int16 mm + RGB, zlib), placed with the result. A good calibration puts the table and
   parts where the robot model has them. Density fine / medium / coarse; the depth topic is found from the colour
   topic (RealSense `aligned_depth_to_color/image_raw`, ZED `depth/depth_registered`) or typed. **Adjust by hand**:
   x, y, z (mm) and roll, pitch, yaw (deg); the cloud moves as you type; **Reset** goes back to the solved pose;
   Save keeps the adjusted pose and records `edited_by_hand` (mm / deg from the solved one). **Open** on a saved
   camera loads it (its camera, mount and frames) to check and adjust it; saving it again keeps its samples.
4. **Save + add to robot**: kept in `cameras/<profile>.yaml` (with the samples) and added as a camera part. The part
   is the camera driver's own root frame (e.g. `camera_link` above `camera_color_optical_frame`), placed so the
   optical frame is where the calibration says. So the driver's TF tree hangs on the robot; nothing else may
   publish a parent for that frame. Editing the part's offset afterwards drops its *calibrated* mark.

MoveIt Calibration (the RViz plugin) is not released for Jazzy; this uses the same board type and OpenCV's solvers.
Screenshot: `docs/ux/web_setup_handeye.png`.

**Robot** (SETUP › Setups › **Hardware…**): one pop-up for the whole robot: **1. Robot arm** (Real or Fake, and the
robot's IP), **2. Gripper**, **3. Force/torque sensor** (and its IP), **4. ROS 2** (the domain ID, and **Lab network** or **This PC only**:
whether ROS 2 programs on other PCs see the robot's topics), **5. Cameras** (tick to add a RealSense, ZED or
USB camera; edit or remove them in SETUP › Cameras). The first time someone takes control on a PC where
nothing is set up yet, it opens by itself (*Later* asks again next time). What is not ready on this PC is greyed out
with what is missing (e.g. the Franka image not built on this PC, or a camera whose ROS package is not in the image).
The arms: **UR7e** (Robotiq 2F-85 and/or ATI F/T) and **Franka FR3** (Franka Hand and/or ATI F/T; the
`franka_scraping:jazzy` image with `~/franka_scraping_ws`, profile `fr3`, Position mode only). **Apply** (idle, Position) writes `config_dir()/robot` (arm, mode, robot IP, F/T IP, ROS domain, network) and the base (the
gripper + F/T sensor, see Base below), adds the cameras, and restarts the robot stack and the GUI when the arm, mode,
an IP, the ROS settings or the base changed (cameras alone need no restart). Fake starts no driver, so nothing is sent to a robot; Real
starts the driver for the robot at the IP, and nothing moves until you command it. The choice is kept for the next
starts. `ur7e_gui.sh --fake`, `--robot-ip` or `UR7E_ROS_DOMAIN_ID=<n>` win for that start only. The Robot box shows the
ROS domain and network in use. Your own scripts: docs/USERS.md. A GUI started by hand (not by `ur7e_gui.sh`)
never asks and cannot set up.

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
- **Name and options**: *Name* is free text shown everywhere (e.g. "D405", "ZED X Nano"); *Topic name* is the
  ROS namespace of its topics (a-z, 0-9, _; filled in from the name). A new name alone can be saved while it runs.
  Resolution and frame rate: RealSense 424 × 240 to 1280 × 720, 5 to 90 fps (1280 × 720: 30 at most); USB cameras
  320 × 240 to 1920 × 1080, 5 to 60 fps (the camera must have that mode); ZED: its grab resolution, 15 to 120 fps,
  and *Pictures sent* full, half or quarter size. A mode the camera does not have is refused on Save where known.
  The Set up pop-up adds a camera type only once; add a second camera of a type here, with its serial number.
- **In OPERATE**: the **Camera drivers** window lists the same drivers with Start / Stop (layout 3, Cameras, has
  it; add it to any layout in SETTINGS › Layouts › Edit). Camera windows show the cameras that publish first. Each camera window has a picture setting: Low (640 px,
  10 fps, the default), Med (960 px, 15 fps) or High (1280 px, 30 fps; more data over the network).
- Stop sends Ctrl-C (SIGINT), then SIGTERM after 8 s, then SIGKILL. A GUI-only restart leaves the drivers running
  (the new GUI takes them over); closing the GUI or restarting the robot stack stops them.
- Nothing here moves the robot. The command is `ros2 launch …` / `ros2 run …` without a shell; it is shown under
  the log. The list is `config_dir()/services/<profile>.yaml`, the logs `config_dir()/services/logs/` (newest 5
  per driver). Calibrated camera colour topics appear first in Camera calibration.
- The robot image needs the drivers (`ros-jazzy-realsense2-camera`, `ros-jazzy-usb-cam` in `docker/Dockerfile`)
  and the container needs the USB devices: `-v /dev:/dev --device-cgroup-rule='c 81:* rmw'
  --device-cgroup-rule='c 189:* rmw'` on the launcher's `docker run` (ur7e_gui.sh does this).
- **ZED** (Jetson only): the ZED SDK is built for Ubuntu 22.04, so the ZED driver runs in its own small image
  (ROS 2 Humble, `docker/zed/`), with the host's ZED SDK, CUDA and TensorRT mounted read-only. Build it once on the
  Jetson with `docker/zed/build.sh`. ur7e_gui.sh then starts the container `ur7e_robotiq_zed` next to the robot
  stack and stops it with the GUI. The GUI has no Docker access: it asks that container to start or stop the ZED
  driver through files in `~/ur7e_data/run/zed` (`scripts/zed_runner.py`, which runs only the ZED launch file).
  Positional tracking is off (the GUI does not use it). The first start with a neural depth mode optimises the
  model for the Jetson (several minutes, once).
- **Fake robot, real cameras**: `ur7e_gui.sh --fake` runs a simulated arm but starts real camera drivers, so you
  can set the cameras up without the robot. The simulated camera pictures are then left out.

## Cameras over the tailnet

Only one image topic is subscribed, and only while someone looks at a picture. It is sent as JPEG, by default up
to 640 px wide at 10 frames/s (a camera window's Med / High setting raises that). If two browsers watch different topics, the last one to open wins.

## Files

`moveit_gui/web/server.py` (HTTP, standard library only), `moveit_gui/web/service.py` (lease, commands, state;
tested without HTTP in `test/test_web.py`), `moveit_gui/web/workcell.py` (Parts, Camera calibration, Setups, restart; `test/test_workcell.py`), `moveit_gui/web/flask_cmds.py` (Flask Geometry, flask scrape; `test/test_flask_web.py`), `moveit_gui/web/static/` (the app: HTML, CSS, `app.js`, `workcell.js`, `drivers.js`, `flask.js`, `view3d.js`, no build
step). Logic without the web: `moveit_gui/parts.py`, `flask_setup.py`, `flask_geometry.py`, `flask_scrape.py`, `handeye.py`, `setups.py`, `services.py` (drivers; `test/test_services.py`). Screenshots: `docs/ux/web_*.png` (`tools/web_screenshots.py`).

## Run history and activity log (DIAGNOSTICS)

**Run history** lists every program and policy run, newest first: start time, program name, who had control when it
started, length, steps done of total, and how it ended (finished, failed with the reason, or stopped by whom). The
list can be filtered by program name. Click a row for the details: the peak force and torque during the run (from the
F/T sensor, sampled at 10 Hz), the linked recording folder if one ran, the policy log folder, and the log lines the
run produced (the last 100). It is kept in `~/.config/moveit_gui/history/<profile>.jsonl`, one JSON line per run; the
last 500 runs are kept and older ones are dropped.

**Activity** says who did what and when, for everyone using the GUI: taking, taking over, giving, keeping, releasing
and losing control, going away, STOP presses, program and policy run / stop, recording start / stop, mode switches,
power and brake actions, setup changes (every Admin command) and saved roles. Each line has the time, the name typed
in the browser, the Tailscale login or IP, the role, what happened and a few details. It can be filtered by person and
by kind, updates by itself, and nobody can edit it from the GUI. It is kept in
`~/.config/moveit_gui/activity/activity.jsonl`; at 2 MB the file is renamed to `activity.jsonl.1` (the old `.1` is
dropped). No control tokens or browser ids are written, only the public user id. A STOP or a recording stop from a
browser without control is named by its typed name (STOP) or as "someone without control" (recording stop).

* **SETTINGS** (next to SETUP) holds the GUI itself: Layouts, Profiles and Roles. Profiles lists the profiles, makes a new one,
  deletes an empty one, and copies another profile's screen layout into yours (the page reloads). The profile picker only
  picks or creates a profile. SETUP keeps the robot: Setups, Parts, TCPs; Calibration (Camera, F/T + payload, Vial, Flask);
  Cell (Cameras, Keep-out zones, Gains).
* The named pose "home" is now "up". Programs that say "home" still work (either name finds the pose saved under the other).
