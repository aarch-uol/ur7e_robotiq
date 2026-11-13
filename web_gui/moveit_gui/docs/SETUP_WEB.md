# Web GUI on a new PC (UR7e + Robotiq 2F-85 + ATI Gamma)

The web GUI runs inside the `ur7e_robotiq:jazzy` Docker image next to the robot stack. You open it in a browser
on this PC, or on any device on your Tailscale network. Nothing is installed on the host apart from Docker, git
and (optionally) Tailscale.

```
~/ur7e_robotiq   robot stack: drivers, controllers, MoveIt config, Dockerfile   (aarch-uol/ur7e_robotiq, main)
~/ws/src/        GUI workspace, built inside the image by the launcher
  moveit_gui     this repo, branch v2-ur-jazzy
  pymoveit2      upstream
~/ur7e_data      everything you save: config (TCPs, vials, F/T bias, payload, gains), programs, policies, logs
```

## 1. Host

* Ubuntu 22.04 / 24.04 with Docker. Add yourself to the `docker` group (`sudo usermod -aG docker $USER`, then log
  in again) and check `docker run --rm hello-world`.
* git, and GitHub access to `aarch-uol/ur7e_robotiq`.
* The PC on the robot network (the UR at `10.6.203.101` by default) with a fixed address the robot can reach
  back: the UR's External Control URCap connects to this PC.
* Optional: [Tailscale](https://tailscale.com/download) (`tailscale up`). The GUI then listens on this PC's
  Tailscale address, so devices on your tailnet can open it. Without Tailscale it listens on 127.0.0.1 only, so
  only this PC's browser can open it. `ur7e_gui.sh --lan` (or `UR7E_GUI_LAN=1`) also listens on this PC's lab
  network address, so anyone on that network can open it. The GUI never listens on all interfaces (0.0.0.0). Don't
  expose it to the internet (no `tailscale funnel`, no port forwarding): it can move the robot.
* A desktop PC (amd64) or an NVIDIA Jetson (arm64, JetPack 6, tested on an Orin NX with JetPack 6.2.1). The
  launcher picks the image for the CPU: `ur7e_robotiq:jazzy` on a desktop, `ur7e_robotiq:jazzy-arm64` on a
  Jetson. The Jetson image is built on the Jetson the first time (slow: about an hour).

**Check a PC:** `python3 ~/ws/src/moveit_gui/scripts/pc_setup.py doctor` lists what is fine, what to look at
(warn) and what is missing (FAIL), each with a hint. It changes nothing. `--robot-ip` sets the robot to look for;
`--json` gives the same list for scripts.

## 2. Get the code

```bash
git clone git@github.com:aarch-uol/ur7e_robotiq.git ~/ur7e_robotiq
mkdir -p ~/ws/src && cd ~/ws/src
git clone -b v2-ur-jazzy https://github.com/mrceki/MoveIt-GUI.git moveit_gui
git clone https://github.com/AndrejOrsula/pymoveit2.git
cp ~/ws/src/moveit_gui/scripts/ur7e_gui.sh ~/ur7e_gui.sh     # the launcher uses $HOME paths
```

## 3. Try it without a robot

```bash
~/ur7e_gui.sh --fake
```

The first run builds the image (`~/ur7e_robotiq/docker/build.sh`). That takes a while. The image holds only what
the robot packages need; the launcher rebuilds it by itself only when those dependencies change in `~/ur7e_robotiq`
(`docker/`, `scripts/setup_workspace.sh`, `patches/`, a `package.xml`: `docker/deps_hash.sh`). The packages
themselves are built from `~/ur7e_robotiq` at each start into `~/ur7e_data/overlay` (seconds; log:
`~/ur7e_data/logs/overlay_build.log`), so a changed launch file, yaml or script just needs a restart. Then it builds
`~/ws` inside the image (log: `~/ur7e_data/logs/ws_build.log`) and prints the address, e.g. `http://100.x.y.z:8180/`. In the browser press
**Take control** (top right) to command anything; other browsers watch until you release. Ctrl-C in the terminal
stops everything.

The server accepts the `Host` header only as an IP address, `localhost`, a short name or `*.ts.net`. If you reach it
by another DNS name (a lab domain name, a CNAME), add `--allow-host robot.lab.example.org` (repeatable) to the
`moveit_gui_web` command line, else the page answers "unknown Host header" (403). `0.0.0.0` is refused as `--host`.

## 4. Real robot

1. On the pendant: the robot is in Remote control and the External Control URCap program points at this PC.
   Power on and brake release: on the pendant, or in the GUI (OPERATE) from any browser that has control, after
   confirming you can see the robot (`MOVEIT_GUI_LOCAL_POWER_ONLY=1 ur7e_gui.sh` limits them to a browser on this PC).
2. Start the GUI:
   ```bash
   ~/ur7e_gui.sh                                  # web GUI, robot 10.6.203.101
   ~/ur7e_gui.sh --robot-ip <ip>                  # another robot
   ~/ur7e_gui.sh -- headless_mode:=true           # extra gui.launch.py arguments after --
   ```
   Or set it up in the browser: the first time, a pop-up asks what this PC runs (arm, Real or Fake and the IP,
   gripper, F/T sensor and its IP, cameras); later SETUP › Setups › Robot › **Hardware…**. The launcher remembers
   it, so a plain `~/ur7e_gui.sh` starts the same way next time.
   Franka FR3 instead: build its image once (`aarch-uol/fr3_scraping_ws`: `cd docker; docker compose build`, the
   workspace in `~/franka_scraping_ws` built), then pick Franka FR3 in the pop-up. In Franka Desk unlock the joints
   and activate FCI first. The launcher starts its MoveIt (position controller) and the Franka Hand node.
   It waits for the controllers, then starts the GUI. Unless you passed `headless_mode:=true`, load and PLAY the
   External Control program on the pendant.
3. The stack runs on `ROS_DOMAIN_ID=42`, so it doesn't clash with other ROS PCs on the lab network (default
   domain 0). Anything that must talk to it (a policy runner, the `ros2` CLI) needs `export ROS_DOMAIN_ID=42`
   in that shell (not in `~/.bashrc`). Change the domain in Hardware… (ROS 2), or with
   `UR7E_ROS_DOMAIN_ID=<n> ~/ur7e_gui.sh` for one start. **This PC only** there keeps the topics off the
   network. On the PC itself, `scripts/ros_shell.sh` gives a ROS 2 shell on the right domain (docs/USERS.md).
4. Gripper missing in OPERATE: the gripper is powered from the tool flange, so if the stack started while the arm
   was off, power the arm on and press **Reconnect** next to the gripper.

## Start with the PC

To have the GUI come up by itself when the PC starts (set the robot up once in the GUI first):

```bash
mkdir -p ~/.config/systemd/user
cp ~/ws/src/moveit_gui/scripts/ur7e_gui.service ~/.config/systemd/user/
systemctl --user daemon-reload && systemctl --user enable --now ur7e_gui
sudo loginctl enable-linger $USER      # start at boot, before anyone logs in
```

On the lab network too (`--lan`): `systemctl --user edit ur7e_gui`, add `[Service]` and
`Environment=UR7E_GUI_LAN=1`, save, then `systemctl --user restart ur7e_gui`.

It runs `~/ur7e_gui.sh` with what Hardware… saved, and restarts it if it fails. On a real robot it starts the driver,
but nothing moves until someone commands it. Log: `journalctl --user -u ur7e_gui -f`. Stop: `systemctl --user stop
ur7e_gui`. Turn it off: `systemctl --user disable --now ur7e_gui`. While it runs, don't start `~/ur7e_gui.sh` in a
terminal too.

## Cameras

Plug the cameras in (RealSense on USB, ZED X on the Jetson's GMSL port), then in SETUP › Cameras press
**+ Add…**, pick the camera, give it a name and save. **Start** / **Stop** run its driver; tick **start with the
GUI** to start it every time. The picture and the topics show when you select it. This works with `--fake` too.
In OPERATE, layout 3 (Cameras) has a **Camera drivers** window with the same Start / Stop buttons.

A ZED needs a Jetson with the ZED SDK installed and its driver image, built once:

```bash
~/ws/src/moveit_gui/docker/zed/build.sh
```

The launcher then starts the ZED container by itself.

## 5. First-time setup in the GUI (SETUP page)

Do these once per tool / workcell. Everything is saved under `~/ur7e_data/config/moveit_gui` and restored on the
next start.

0. **Base** (SETUP › Setups): check that it matches what is on the flange (F/T sensor + gripper, gripper, F/T
   sensor, or bare arm). **Switch…** restarts the robot stack with the other base.
1. **Parts**: add what is bolted on (spatula, camera bracket, table; or a CAD mesh: .stl, .obj, .dae) so MoveIt
   plans around it. Pick each part's collision (convex hull for meshes, or none for show-only pieces) and tick
   **Collision** in the 3D view to see what MoveIt sees. Then **TCP at its
   tip…** on the tool part and make that TCP active in the top bar. Parts work at once; **Restart robot stack**
   (banner) also writes them into the robot model. Or **TCPs**: add the tool tip as a plain virtual TCP (e.g.
   `spatula_tip` = `robotiq_tcp` + 150 mm in Z).
2. **F/T + Payload**: with the tool on and nothing touching it, press **Zero, keep tool weight** (F/T bias), collect the
   calibration poses (one at a time, or Collect all poses), Evaluate, then **Apply to admittance**. The
   **Saved** block shows what is kept: the payload restored before every switch to Admittance, and the bias
   restored after a stack restart.
3. **Vial Geometry**: touch the rim (3 points, optionally the bottom) and save. Or pick a vial in **Vial
   profiles** (built-in `baseline` = scraping.py's position; save your own vials there and switch with Use).
4. **Keep-out zones**: add the table first (or make it a part).
5. **Camera calibration** (if a camera is used): print the board, Start, Plan 12 poses, Collect all, Solve, Save +
   add to robot.
6. **Setups**: save the whole configuration under a name (it remembers the base too). When you change the tool,
   save a new one; Load switches back, base included (the GUI, and the stack if needed, restart by themselves).

## 6. Move to another PC / back up

Copy `~/ur7e_data` (TCPs, parts, camera calibrations, setups, vials, F/T bias, payload, gains, programs, policies). The folder is created by the
container, so some files belong to root: `sudo cp -a ~/ur7e_data <dest>`.

## 7. Update

```bash
cd ~/ur7e_robotiq && git pull          # built on the next start (the image only if its dependencies changed)
cd ~/ws/src/moveit_gui && git pull     # rebuilt in the image on every start
```

Example programs are copied into `~/ur7e_data/programs` only when missing. To get a newer example, copy it over
yours from `~/ws/src/moveit_gui/programs/`.

## Trouble

* **403 "unknown Host header"**: you reached the GUI by a DNS name it does not know: add `--allow-host NAME`.
* **Port busy**: `--port <n>`. Find the old process with `ss -ltnp | grep 8180` and stop it by PID.
* **A container called ur7e_robotiq is already running**: another terminal has the stack. `docker stop ur7e_robotiq`.
* **Restart from the GUI does nothing**: only a GUI started by `ur7e_gui.sh` can restart (it exits with code 75 / 76
  and the launcher starts it again). A refusal says why (robot busy).
* **Robot stack fails after adding a part**: the launch log shows the xacro error. Fix or delete the part in SETUP ›
  Parts (in a fake GUI: `~/ur7e_gui.sh --fake`), or move `~/ur7e_data/config/moveit_gui/parts/generated` away.
* **Old UI in the browser after an update**: a banner offers a reload. Otherwise reload the page.
* **More**: `docs/WEB.md` (every page and button), `README.md` (profiles, programs, architecture).
