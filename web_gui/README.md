# Web GUI (branch `web-gui`)

This branch is `main` (the robot stack) plus the web GUI for it, in `web_gui/moveit_gui`. `main` stays the plain
stack for everyone else.

`web_gui/moveit_gui` is a copy of [mrceki/MoveIt-GUI](https://github.com/mrceki/MoveIt-GUI), branch
`v2-ur-jazzy`, commit `fb317c1` (maintainer email set to the university address here). Development happens there;
this folder is refreshed from it.

`web_gui/COLCON_IGNORE` keeps the GUI out of the robot image build (its rosdep and colcon steps): the launcher builds
the GUI itself, in `~/ws`, inside the image.

## Set up on a new PC

```bash
git clone -b web-gui git@github.com:aarch-uol/ur7e_robotiq.git ~/ur7e_robotiq
mkdir -p ~/ws/src
cp -r ~/ur7e_robotiq/web_gui/moveit_gui ~/ws/src/moveit_gui     # a copy, not a symlink: ~/ws is mounted alone
git clone https://github.com/AndrejOrsula/pymoveit2.git ~/ws/src/pymoveit2
cp ~/ws/src/moveit_gui/scripts/ur7e_gui.sh ~/ur7e_gui.sh
~/ur7e_gui.sh --fake       # try it without a robot first; then ~/ur7e_gui.sh for the real one
```

Everything else (Tailscale, robot IP, first start, bases, setups) is in
[`moveit_gui/docs/SETUP_WEB.md`](moveit_gui/docs/SETUP_WEB.md); the GUI itself in
[`moveit_gui/docs/WEB.md`](moveit_gui/docs/WEB.md). To update, copy the folder again after pulling this branch.
