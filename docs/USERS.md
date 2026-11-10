# Using the UR7e

## 1. The GUI

Open **http://X.X.X.38:XXXX/** in a browser.

* Press **Take control** (top right) before you command anything, and **Release** when you are done.
  One person controls the robot at a time. Everyone else can watch.
* **STOP** (top right, or Esc) works from every browser. It is a software stop: the red button on the pendant
  is the emergency stop.
* Check that nobody is near the robot before you move it.
* The first time, pick your **profile** (your name, or *New profile…*). It keeps your own settings (TCPs,
  keep-out zones, fixtures, vials, gains, F/T bias, limit and payload, parts, camera calibrations), your setups,
  programs and screen layout. Change it with your name at the top right (*Change my name*).
* The robot runs the settings of the person in control. When you take control and someone else's settings are in
  use, the GUI asks to switch to yours and restarts (about a minute). Their settings stay in their profile. The first
  time, you choose what yours start from: a copy of the ones in use, or a setup. SETUP › Setups shows whose
  settings are in use.
* Nobody can change another person's settings, setups or programs. Copy them into your profile instead.
* Nobody using the robot? SETUP › Setups › Robot › **Stop robot stack…** stops everything. The address then shows
  a page with **Start robot stack** (about a minute).

## 2. Your own setup (camera holder, camera)

Each person keeps their own **setup**: the camera holder in the robot model, the camera calibration, TCPs and the
rest. The gripper and the F/T sensor stay the same for everyone.

**Make yours once:**
1. Load the setup that is closest (SETUP › Setups › Load…), or start from the one in use.
2. SETUP › Parts › **Import mesh…**: your camera holder (.stl, .obj, .dae), on the tool. Press **Restart robot
   stack** (banner) so MoveIt plans around it.
3. SETUP › Cameras: add your camera if it is not there yet, and **Start** it.
4. SETUP › Camera: pick the camera, *eye-in-hand*, **Start**, **Plan poses**, **Collect all…**,
   **Solve**, **Save + add to robot**.
5. SETUP › Setups: type a name (e.g. `my_d405`) and **Save**.

Setups are saved in the profile shown above the list. "Shared" holds the ones saved before profiles.

**Copy someone's setup:** SETUP › Setups › **Copy from another profile…**. You get your own copy. To copy a
layout: SETTINGS › Profiles › **Copy a layout**.

**Switch:** SETUP › Setups › your setup › **Load…**. The robot stack restarts (about a minute) with your holder and
calibration. Then start your camera in OPERATE › Camera drivers.

**Changed something?** The top of SETUP › Setups says what changed. Press **Save into "<name>"** to keep it.

## 3. Your own Python / ROS 2 scripts

The robot's topics are on **ROS domain 42**. The GUI shows the number in SETUP › Setups › Robot.

### From your own PC on the lab network

You need ROS 2 Jazzy (Ubuntu 24.04, or the `osrf/ros:jazzy-desktop` Docker image).

```bash
export ROS_DOMAIN_ID=42          # in this terminal only, not in ~/.bashrc
ros2 topic list
python3 my_script.py
```

This only works when the robot is set to **Lab network** (see 4).

### On the Jetson

The Jetson has no ROS 2 of its own. `ros_shell.sh` opens one in a container, with the right domain already set.
It needs a Jetson account in the `docker` group.

```bash
cd ~/my_project                                   # this folder is /work inside
~/ws/src/moveit_gui/scripts/ros_shell.sh          # a shell: ros2 topic list, python3 my_script.py
~/ws/src/moveit_gui/scripts/ros_shell.sh my_script.py   # or run one script
```

It has ROS 2 Jazzy, the robot's message packages and pymoveit2.

### Example: read the joints and the F/T sensor

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

rclpy.init()
node = Node("my_reader")
node.create_subscription(JointState, "/joint_states", lambda m: print(m.position), 10)
node.create_subscription(WrenchStamped, "/ft/force_torque_sensor_broadcaster/wrench",
                         lambda m: print(m.wrench.force), 10)
rclpy.spin(node)
```

### Scripts that move the robot

Don't send motion from a script while someone uses the GUI. Run it as a **policy**: the GUI starts it from
AUTOMATION, checks the robot is in the right mode first, and stops it with STOP. See
`policies/_template.yaml.example` and docs/WEB.md (AUTOMATION).

## 4. Network setting

In SETUP › Setups › Robot › **Hardware…**, under **ROS 2**:

* **Domain ID**: the number your scripts use (42 by default).
* **Lab network**: ROS 2 programs on other lab PCs see the robot's topics.
* **This PC only**: only programs on the Jetson (`ros_shell.sh`) see them. Other PCs on the network can't read
  or send anything over ROS 2.

Apply restarts the robot stack (about a minute). The GUI in the browser works in both settings.
