# Robot (teach pendant) setup

Do this once. It's written for a UR7e on PolyScope 5. Menu names can differ slightly between PolyScope versions.

**Save the installation** after you change anything under the *Installation* tab (top bar → **Save** → **Save Installation**). Otherwise the settings are lost at the next reboot.

---

## 1. Network

1. **Settings → System → Network**: give the robot a **static IP**.
2. Give the ROS PC's **wired** Ethernet an address on the same subnet.
3. From the PC, check that it works: `ping` the robot's IP.

## 2. URCaps

**Settings → System → URCaps.** You need exactly these two, and the Robotiq one must be off:

| URCap | State | Why |
|---|---|---|
| **External Control** | installed, active | Lets ROS drive the arm. Download `externalcontrol-x.x.x.jar` from [Universal_Robots_ExternalControl_URCap releases](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases). |
| **RS485** (`rs485-1.0.urcap`) | installed, active | Exposes the wrist's RS-485 port to the network on TCP port **54321**. The gripper driver connects to this. From [Universal_Robots_ToolComm_Forwarder_URCap](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap/releases). |
| **Robotiq Grippers** | **disabled or removed** | ⚠️ It talks to the gripper by itself and fights with ROS over the same wire. This makes the gripper link drop constantly. |

To install a URCap: copy it to a USB stick, then **+** → pick the file → **Restart**.
To disable one: select it → click **Deactivate** (the button then reads *Activate*) → **Restart**.

> **Restart the robot after any URCap change.** Changes don't take effect until you do.

## 3. External Control settings

**Installation → URCaps → External Control:**

| Field | Value |
|---|---|
| Host IP | the **ROS PC's** IP, **not** the robot's |
| Custom port | `50002` (default) |
| Host name | anything |

## 4. Tool I/O (the gripper port)

**Installation → General → Tool I/O:**

| Setting | Value |
|---|---|
| Controlled by | **User** (⚠️ not *Robotiq_Grippers* or any URCap) |
| Tool Communication Interface | **enabled** |
| Baud rate | **115200** |
| Parity | **None** |
| Stop bits | **One** |
| RX idle chars | **1.5** |
| TX idle chars | **3.5** |
| Tool output voltage | **24 V** |

ROS doesn't change these settings. The robot uses whatever is saved here.

## 5. The program you press Play on

1. **Program → New Program**.
2. Add **one** node: **URCaps → External Control**. Nothing else. ⚠️ There must be no Robotiq *Gripper* / *Gripper Activate* nodes.
3. Save it, for example as `ros_control.urp`.
4. Optional: **Installation → General → Startup → Default program** → `ros_control.urp`, so it loads at boot.

## 6. Services (only if your PolyScope has this menu)

**Settings → Security → Services**: enable **Dashboard Server**, **Primary Client Interface** and **RTDE**.

---

## Checklist

- [ ] PC can `ping` the robot
- [ ] External Control URCap active, Host IP = PC IP
- [ ] RS485 URCap active
- [ ] Robotiq Grippers URCap **disabled** (and robot restarted afterwards)
- [ ] Tool I/O controlled by **User**, 115200 / None / One / 1.5 / 3.5, 24 V
- [ ] Program contains only **External Control**
- [ ] Installation saved

## Every session

1. Power on the robot and release the brakes (**ON → START**, the arm clicks).
2. Load `ros_control.urp`.
3. Start `moveit.launch.py` on the PC. **Then** press **▶ Play**.

If you restart the launch, press **Play** again. The program has to reconnect to the new driver.
