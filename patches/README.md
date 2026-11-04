# Patches

`scripts/setup_workspace.sh` clones three third-party repos at pinned commits and applies these patches with `git apply` (idempotent: skipped if already applied).

## robotiq_driver_pty_transport.patch  (robotiq/ros @ 8d7b841)

The Robotiq driver opens its serial port through libserialport, which rejects pseudo-terminals (`/dev/pts/*`). On this robot the gripper sits on the UR wrist RS-485 port, exposed by the official UR tool-communication forwarder as the PTY `/tmp/ttyUR`. The patch adds `posix_serial.{hpp,cpp}` (plain POSIX termios) and makes `hardware_interface.cpp` build the `Robotiq::Gripper` on that transport instead of libserialport. Without it the gripper cannot connect.

## net_ft_gamma_runtime.patch  (gbartyzel/ros2_net_ft_driver @ dbe9916)

On activation the driver clears the sensor's software bias and treated a failure as fatal (`ERROR`), which aborted activation of the ATI Gamma. The patch only calls `clear_bias()` when hardware biasing is requested, and downgrades a failure there from FATAL/ERROR to a warning so the sensor still activates.

## cartesian_impedance_controller_ur6dof.patch  (mrceki/Cartesian-Impedance-Controller @ 0a1c635)

The upstream controller is written for 7-DOF arms. The patch (1) replaces the hard-coded 7x7 identity in the null-space torque with `n_joints_` so the UR's 6 joints work, and (2) on activation, validates all command and state handles and initializes the equilibrium pose and null-space configuration from the measured state (after `read_state_from_hardware()`, and writing zero torque first) instead of calling `update()` against a zero reference, which could cause a large torque transient on a real arm.

## Items for the owner to review (not changed)

- `robotiq_driver_pty_transport.patch` adds two new files (`posix_serial.hpp`, `posix_serial.cpp`) with the header "Copyright (c) 2026 Robotiq, Inc. ... Licensed under the BSD-3-Clause license; see LICENSE for details." These files were written for this repo, not by Robotiq, so the copyright holder is almost certainly wrong, and the licence claim is unverified. The owner should decide the correct holder/licence.
- This repo itself has no LICENSE file, while the package.xml files declare BSD-3-Clause (`ur7e_robotiq_bringup`, `ur7e_safety`) and MIT (`topic_sensor_bridge`, placeholder maintainer `user@example.com`).
