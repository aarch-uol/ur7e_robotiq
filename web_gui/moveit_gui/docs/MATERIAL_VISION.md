# Material vision

In a shell inside the robot container, with its ROS environment sourced:

```bash
python3 /ws/scraping/vision/run_vision.py --ros-args --params-file /ws/scraping/vision/vision/config/ur7e_d405.yaml
```

The D405 camera topics and TF to `base_link` must be available. The wrapper resolves its modules and weights relative to the bundle and uses a writable temporary directory for Ultralytics.

Drivers & cameras currently has `realsense`, `usb_cam`, `zed`, `launch`, and `run` service types. `run` executes `ros2 run <package> <executable>`; it cannot execute an arbitrary Python script. The current script bundle has no applicable services entry. Start it in the container shell. An installed ROS package or launch wrapper would be needed before adding it as a service; no robot services file is changed here.

The vision node publishes `/scraping_vision/world_midpoints`. The training config's `observation.cluster_mode` determines the policy's observation contract. Older cluster policies read these midpoints; fixed band policies need `/scraping_vision/fixed_band_targets`, or runner-side `band_sorting.enabled: true` with a measured finite `vial_bottom_z` and valid band geometry in the robot config. Select the checkpoint with its matching training config, rather than changing the observation mode of an existing trained policy. The supplied node has no mode parameter to switch between those contracts.

## Fixed band targets

The node can publish `/scraping_vision/fixed_band_targets` itself (the patched node in `scraping_vision/`: copy
`fixed_bands.py` next to `ros_integration.py` and apply `main.patch` and `ros_integration.patch`). Give it the
policy's robot config:

```bash
python3 /ws/scraping/vision/run_vision.py --ros-args --params-file /ws/scraping/vision/vision/config/ur7e_d405.yaml \
  -p fixed_bands_robot_config:=/ws/scraping/policies/round44/robot_config.yaml \
  -p fixed_bands_only:=true -p vial_cache_sec:=5.0
```

- It publishes 16 floats: the remaining material per height band (1 = the amount at the episode start) and, per
  band, the tool tip's height above the band's material (m; 0 for an empty band). The banding is the policy
  runner's own `band_sorting.py`, so the numbers match what the policy saw in training.
- Every material pixel inside the vial crop counts, weighted by the area it covers, not the k-means midpoints.
- It reads `band_sorting.vial_bottom_z` (must be measured: base_link z of the vial's inside floor, tip on the
  floor minus 0.002), `band_sorting.band_edges_m` (or the training config's surface bank),
  `servo_admittance.base_frame` / `ee_frame` (the tip height comes from TF, so the TCP must be in TF) and
  `topics.fixed_band_targets`. It refuses to start the topic if the config sets observation position biases.
- Episode start: when the `mujoco_policy_runner` node appears (or on `~/reset_fixed_bands`), the material total
  is the median of the next 5 frames. Nothing is published before that.
- `fixed_bands_only:=true` skips clustering, k-means and the midpoint topics, and `vial_cache_sec` reuses the
  YOLO box and GrabCut mask (found again in the background). On the robot computer's CPU this runs at about
  13 Hz (the full pipeline is about 1.6 s per frame), which the runner needs: it treats the topic as stale after
  0.5 s. Without `fixed_bands_only` the midpoint topics are published too, but much too slowly for fixed bands.
- `band_sorting.enabled` stays `false`: that is the runner's own fallback from the midpoints.
