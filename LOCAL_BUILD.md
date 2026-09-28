# EZ21 Local Build

Workspace: `/mnt/nvme0n1/autoware.ez21`
Branch: `ez21` (root commit `8707e1e`).
Platform: Ubuntu 22.04 ARM64, ROS 2 Humble, Jetson CUDA 12.6, TensorRT 10.3.

## Environment

```bash
source /mnt/nvme0n1/autoware.ez21/setup_env.bash
ros2 pkg prefix autoware_launch
```

The prefix should point inside this workspace. The existing shell startup file
still loads the older workspace; source the file above to select this build.
The environment file includes the CUPTI library needed by the installed PyTorch
wheel and the workspace-local spconv library.

## Rebuild

```bash
/mnt/nvme0n1/autoware.ez21/build_ez21.bash
```

The build script starts with a clean environment to avoid linking against the
older workspace. It uses four concurrent packages, two compiler jobs per package,
Release mode, and disables build-time tests. Exact source revisions are recorded
in `source-versions.repos`.

## Simulation

```bash
source /mnt/nvme0n1/autoware.ez21/setup_env.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/mnt/nvme0n1/autoware.ez21/src/launcher/autoware_launch/ui/generated_maps/autoware_init_map \
  vehicle_model:=ez21_vehicle sensor_model:=ez21_sensor_kit rviz:=true
```

The simulator needs an initial pose and route before planning can run.
Generated maps are local artifacts and are not included in this source snapshot.
Supply a map and initial pose matching the intended scenario for driving tests.

## Validation And Changes

- All 477 discovered packages have successful `colcon_build.rc` results.
- Clean-environment rosdep check passes.
- CUDA runtime detects one GPU; the spconv shared library loads successfully.
- EZ21 vehicle interface starts with `enable_can_io:=false`.
- Smart MPC enters `rclpy.spin` and runs until the smoke-test timeout.
- The Robosense M1 launch file parses with `--show-args`.
- Planning simulation starts map, planning, control, and simulator components.
- Publishing an initial pose produces `/localization/kinematic_state` odometry.
- Simulation startup logged one emergency-stop cancellation timeout while
  components initialized; no component load failure was found before shutdown.
- Smoke tests use isolated ROS domains 183/184 and do not command vehicle CAN.
- Source fix: compile the Yabloc fallback segmenter only when OpenCV ximgproc
  segmentation is unavailable, avoiding an unused-function build error.
- Repaired interrupted system package configuration and reinstalled
  `ros-humble-parameter-traits`; installed geographic ROS dependencies and CUDA /
  TensorRT development packages.
- cumm/spconv Jetson packages are extracted under `.deps/usr/local`, leaving the
  system CUDA version unchanged. These optional pretrained perception pipelines
  have been compiled but not validated with model files or live sensors.

Relevant logs: `build-pass3.log`, `build-spconv.log`, `build-final-check.log`,
`vehicle-smoke.log`, `smart-mpc-smoke.log`, and `simulation-smoke-final.log`.
Timeout-driven smoke-test shutdown can emit KeyboardInterrupt or shutdown errors.
Full driving behavior, vehicle calibration, and physical sensors have not been
validated by these build and startup checks.
