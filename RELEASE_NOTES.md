# Scout Release Notes

## scout-v0.3.0-ht-rc.1 (2026-10-23)

Branch: `feature/scout-ht-directional-traversability`.
Adds observation-only costmap-first, robot-to-cell directional HT fusion,
strict map/TF/timestamp checks, diagnostics, an independent perception launcher,
RViz configuration and the implementation plan/startup guide.
It does not change the current vehicle's planner/control inputs.
59 Python tests and the real ELE HT model's isolated synthetic fusion probe passed.

ELE remains a separate source repository at
`git@github.com:Unmanned-Systems-Lab/ele_scouts_kit.git`.
The companion branch is `feature/autoware-ht-integration-2026-10-23`, candidate
tag `ele-scout-ht-v0.1.0-rc.1`, commit
`d94a4ba60dc8743cc857b1d98d2ca5f873105a61`; 47 focused ELE tests passed.
Its exact revision is recorded in `repositories/scout-external.repos`; the
commit must be published there and readable before `vcs import` can retrieve it.
No ELE source, checkpoint or local build artifacts are vendored into Autoware.

## scout-v0.2.0-rc.1 (2026-10-23)

Source branch: `release/scout-fixposition-2026-10-23`.
This annotated tag records a development candidate, not a validated autonomous
driving release. Existing `main` and `ez21-fixposition` branches are preserved.

### Included

- Fixposition-aware UI localization and map-origin checks; the integrated UI
  receives the actual launch map path instead of assuming a different directory.
- Online selection/generation and activation of maps aligned with live
  `FP_ENU0`, including runtime projection updates, map-loader refresh, route
  clearing and planner state reset. Switching requires a stationary vehicle in
  STOP with Autoware control disabled; failed switches block route/control actions.
- Route starts from current localization, not the first UI map point. Generated
  lanes extend behind the vehicle by 5 m when the alignment checks succeed.
- Scout-derived default minimum turn radius of 1.25 m, default map speed limit
  of 3.6 km/h and 0.2 m arc sampling. Existing maps are not rewritten.
- Configurable self-return crop margins (0.10 m horizontal, 0.05 m vertical)
  and scoped lidar parameter-node naming to avoid duplicate ROS node names.
- Independent UI entry point and ZED 2i / RSHELIOS / Fixposition / Scout dataset
  launch, preflight and MCAP recording scripts, with startup and dataset guides.

### Validation and Limits

- `source setup_env.bash` followed by
  `python3 -m unittest discover -s tests -v`: 29 tests passed on the local build.
- Local built C++ tests passed in an isolated ROS domain using Fast DDS:
  `test_runtime_reload` (1 case) and
  `test_autoware_behavior_path_planner_node_interface` (3 cases). These cover
  projection reload and planner map replacement, not the full Autoware suite.
- MPC/PID parameters are not a completed Scout skid-steer calibration. Generated
  maps, alignment, vehicle geometry and autonomous tracking require field checks.
- Runtime map hashes from startup-only services may still describe the startup
  map; use the UI runtime-map status to identify the active map directory.
- Source only: build/install/log output, generated site maps and recorded bags
  are not part of this release. Local map files are preserved but ignored by Git.
  Example absolute map paths in the startup guide refer to this vehicle; provide
  a valid local map when deploying elsewhere.
- The local overlay depends on the existing EZ21 underlay/CUDA packages. Dataset
  collection additionally needs the separately installed ZED SDK and ROS wrapper;
  cloning this repository alone does not install these binary dependencies.

### Retrieve This Snapshot

```bash
git clone --branch release/scout-fixposition-2026-10-23 \
  git@github.com:Unmanned-Systems-Lab/autoware.scout.git
cd autoware.scout
git show scout-v0.2.0-rc.1
```

Use [SCOUT_STARTUP.md](SCOUT_STARTUP.md) for integrated Autoware/UI startup and
[SCOUT_DATASET.md](SCOUT_DATASET.md) for independent data collection.
