# Scout Porting Record

## Provenance

| Source | Revision / local reference |
| --- | --- |
| EZ21 root | `8707e1e`, verified `/mnt/nvme0n1/autoware.ez21` workspace |
| Vendored Autoware | `SOURCE_VERSIONS.repos`, including local verified build fixes |
| ELE Scout | `7cd13052c4f3378a675e9c3b2b859b780c10bfbe`, `/home/agx/ele_scout_kit_github_20260827` |
| scout_ros2 | `34bf5dfaf8b1c8f2dd08d19790d0eead16bfb791` |
| ugv_sdk | `58436e9c1732474566e249ce7f726e12e26304d6` |

Sources are ordinary vendored files, not gitlinks. Upstream license files remain
in their packages. ELE's MIT license is preserved as
`src/vehicle/scout_vehicle_description/LICENSE-ELE`; its meshes and mounting
assembly are adapted. Fixposition and Scout licenses remain with their sources.
The root Autoware license does not supersede third-party licenses.

## Scope

- Replaced active EZ21 model packages with Scout vehicle and sensor kit packages.
- Added the Scout Autoware chassis adapter and `real_scout.launch.xml` entry point.
- Old real-vehicle entry names forward to Scout for compatibility.
- Shared localization/preprocessing code retains some EZ21/M1 names; these do
  not select the old hardware in the Scout entry point.
- Imported ELE Fixposition source/settings and retained Autoware-compatible
  RoboSense point fields with ELE RSHELIOS settings.
- Updated Smart MPC nominal wheelbase and map-generator vehicle-width lookup.
- Excluded inherited publication workflows, user recordings, generated maps,
  credentials, build outputs and downloaded platform dependencies.

Local validation reuses the separately verified EZ21 core as an underlay.
No physical vehicle motion or calibration is claimed by software-only tests.

## Local Validation

- Jetson aarch64, Ubuntu 22.04 / ROS 2 Humble, 2026-09-26.
- 17 selected adaptation packages compiled successfully against the verified
  EZ21 underlay (see the exact package list in README).
- 10 Python tests passed: six chassis adapter tests and four model/sensor/
  best-effort localization tests.
- Real Scout launch arguments resolved; hardware-disabled launch loaded Scout
  robot frames, localization bridge, perception, planning and control components.
  Missing sensor/vehicle input diagnostics are expected in this check.
- Chassis adapter launched with CAN disabled and exited cleanly on SIGINT.
- Local CUDA helper libraries were copied into ignored `.deps`; these platform
  binaries are not part of the published source repository.
