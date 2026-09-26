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

## Live Sensor Connectivity Check

Only sensor drivers were started in isolated, localhost-only ROS domains.
No chassis/CAN driver, autonomous mode or motion-command publisher was enabled.

- Ethernet `eno1` had carrier but no IPv4 address. Passive capture showed devices
  requesting host `192.168.1.102`. Temporarily assigning `192.168.1.102/24` made
  the sensor network reachable without changing Wi-Fi or the default route.
- RSHELIOS: received 384 clouds in a 40-second observation window (about 9.6 Hz),
  with 57,600 points in the last cloud, frame `rslidar`, and the expected
  Autoware point fields.
- Fixposition: ping and TCP `192.168.1.103:21000` succeeded. After restarting the
  driver with the Ethernet address present, ROS received ENU odometry, status
  and derived POI IMU messages.
- **Localization was not valid:** `init_status=0` (not initialized),
  `fusion_status=0` (not started), and both GNSS receivers reported no fix.
  Position and derived IMU values were zero; the driver warned that the
  `FP_ECEF -> FP_ENU0` transform was invalid. Receiving messages is not evidence
  of valid localization. Do not use this state for autonomous operation.
- All test drivers were stopped and the temporary Ethernet address was removed.
  No persistent network configuration or sensor configuration was changed.

Before further sensor testing, configure the host sensor-network address after
checking for address conflicts. Check GNSS antennas, reception and device fusion
status while keeping vehicle control disabled. Do not move the vehicle
autonomously to initialize localization. Raw test logs are intentionally not
included in the repository.
