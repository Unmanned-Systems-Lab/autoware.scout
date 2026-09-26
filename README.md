# Autoware Scout

Autoware Humble source snapshot adapted from EZ21 for AgileX Scout, using the
local ELE Scout vehicle assembly, RoboSense RSHELIOS and Fixposition interfaces.
The original verified EZ21 workspace is not modified.

## Build

```bash
git clone git@github.com:Unmanned-Systems-Lab/autoware.scout.git
cd autoware.scout
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
./build_scout.bash
source setup_env.bash
```

Target: Ubuntu 22.04, ROS 2 Humble. Full Autoware dependencies include
CUDA/TensorRT and platform-specific libraries; rosdep alone may not supply these.
The checked-in setup scripts/manifests provide dependency references.
`src` contains the source snapshot: **do not run vcs import over it**.
`SOURCE_VERSIONS.repos` records provenance, not Scout modifications.
Build outputs, downloaded dependencies and maps are not committed.

Local incremental build against an already built matching EZ21 workspace:

```bash
SCOUT_UNDERLAY=/mnt/nvme0n1/autoware.ez21/install ./build_scout.bash \
  --packages-select ugv_sdk scout_msgs scout_base scout_autoware_interface \
  scout_vehicle_description scout_vehicle_launch scout_sensor_kit_description \
  scout_sensor_kit_launch fpsdk_common fpsdk_ros2 fixposition_driver_lib \
  fixposition_driver_msgs rtcm_msgs fixposition_driver_ros2 ins_driver_ez21 \
  autoware_launch autoware_smart_mpc_trajectory_follower \
  --allow-overriding ins_driver_ez21 autoware_launch autoware_smart_mpc_trajectory_follower
```

Overlay mode requires the underlay to remain installed. Local adaptation testing
uses this mode, not a fresh standalone full-stack build.

## Run

Prepare SocketCAN according to the actual chassis configuration. Verify emergency
stop, manual takeover and surroundings before enabling hardware. This starts real
hardware drivers:

```bash
./start_scout.bash map_path:=/absolute/path/to/map vehicle_interface_can_interface:=can0
```

Hardware-disabled launch check, with no chassis or sensor drivers:

```bash
./start_scout.bash map_path:=/absolute/path/to/map \
  start_sensor_drivers:=false launch_vehicle_interface:=false rviz:=false
```

The map must contain the Autoware map files and be aligned with Fixposition
`FP_ENU0`. The bridge compensates the device lever arm, but **does not convert
an arbitrary GNSS origin to the map origin**. Verify alignment against surveyed
map coordinates before vehicle operation.

## Interfaces

| Component | Scout configuration |
| --- | --- |
| Vehicle / sensors | `scout_vehicle` / `scout_sensor_kit` |
| Chassis | `scout_base` + `ugv_sdk`, SocketCAN `can0` by default |
| Command adapter | Autoware Control and Gear -> `/scout/cmd_vel` |
| Feedback | `/scout/status` -> Autoware velocity, steering, gear, mode |
| Lidar | RSHELIOS, UDP MSOP 3344 / DIFOP 5566, `/rslidar_points`, frame `rslidar` |
| Fixposition | `tcpcli://192.168.1.103:21000`, `/fixposition/odometry_enu` |
| IMU | Fixposition POI IMU -> `/sensing/imu/imu_data` |

Sensor settings: `src/sensor_component/scout_sensor_kit_launch/config`.
The existing RoboSense driver is retained for Autoware-compatible point fields;
its sensor type and network settings follow ELE Scout. The former CAN INS driver
is disabled. The reusable localization bridge remains in `ins_driver_ez21` and
accepts Fixposition best-effort odometry.

ELE mounting geometry is preserved relative to the chassis. Autoware `base_link`
is at ground level at the rear axle. Wheelbase: 0.498 m; tread: 0.58306 m;
wheel radius: 0.16459 m. Base-to-Fixposition: (0.4495158179, 0, 0.65899) m;
base-to-lidar: (0.4215, 0, 0.78499) m. The combined model owns sensor frames;
driver TF is isolated to avoid multiple parents. Camera and other mesh assets
are visual only, not additional enabled sensor drivers.

Scout is skid-steer, not Ackermann. The adapter uses the equivalent relation
`yaw_rate = velocity * tan(steering_angle) / wheelbase`, limited to 1.0 m/s and
0.8 rad/s by default. Gear is inferred, not physical gearbox feedback. Startup is
manual; autonomous requests require healthy CAN-mode feedback. Missing/stale
ROS commands or status, invalid values and incompatible gear produce zero velocity.
Remote takeover or invalid/stale feedback cancels automatic mode; recovering
feedback alone cannot re-enable motion without another autonomous request.
These checks do not replace chassis firmware timeouts or an independent E-stop.
The bicycle simulator and Smart MPC wheelbase are updated, but slip dynamics and
controller parameters require real Scout calibration before autonomous operation.
The inherited `real_vehicle_minimal` control preset disables optional AEB and
several collision/control checkers. This is a development configuration, not a
road-ready safety configuration; review the preset before any vehicle testing.

## Tests

```bash
source setup_env.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 -m unittest discover -s src/vehicle/scout_autoware_interface/test -v
python3 -m unittest discover -s tests -v
```

Tests cover command gating/limits, reverse steering, the combined model, sensor
endpoints and best-effort Fixposition localization with lever-arm correction.
Hardware-disabled checks are not real-vehicle acceptance tests.
Live sensor checks confirmed lidar data and Fixposition connectivity, but
Fixposition had no GNSS fix and uninitialized fusion. Its zero-valued outputs
must not be treated as valid localization. The host Ethernet interface also
requires a sensor-network address; the temporary test address was removed.
See [SCOUT_PORTING.md](SCOUT_PORTING.md) for provenance and licensing.
