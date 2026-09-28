# EZ21 Fixposition localization

The two `real_vehicle_planning_control_minimal*.launch.xml` entries now use
Fixposition `/fixposition/odometry_enu` and `/fixposition/fpa/odomstatus`.
The previous EZ21 CAN INS driver is disabled in these entries. Vehicle model,
vehicle interface and M1 lidar remain EZ21-specific.
The map/UI tool reads Fixposition `/fixposition/odometry_llh` instead of the old
CAN INS `/sensing/ins/raw_nav_sat_fix` topic.

Build the newly vendored Fixposition packages and changed launch/bridge packages:

```bash
cd /mnt/nvme0n1/autoware.ez21
./build_ez21.bash --packages-select fpsdk_common fpsdk_ros2 \
  fixposition_driver_lib fixposition_driver_msgs rtcm_msgs fixposition_driver_ros2 \
  ins_driver_ez21 autoware_launch --allow-overriding autoware_launch
```

The Fixposition TCP stream is configured in
`autoware_launch/config/sensing/fixposition.yaml`. The verified network endpoint
was `192.168.1.103:21000`; the sensor Ethernet interface needs an address on
that subnet. Check the actual device configuration before driving.

## Required calibration

The localization bridge **will not publish a pose** until both
`fixposition_extrinsics_calibrated` and `fixposition_map_aligned` are set to
`true`. These default to `false`. The former EZ21 CAN INS offsets and Scout
Fixposition offsets are not valid substitutes for an EZ21 Fixposition survey.
Both real-vehicle entries also default `launch_vehicle_interface` to `false`.
Connecting the chassis later requires explicitly passing
`launch_vehicle_interface:=true` after the localization checks are complete.

Measure the transform from EZ21 `base_link` to Fixposition `FP_POI` and set
`localization_base_link_to_ins_{x,y,z,roll,pitch,yaw}` in the real-vehicle
launch arguments. `base_link` on EZ21 is at the rear axle center at wheel-axis
height. Do not set `fixposition_extrinsics_calibrated:=true` until all six
values and axes are checked on the installed vehicle.

The map frame must match Fixposition `FP_ENU0`, including both translation and
heading. The generated test map has a Local projector origin at approximately
29.5916632735 N, 121.9868838033 E, 21.306 m. That alone does **not** establish
alignment with the device's `FP_ENU0`. Survey the origin/heading or implement
an explicit frame transform first. Set `fixposition_map_aligned:=true` only
after checking the same physical points in both frames.

Even when these flags are true, the bridge requires fresh Fixposition status,
global initialization (`init_status=2`), a GNSS fix, and a finite ENU odometry
message from `FP_POI`. If status becomes invalid or stale, it revokes the
Autoware initialized state and stops publishing pose/TF. The earlier live
test showed `init_status=0`, no GNSS fix and zero odometry, so that data is
correctly blocked.

## No-motion check

With a suitable map available, this starts the M1 real-vehicle stack without
the EZ21 vehicle interface or Fixposition network driver. Supply a recorded
Fixposition stream separately to test the bridge:

```bash
source /mnt/nvme0n1/autoware.ez21/setup_env.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1
ros2 launch autoware_launch real_vehicle_planning_control_minimal_robosense_m1.launch.xml \
  launch_vehicle_interface:=false launch_fixposition_driver:=false \
  launch_m1_driver:=false rviz:=false launch_ui:=false
```

For a live sensor-only check, start the Fixposition node from its parameter
file. Sensor-only operation does not enable EZ21 control:

```bash
ros2 run fixposition_driver_ros2 fixposition_driver_ros2_exec --ros-args \
  --params-file /mnt/nvme0n1/autoware.ez21/src/launcher/autoware_launch/autoware_launch/config/sensing/fixposition.yaml \
  -r /tf:=/fixposition/tf -r /tf_static:=/fixposition/tf_static
```

Check `/fixposition/fpa/odomstatus` and `/fixposition/odometry_enu` before
enabling localization. Do not turn on automatic vehicle control until the
sensor state, map frame and extrinsics are independently verified.

## Verification on this host

The eight selected packages compiled successfully. Four bridge tests passed,
including best-effort ROS transport and stale/invalid status rejection. A
hardware-disabled real-vehicle launch loaded EZ21 model and the Fixposition
localization bridge, then exited without node errors.

In a separate sensor-only check on 2026-09-28, the new EZ21 workspace connected
to `192.168.1.103:21000` and received 220 Fixposition status messages and 221
ENU odometry messages in 22 seconds. The latest status was `init_status=2`
(global initialization) and both GNSS status values were `5` (RTK float).
Status can change between runs. With geometry/map confirmation flags left
false, zero Autoware kinematic-state messages were published as intended.
No chassis interface or vehicle motion was used in this check.
