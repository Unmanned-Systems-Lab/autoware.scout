# Autoware EZ21 with Fixposition

This branch contains a full ROS 2 Humble source snapshot for EZ21 with
Fixposition localization. It is intentionally separate from the Scout vehicle
on this repository's `main` branch. The EZ21 vehicle model, chassis interface
and RoboSense M1 lidar remain in use here.

## Build

```bash
git clone --branch ez21-fixposition --single-branch \
  git@github.com:Unmanned-Systems-Lab/autoware.scout.git
cd autoware.scout
./build_ez21.bash
source setup_env.bash
```

Target platform is Ubuntu 22.04 ARM64, ROS 2 Humble and Jetson CUDA 12.6.
The full stack needs Autoware, CUDA/TensorRT and platform dependencies.
Local extracted libraries under `.deps` and build outputs are not included.
`src` already contains the adapted source: do not run `vcs import` over it.
`source-versions.repos` records upstream revisions, not these local changes.
See [LOCAL_BUILD.md](LOCAL_BUILD.md) for the original Jetson build notes.

## Fixposition

The EZ21 real-vehicle launch entries use Fixposition ENU odometry and status.
The old CAN INS is disabled. The bridge accepts best-effort sensor messages and
only publishes localization after a fresh, initialized Fixposition status with
a GNSS fix. It also requires two explicit confirmations, both disabled by
default: measured EZ21 `base_link` to `FP_POI` extrinsics and alignment between
the map frame and Fixposition `FP_ENU0`. The EZ21 vehicle interface is also
disabled by default. No software flag substitutes for measuring these frames.

Detailed setup and commands: [FIXPOSITION_EZ21.md](src/launcher/autoware_launch/FIXPOSITION_EZ21.md).

The local adaptation build completed for eight changed/imported packages, four
bridge tests passed, and a hardware-disabled launch loaded the EZ21 model and
localization bridge. A sensor-only check received Fixposition status and ENU
odometry, while the unconfirmed frame settings correctly blocked Autoware pose
output. No vehicle motion was commanded.

This branch is an isolated source snapshot, not a merge of EZ21 into Scout.
Third-party package licenses remain alongside their source, including
`src/sensor_component/fixposition/LICENSE`.
