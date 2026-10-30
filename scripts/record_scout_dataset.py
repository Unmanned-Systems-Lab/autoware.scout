#!/usr/bin/env python3
"""Validate live Scout sensors, then record their raw ROS topics to one MCAP bag."""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


REQUIRED = {
    '/rslidar_points': 'laser point cloud',
    '/zed2i/zed_node/rgb/color/rect/image': 'ZED color image',
    '/zed2i/zed_node/depth/depth_registered': 'ZED metric depth image',
    '/zed2i/zed_node/imu/data': 'ZED IMU',
    '/zed2i/zed_node/left_cam_imu_transform': 'ZED SDK left-camera to IMU extrinsic',
    '/fixposition/odometry_llh': 'GNSS latitude/longitude',
    '/fixposition/fpa/odomstatus': 'GNSS/RTK status',
    '/sensing/imu/imu_data': 'Fixposition IMU',
    '/scout/status': 'battery and individual motor status',
    '/scout/rc_state': 'raw RC switches and stick positions',
    '/scout/rc_cmd_vel_estimated': 'estimated RC linear/angular command',
    '/scout/wheel_odometry': 'wheel odometry',
}

REQUIRED_NO_IMAGES_LIDAR = {
    name: description for name, description in REQUIRED.items()
    if name not in {
        '/rslidar_points',
        '/zed2i/zed_node/rgb/color/rect/image',
        '/zed2i/zed_node/depth/depth_registered',
        '/scout/rc_cmd_vel_estimated',
    }
}
REQUIRED_NO_IMAGES_LIDAR.update({
    '/zed2i/zed_node/imu/data_raw': 'ZED raw IMU',
    '/fixposition/gnss1': 'GNSS1 independent position',
    '/fixposition/gnss2': 'GNSS2 independent position',
})


def selected_topic(name, topic_type, no_images_lidar=False):
    if no_images_lidar and (name.startswith('/rslidar') or topic_type in {
            'sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage',
            'sensor_msgs/msg/PointCloud2', 'stereo_msgs/msg/DisparityImage'}):
        return False
    if name.startswith('/fixposition/'):
        return True
    if name.startswith('/zed2i/zed_node/'):
        # Raw color and metric depth are the dataset sources. These image_transport
        # derivatives duplicate the images and increase CPU/disk use.
        return not (name.endswith('/compressed') or
                    name.endswith('/compressedDepth') or
                    name.endswith('/theora'))
    return name in {
        '/rslidar_points', '/rslidar_packets', '/rslidar_imu_data',
        '/scout/status', '/scout/rc_state', '/scout/wheel_odometry', '/scout/wheel_tf',
        '/scout/cmd_vel', '/scout/rc_cmd_vel_estimated',
        '/light_control', '/sensing/imu/imu_data',
        '/tf', '/tf_static', '/zed2i/joint_states', '/zed2i/zed2i_description',
    }


def valid_navfix(message):
    return (message is not None and message.status.status >= 0 and
            -90 <= message.latitude <= 90 and -180 <= message.longitude <= 180 and
            (abs(message.latitude) > 1e-7 or abs(message.longitude) > 1e-7))


def discover_and_check(wait_seconds, allow_invalid_gnss, require_rtk_fixed,
                       required, no_images_lidar):
    rclpy.init()
    node = rclpy.create_node('scout_dataset_preflight')
    try:
        deadline = time.monotonic() + wait_seconds
        graph = {}
        while time.monotonic() < deadline:
            graph = {name: types for name, types in node.get_topic_names_and_types()}
            if all(name in graph and node.count_publishers(name) > 0 for name in required):
                break
            rclpy.spin_once(node, timeout_sec=0.25)

        missing = [name for name in required
                   if name not in graph or node.count_publishers(name) == 0]
        if missing:
            raise RuntimeError('缺少发布者：' + ', '.join(missing))

        received = {}
        subscriptions = []
        for name in required:
            msg_type = get_message(graph[name][0])
            subscriptions.append(node.create_subscription(
                msg_type, name, lambda msg, topic=name: received.__setitem__(topic, msg),
                qos_profile_sensor_data))
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if len(received) < len(required):
                continue
            if not allow_invalid_gnss and not valid_navfix(
                    received['/fixposition/odometry_llh']):
                continue
            status = received['/fixposition/fpa/odomstatus']
            if require_rtk_fixed and (status.gnss1_status != 8 or status.gnss2_status != 8):
                continue
            break
        stale = [name for name in required if name not in received]
        if stale:
            if ('/scout/rc_cmd_vel_estimated' in stale and
                    '/scout/status' in received):
                mode = received['/scout/status'].control_mode
                raise RuntimeError(
                    '遥控估计速度暂无数据；底盘 control_mode='
                    f'{mode}（3=遥控模式），请先用遥控器接管，再检查；'
                    '其他未收到数据的话题：' + ', '.join(stale))
            raise RuntimeError('话题已出现但未收到数据：' + ', '.join(stale))
        fix = received['/fixposition/odometry_llh']
        if not allow_invalid_gnss and not valid_navfix(fix):
            raise RuntimeError('Fixposition 正在发无效坐标（NavSatFix status '
                               f'{fix.status.status}，经纬度 {fix.latitude}/{fix.longitude}）；'
                               '请到开阔处等 GNSS 定位，或仅在诊断时加 --allow-invalid-gnss')
        status = received['/fixposition/fpa/odomstatus']
        if require_rtk_fixed and (status.gnss1_status != 8 or status.gnss2_status != 8):
            raise RuntimeError('RTK 尚未固定：GNSS1/GNSS2 状态 '
                               f'{status.gnss1_status}/{status.gnss2_status}，需要均为 8')

        topics = {name: types[0] for name, types in graph.items()
                  if selected_topic(name, types[0], no_images_lidar)
                  and node.count_publishers(name) > 0}
        return dict(sorted(topics.items())), received
    finally:
        node.destroy_node()
        rclpy.shutdown()


def copy_configs(destination, repository):
    config_dir = destination / 'config'
    config_dir.mkdir()
    sensor_share = Path(get_package_share_directory('scout_sensor_kit_launch'))
    vehicle_share = Path(get_package_share_directory('scout_vehicle_description'))
    files = {
        'zed2i_640x360_15hz.yaml': repository / 'config/zed2i_640x360_15hz.yaml',
        'scout_rc_command.yaml': repository / 'config/scout_rc_command.yaml',
        'fixposition.yaml': sensor_share / 'config/fixposition.yaml',
        'rslidar.yaml': sensor_share / 'config/rslidar.yaml',
        'vehicle.xacro': vehicle_share / 'urdf/vehicle.xacro',
        'scout_dataset.launch.py': repository / 'launch/scout_dataset.launch.py',
        'record_scout_dataset.py': repository / 'scripts/record_scout_dataset.py',
        'verify_scout_dataset.py': repository / 'scripts/verify_scout_dataset.py',
        'scout_dataset_env.bash': repository / 'scripts/scout_dataset_env.bash',
        'scout_rc_command_estimator.py': repository / 'scripts/scout_rc_command_estimator.py',
    }
    # Preserve the serial-numbered Stereolabs camera/stereo calibration used by
    # this SDK installation. The camera-to-IMU transform is recorded separately
    # in /zed2i/zed_node/left_cam_imu_transform and /tf_static.
    zed_settings = Path(os.environ.get('ZED_DIR', '')) / 'settings'
    for calibration in sorted(zed_settings.glob('SN*.conf')):
        files[calibration.name] = calibration
    for target, source in files.items():
        shutil.copy2(source, config_dir / target)


def main():
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)
    parser = argparse.ArgumentParser(description='Record Scout sensor dataset to MCAP')
    parser.add_argument('--check', action='store_true', help='check all required live streams')
    parser.add_argument('--output-root', type=Path,
                        default=Path('/mnt/nvme0n1/autoware/bags'))
    parser.add_argument('--min-free-gb', type=float, default=10.0,
                        help='minimum free disk space before recording (default: 10 GiB)')
    parser.add_argument('--wait-seconds', type=float, default=10.0,
                        help='live-data timeout (default: 10 seconds)')
    parser.add_argument('--allow-invalid-gnss', action='store_true',
                        help='diagnostic only: allow invalid GNSS fix/zero coordinates')
    parser.add_argument('--require-rtk-fixed', action='store_true',
                        help='require both GNSS status codes to be 8 before recording')
    parser.add_argument('--no-images-lidar', action='store_true',
                        help='record GNSS, IMUs, base and other non-image topics; exclude lidar')
    args = parser.parse_args()
    if args.min_free_gb < 0 or args.wait_seconds <= 0:
        parser.error('min-free-gb must be >= 0 and wait-seconds must be > 0')

    required = REQUIRED_NO_IMAGES_LIDAR if args.no_images_lidar else REQUIRED
    try:
        topics, samples = discover_and_check(
            args.wait_seconds, args.allow_invalid_gnss, args.require_rtk_fixed,
            required, args.no_images_lidar)
    except Exception as exc:
        print(f'采集检查失败：{exc}', file=sys.stderr)
        return 1
    print(f'{len(required)} 路必需数据已收到；可录制 {len(topics)} 个话题：', flush=True)
    status = samples['/fixposition/fpa/odomstatus']
    rtk1, rtk2 = status.gnss1_status, status.gnss2_status
    print(f'RTK GNSS1/GNSS2 状态码：{rtk1}/{rtk2}（8 = RTK Fixed）', flush=True)
    if rtk1 != 8 and rtk2 != 8:
        print('注意：当前未检测到 RTK Fixed；仍可录制，但需在数据集里标记定位质量。',
              file=sys.stderr, flush=True)
    scout = samples['/scout/status']
    print(f'底盘电池电压：{scout.battery_voltage:.2f} V；电机状态数：'
          f'{len(scout.actuator_states)}', flush=True)
    print(f'底盘控制模式：{scout.control_mode}（1=CAN，3=遥控器）；'
          '只读采集节点不会切换模式', flush=True)
    rc = samples['/scout/rc_state']
    print('遥控摇杆原始值：右纵/横 '
          f'{rc.stick_right_v}/{rc.stick_right_h}，左纵/横 '
          f'{rc.stick_left_v}/{rc.stick_left_h}；这不是 m/s 单位的 cmd_vel',
          flush=True)
    for name in topics:
        print(f'  {name}', flush=True)
    if args.check:
        return 0

    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    free_bytes = shutil.disk_usage(output_root).free
    if free_bytes < args.min_free_gb * 1024**3:
        print(f'剩余空间 {free_bytes / 1024**3:.1f} GiB，低于 '
              f'{args.min_free_gb:g} GiB；未开始录制。', file=sys.stderr)
        return 1

    repository = Path(__file__).resolve().parent.parent
    now = datetime.now().astimezone()
    session = output_root / f'scout_dataset_{now:%Y%m%d_%H%M%S_%f}'
    session.mkdir()
    copy_configs(session, repository)
    try:
        revision = subprocess.check_output(
            ['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        revision = None
    manifest = {
        'created_at_local': now.isoformat(),
        'created_at_utc': now.astimezone(timezone.utc).isoformat(),
        'hostname': os.uname().nodename,
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
        'repository_revision': revision,
        'bag_path': 'rosbag',
        'topics': topics,
        'required_live_topics': list(required),
        'recording_profile': 'no_images_lidar' if args.no_images_lidar else 'full',
        'zed_factory_calibration_files': sorted(
            path.name for path in (session / 'config').glob('SN*.conf')),
        'start_rtk_status_codes': {'gnss1': rtk1, 'gnss2': rtk2},
        'start_gnss_fix_valid': valid_navfix(samples['/fixposition/odometry_llh']),
        'start_battery_voltage_v': scout.battery_voltage,
        'start_control_mode': scout.control_mode,
        'note': ('ZED IMU included; image, pointcloud and lidar topics excluded.'
                 if args.no_images_lidar else
                 'Original ZED RGB and 32FC1 metric depth; image_transport compressed/theora duplicates omitted.'),
    }
    (session / 'session.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    command = [
        'ros2', 'bag', 'record', '-s', 'mcap',
        '--max-bag-size', '4294967296',
        '--max-cache-size', '268435456',
        '-o', str(session / 'rosbag'), *topics.keys(),
    ]
    print(f'录制目录：{session}', flush=True)
    print('按 Ctrl+C 结束，等待 MCAP 写入完成后再关闭终端。', flush=True)
    recorder = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True)
    stopped_for_space = False
    try:
        while recorder.poll() is None:
            time.sleep(5)
            free_bytes = shutil.disk_usage(output_root).free
            if free_bytes < args.min_free_gb * 1024**3:
                print(f'剩余空间降到 {free_bytes / 1024**3:.1f} GiB；正在停止并写完 MCAP。',
                      file=sys.stderr, flush=True)
                stopped_for_space = True
                break
    except KeyboardInterrupt:
        print('\n正在停止录包并写完 MCAP，请稍候。', flush=True)
    if recorder.poll() is None:
        os.killpg(recorder.pid, signal.SIGINT)
    try:
        recorder.wait(timeout=120)
    except subprocess.TimeoutExpired:
        print('录包进程 120 秒仍未退出，请检查存储设备与进程。', file=sys.stderr)
        return 1
    print(f'录制结束。查看：ros2 bag info {session / "rosbag"}', flush=True)
    return 2 if stopped_for_space else recorder.returncode


if __name__ == '__main__':
    sys.exit(main())
