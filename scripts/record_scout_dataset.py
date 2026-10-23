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
    '/fixposition/odometry_llh': 'GNSS latitude/longitude',
    '/fixposition/fpa/odomstatus': 'GNSS/RTK status',
    '/sensing/imu/imu_data': 'Fixposition IMU',
    '/scout/status': 'battery and individual motor status',
    '/scout/wheel_odometry': 'wheel odometry',
}


def selected_topic(name):
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
        '/scout/status', '/scout/wheel_odometry', '/scout/wheel_tf',
        '/scout/cmd_vel', '/light_control', '/sensing/imu/imu_data',
        '/tf', '/tf_static', '/zed2i/joint_states', '/zed2i/zed2i_description',
    }


def discover_and_check(wait_seconds):
    rclpy.init()
    node = rclpy.create_node('scout_dataset_preflight')
    try:
        deadline = time.monotonic() + wait_seconds
        graph = {}
        while time.monotonic() < deadline:
            graph = {name: types for name, types in node.get_topic_names_and_types()}
            if all(name in graph and node.count_publishers(name) > 0 for name in REQUIRED):
                break
            rclpy.spin_once(node, timeout_sec=0.25)

        missing = [name for name in REQUIRED
                   if name not in graph or node.count_publishers(name) == 0]
        if missing:
            raise RuntimeError('缺少发布者：' + ', '.join(missing))

        received = {}
        subscriptions = []
        for name in REQUIRED:
            msg_type = get_message(graph[name][0])
            subscriptions.append(node.create_subscription(
                msg_type, name, lambda msg, topic=name: received.setdefault(topic, msg),
                qos_profile_sensor_data))
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline and len(received) < len(REQUIRED):
            rclpy.spin_once(node, timeout_sec=0.2)
        stale = [name for name in REQUIRED if name not in received]
        if stale:
            raise RuntimeError('话题已出现但未收到数据：' + ', '.join(stale))

        topics = {name: types[0] for name, types in graph.items()
                  if selected_topic(name) and node.count_publishers(name) > 0}
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
        'fixposition.yaml': sensor_share / 'config/fixposition.yaml',
        'rslidar.yaml': sensor_share / 'config/rslidar.yaml',
        'vehicle.xacro': vehicle_share / 'urdf/vehicle.xacro',
        'scout_dataset.launch.py': repository / 'launch/scout_dataset.launch.py',
        'record_scout_dataset.py': repository / 'scripts/record_scout_dataset.py',
    }
    for target, source in files.items():
        shutil.copy2(source, config_dir / target)


def main():
    parser = argparse.ArgumentParser(description='Record Scout sensor dataset to MCAP')
    parser.add_argument('--check', action='store_true', help='check all nine required live streams')
    parser.add_argument('--output-root', type=Path,
                        default=Path('/mnt/nvme0n1/autoware/bags'))
    parser.add_argument('--min-free-gb', type=float, default=10.0,
                        help='minimum free disk space before recording (default: 10 GiB)')
    parser.add_argument('--wait-seconds', type=float, default=10.0,
                        help='live-data timeout (default: 10 seconds)')
    args = parser.parse_args()
    if args.min_free_gb < 0 or args.wait_seconds <= 0:
        parser.error('min-free-gb must be >= 0 and wait-seconds must be > 0')

    try:
        topics, samples = discover_and_check(args.wait_seconds)
    except Exception as exc:
        print(f'采集检查失败：{exc}', file=sys.stderr)
        return 1
    print(f'9 路必需数据已收到；可录制 {len(topics)} 个话题：', flush=True)
    status = samples['/fixposition/fpa/odomstatus']
    rtk1, rtk2 = status.gnss1_status, status.gnss2_status
    print(f'RTK GNSS1/GNSS2 状态码：{rtk1}/{rtk2}（8 = RTK Fixed）', flush=True)
    if rtk1 != 8 and rtk2 != 8:
        print('注意：当前未检测到 RTK Fixed；仍可录制，但需在数据集里标记定位质量。',
              file=sys.stderr, flush=True)
    scout = samples['/scout/status']
    print(f'底盘电池电压：{scout.battery_voltage:.2f} V；电机状态数：'
          f'{len(scout.actuator_states)}', flush=True)
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
        'required_live_topics': list(REQUIRED),
        'start_rtk_status_codes': {'gnss1': rtk1, 'gnss2': rtk2},
        'start_battery_voltage_v': scout.battery_voltage,
        'note': 'Original ZED RGB and 32FC1 metric depth; image_transport compressed/theora duplicates omitted.',
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
