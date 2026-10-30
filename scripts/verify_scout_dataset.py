#!/usr/bin/env python3
"""Check that a completed Scout MCAP contains useful, aligned training data."""

import argparse
from bisect import bisect_left
from collections import defaultdict, deque
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

from record_scout_dataset import REQUIRED, valid_navfix


RGB = '/zed2i/zed_node/rgb/color/rect/image'
DEPTH = '/zed2i/zed_node/depth/depth_registered'
ZED_IMU = '/zed2i/zed_node/imu/data'
ZED_CAM_IMU = '/zed2i/zed_node/left_cam_imu_transform'
RTK = '/fixposition/fpa/odomstatus'
FIX = '/fixposition/odometry_llh'
SCOUT = '/scout/status'
COLOR_INFO = '/zed2i/zed_node/rgb/color/rect/camera_info'
DEPTH_INFO = '/zed2i/zed_node/depth/camera_info'
RATE_MINIMUMS = {
    RGB: 12, DEPTH: 12, '/rslidar_points': 8,
    ZED_IMU: 75,
    FIX: 8, RTK: 8, '/sensing/imu/imu_data': 8,
    SCOUT: 40, '/scout/rc_state': 40,
    '/scout/rc_cmd_vel_estimated': 40, '/scout/wheel_odometry': 40,
}


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def utc_time(ns):
    return datetime.fromtimestamp(ns / 1e9, timezone.utc).isoformat()


def gnss_sample(message, bag_stamp):
    header_stamp = stamp_ns(message.header)
    return {
        'latitude_deg': message.latitude,
        'longitude_deg': message.longitude,
        'altitude_m': message.altitude,
        'bag_timestamp_ns': bag_stamp,
        'bag_time_utc': utc_time(bag_stamp),
        'sensor_time_utc': utc_time(header_stamp) if header_stamp else None,
        'navsatfix_status': message.status.status,
    }


def connected(edges, start, target):
    queue = deque([start])
    seen = {start}
    while queue:
        frame = queue.popleft()
        if frame == target:
            return True
        for next_frame in edges.get(frame, ()):
            if next_frame not in seen:
                seen.add(next_frame)
                queue.append(next_frame)
    return False


def matched_fraction(left, right, tolerance_ns=50_000_000):
    if not left or not right:
        return 0.0
    right = sorted(right)
    matched = 0
    for value in left:
        i = bisect_left(right, value)
        best = min((abs(value - right[j]) for j in (i - 1, i)
                    if 0 <= j < len(right)), default=tolerance_ns + 1)
        matched += best <= tolerance_ns
    return matched / len(left)


def zed_recording_config(bag_dir):
    path = bag_dir.parent / 'config' / 'zed2i_640x360_15hz.yaml'
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text()) or {}
    return data.get('/**', {}).get('ros__parameters', {}).get('sensors', {})


def verify(bag_dir):
    meta = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {item['topic_metadata']['name']: item['message_count']
              for item in meta['topics_with_message_count']}
    types = {item['topic_metadata']['name']: item['topic_metadata']['type']
             for item in meta['topics_with_message_count']}
    errors = []
    warnings = []
    zed_sensors = zed_recording_config(bag_dir)
    required = dict(REQUIRED)
    if not zed_sensors.get('publish_cam_imu_transf', False):
        # Older sessions did not publish the SDK camera-to-IMU topic.
        required.pop(ZED_CAM_IMU, None)
    for name in required:
        if counts.get(name, 0) == 0:
            errors.append(f'必需话题没有消息：{name}')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    inspect = set(required) | {COLOR_INFO, DEPTH_INFO, '/tf_static',
                               '/fixposition/tf', '/fixposition/tf_static'}
    msg_types = {name: get_message(types[name]) for name in inspect if name in types}
    first_bag = {}
    last_bag = {}
    last_header = {}
    backward = defaultdict(int)
    delay_ms = defaultdict(list)
    image_stamps = defaultdict(list)
    image_bad = defaultdict(int)
    camera_info_valid = {COLOR_INFO: False, DEPTH_INFO: False}
    rtk_total = rtk_fixed = valid_fixes = fix_total = 0
    first_valid_fix = last_valid_fix = None
    depth_valid_fractions = []
    depth_seen = 0
    battery = []
    actuator_lengths = set()
    rc_stick_values = {axis: set() for axis in (
        'stick_right_v', 'stick_right_h', 'stick_left_v', 'stick_left_h')}
    edges = defaultdict(set)
    fixposition_tf_edges = set()
    enu0_ecef_transform = None
    enu0_reference_changes = 0
    camera_imu_transform = None
    camera_imu_transform_count = 0
    while reader.has_next():
        name, raw, bag_stamp = reader.read_next()
        if name not in msg_types:
            continue
        if name not in first_bag:
            first_bag[name] = bag_stamp
        last_bag[name] = bag_stamp
        message = deserialize_message(raw, msg_types[name])
        if hasattr(message, 'header'):
            header_stamp = stamp_ns(message.header)
            if header_stamp:
                if name in last_header and header_stamp < last_header[name]:
                    backward[name] += 1
                last_header[name] = header_stamp
                if len(delay_ms[name]) < 1000:
                    delay_ms[name].append((bag_stamp - header_stamp) / 1e6)
                if name in (RGB, DEPTH):
                    image_stamps[name].append(header_stamp)
        if name in (RGB, DEPTH):
            expected = (640, 360, 'bgra8' if name == RGB else '32FC1')
            if (message.width, message.height, message.encoding) != expected:
                image_bad[name] += 1
            if name == DEPTH:
                depth_seen += 1
                if depth_seen <= 5 or depth_seen % 100 == 0:
                    values = np.frombuffer(message.data, dtype=np.float32)
                    if values.size:
                        depth_valid_fractions.append(float(np.mean(np.isfinite(values) & (values > 0))))
        elif name in camera_info_valid:
            camera_info_valid[name] |= (message.width == 640 and message.height == 360 and
                                        message.k[0] > 0 and message.k[4] > 0)
        elif name == RTK:
            rtk_total += 1
            rtk_fixed += message.gnss1_status == 8 and message.gnss2_status == 8
        elif name == FIX:
            fix_total += 1
            if valid_navfix(message):
                valid_fixes += 1
                sample = gnss_sample(message, bag_stamp)
                if first_valid_fix is None:
                    first_valid_fix = sample
                last_valid_fix = sample
        elif name == SCOUT:
            battery.append(message.battery_voltage)
            actuator_lengths.add(len(message.actuator_states))
        elif name == ZED_CAM_IMU:
            camera_imu_transform_count += 1
            if camera_imu_transform is None:
                camera_imu_transform = {
                    'parent_frame': message.header.frame_id,
                    'child_frame': message.child_frame_id,
                    'translation_m': [message.transform.translation.x,
                                      message.transform.translation.y,
                                      message.transform.translation.z],
                    'rotation_xyzw': [message.transform.rotation.x,
                                      message.transform.rotation.y,
                                      message.transform.rotation.z,
                                      message.transform.rotation.w],
                }
        elif name == '/scout/rc_state':
            for axis, values in rc_stick_values.items():
                values.add(getattr(message, axis))
        elif name == '/tf_static':
            for transform in message.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                edges[parent].add(child)
                edges[child].add(parent)
        elif name in ('/fixposition/tf', '/fixposition/tf_static'):
            for transform in message.transforms:
                parent, child = transform.header.frame_id, transform.child_frame_id
                fixposition_tf_edges.add((parent, child))
                if (parent, child) == ('FP_ECEF', 'FP_ENU0'):
                    reference = {
                        'translation_m': [transform.transform.translation.x,
                                          transform.transform.translation.y,
                                          transform.transform.translation.z],
                        'rotation_xyzw': [transform.transform.rotation.x,
                                          transform.transform.rotation.y,
                                          transform.transform.rotation.z,
                                          transform.transform.rotation.w],
                    }
                    if enu0_ecef_transform is None:
                        enu0_ecef_transform = reference
                    elif (max(abs(a - b) for a, b in zip(
                            reference['translation_m'],
                            enu0_ecef_transform['translation_m'])) > 0.01 or
                          max(abs(a - b) for a, b in zip(
                            reference['rotation_xyzw'],
                            enu0_ecef_transform['rotation_xyzw'])) > 1e-4):
                        enu0_reference_changes += 1

    rates = {}
    rate_minimums = dict(RATE_MINIMUMS)
    requested_imu_rate = float(zed_sensors.get('sensors_pub_rate', 100.0))
    rate_minimums[ZED_IMU] = max(75.0, requested_imu_rate * 0.9)
    for name, threshold in rate_minimums.items():
        count = counts.get(name, 0)
        span = (last_bag.get(name, 0) - first_bag.get(name, 0)) / 1e9
        rate = (count - 1) / span if count > 1 and span > 0 else 0.0
        rates[name] = round(rate, 2)
        if count and rate < threshold and span >= 5:
            errors.append(f'帧率偏低：{name} {rate:.2f} Hz（最低 {threshold} Hz）')
    for name, bad in image_bad.items():
        if bad:
            errors.append(f'图像尺寸或编码错误：{name}，{bad} 帧')
    for name, valid in camera_info_valid.items():
        if not valid:
            errors.append(f'相机内参缺失或无效：{name}')
    for name, count in backward.items():
        if count:
            errors.append(f'消息头时间戳倒退：{name}，{count} 次')

    pair_fraction = matched_fraction(image_stamps[RGB], image_stamps[DEPTH])
    if counts.get(RGB, 0) and pair_fraction < 0.95:
        errors.append(f'彩色图/深度图 50 ms 内匹配率只有 {pair_fraction:.1%}')
    fix_fraction = valid_fixes / fix_total if fix_total else 0
    fixed_fraction = rtk_fixed / rtk_total if rtk_total else 0
    if fix_fraction < 0.95:
        errors.append(f'有效 GNSS 坐标比例只有 {fix_fraction:.1%}')
    if fixed_fraction < 0.95:
        warnings.append(f'双天线 RTK Fixed 比例只有 {fixed_fraction:.1%}；需要固定解的训练片段应筛选')
    if actuator_lengths != {4}:
        errors.append(f'底盘电机状态数组长度异常：{sorted(actuator_lengths)}')
    if counts.get('/scout/rc_state', 0) and all(len(values) <= 1 for values in rc_stick_values.values()):
        warnings.append('整段记录中遥控摇杆原始值没有变化；若本段应包含人工驾驶，请检查遥控器反馈')
    tf_targets = ['rslidar', 'FP_POI', 'zed2i_left_camera_frame_optical']
    tf_connected = {target: connected(edges, 'base_link', target) for target in tf_targets}
    for target, okay in tf_connected.items():
        if not okay:
            errors.append(f'静态 TF 未连通：base_link → {target}')
    has_enu_tf = any('FP_ENU0' in edge for edge in fixposition_tf_edges)
    if fixed_fraction >= 0.95 and not has_enu_tf:
        warnings.append('RTK 已固定，但包中未发现 FP_ENU0 相关 TF；检查全球轨迹坐标关系')
    if enu0_reference_changes:
        errors.append(f'同一包内 FP_ECEF → FP_ENU0 原点发生变化：{enu0_reference_changes} 条')
    depth_valid = statistics.mean(depth_valid_fractions) if depth_valid_fractions else 0.0
    if depth_valid < 0.05:
        warnings.append(f'深度图采样有效像素比例较低：{depth_valid:.1%}')
    bag_start_ns = meta['starting_time']['nanoseconds_since_epoch']
    bag_end_ns = bag_start_ns + meta['duration']['nanoseconds']

    report = {
        'bag_path': str(bag_dir),
        'bag_duration_seconds': meta['duration']['nanoseconds'] / 1e9,
        'message_count': meta['message_count'],
        'required_topic_counts': {name: counts.get(name, 0) for name in required},
        'zed_imu_requested_hz': requested_imu_rate,
        'zed_camera_imu_transform': camera_imu_transform,
        'zed_camera_imu_transform_count': camera_imu_transform_count,
        'required_topic_rates_hz': rates,
        'rgb_depth_within_50ms_fraction': round(pair_fraction, 4),
        'valid_gnss_fraction': round(fix_fraction, 4),
        'gnss_first_valid_fix': first_valid_fix,
        'gnss_last_valid_fix': last_valid_fix,
        'gnss_first_valid_fix_after_bag_start_seconds': (
            round((first_valid_fix['bag_timestamp_ns'] - bag_start_ns) / 1e9, 3)
            if first_valid_fix else None),
        'gnss_last_valid_fix_before_bag_end_seconds': (
            round((bag_end_ns - last_valid_fix['bag_timestamp_ns']) / 1e9, 3)
            if last_valid_fix else None),
        'segment_local_origin': first_valid_fix,
        'dual_rtk_fixed_fraction': round(fixed_fraction, 4),
        'depth_sample_valid_pixel_fraction': round(depth_valid, 4),
        'battery_voltage_v_min_max': [min(battery), max(battery)] if battery else None,
        'rc_stick_distinct_value_counts': {axis: len(values) for axis, values in rc_stick_values.items()},
        'base_link_static_tf_connected': tf_connected,
        'fixposition_enu_tf_present': has_enu_tf,
        'fixposition_enu0_ecef_transform': enu0_ecef_transform,
        'fixposition_enu0_reference_changed_messages': enu0_reference_changes,
        'median_bag_minus_header_ms': {name: round(statistics.median(values), 2)
                                        for name, values in delay_ms.items() if values},
        'optional_topics_with_zero_messages': sorted(
            name for name, count in counts.items() if name not in required and count == 0),
        'errors': errors,
        'warnings': warnings,
        'extrinsics_note': 'Connected TF uses vehicle.xacro mount geometry; verify physical camera/lidar/RTK extrinsics before 3D fusion training.',
    }
    return report


def main():
    parser = argparse.ArgumentParser(description='Verify completed Scout dataset bag')
    parser.add_argument('session', type=Path, help='scout_dataset_* directory or its rosbag directory')
    args = parser.parse_args()
    session = args.session.expanduser().resolve()
    bag_dir = session / 'rosbag' if (session / 'rosbag').is_dir() else session
    if not (bag_dir / 'metadata.yaml').is_file():
        parser.error(f'找不到已完成的 ROS 2 bag：{bag_dir}')
    report = verify(bag_dir)
    report_path = bag_dir.parent / 'quality_report.json'
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(f'报告：{report_path}')
    print(f'包时长 {report["bag_duration_seconds"]:.1f} 秒，消息 {report["message_count"]} 条')
    for name, count in report['required_topic_counts'].items():
        rate = report['required_topic_rates_hz'].get(name)
        print(f'{name}: {count} 条' + (f'，{rate:.2f} Hz' if rate is not None else ''))
    print(f'彩色/深度图 50 ms 匹配率：{report["rgb_depth_within_50ms_fraction"]:.1%}')
    print(f'有效 GNSS：{report["valid_gnss_fraction"]:.1%}；双天线 RTK Fixed：'
          f'{report["dual_rtk_fixed_fraction"]:.1%}')
    for label, key in [('起点', 'gnss_first_valid_fix'), ('终点', 'gnss_last_valid_fix')]:
        fix = report[key]
        if fix:
            print(f'{label}：{fix["latitude_deg"]:.8f}, {fix["longitude_deg"]:.8f}，'
                  f'海拔 {fix["altitude_m"]:.2f} m，{fix["bag_time_utc"]}')
        else:
            print(f'{label}：无有效 GNSS 坐标')
    print('静态坐标连接：' + ', '.join(
        f'{name}={"是" if okay else "否"}'
        for name, okay in report['base_link_static_tf_connected'].items()))
    for issue in report['warnings']:
        print('提示：' + issue)
    for issue in report['errors']:
        print('失败：' + issue)
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    sys.exit(main())
