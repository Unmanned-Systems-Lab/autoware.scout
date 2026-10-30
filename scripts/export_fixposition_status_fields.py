#!/usr/bin/env python3
"""Write every recorded Fixposition ODOMSTATUS message to a small CSV sidecar."""

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from fixposition_driver_msgs.msg import FpaOdomstatus
import yaml


STATUS_TOPIC = '/fixposition/fpa/odomstatus'
COLUMNS = [
    'bag_timestamp_ns', 'header_timestamp_ns',
    'init_status', 'wheelspeed_status', 'gnss1_status', 'gnss2_status',
    'correction_status', 'imu_status', 'imu_noise',
    'fusion_ws', 'fusion_corr', 'fusion_gnss1', 'fusion_gnss2',
    'imu_conv', 'baseline_status',
]
FIELD_TO_MSG = {
    'wheelspeed_status': 'ws_status',
    'correction_status': 'corr_status',
}


def export_one(session):
    bag = session / 'rosbag'
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())[
        'rosbag2_bagfile_information']
    expected = next((item['message_count'] for item in metadata['topics_with_message_count']
                     if item['topic_metadata']['name'] == STATUS_TOPIC), 0)
    if expected <= 0:
        raise RuntimeError(f'{session.name}: no ODOMSTATUS messages')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[STATUS_TOPIC]))
    path = session / 'rtk_status_fields.csv'
    temp = session / 'rtk_status_fields.csv.partial'
    counts = defaultdict(Counter)
    rows = 0
    with temp.open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=COLUMNS)
        writer.writeheader()
        while reader.has_next():
            name, payload, bag_time = reader.read_next()
            if name != STATUS_TOPIC:
                raise RuntimeError(f'{session.name}: unexpected topic {name}')
            message = deserialize_message(payload, FpaOdomstatus)
            row = {
                'bag_timestamp_ns': bag_time,
                'header_timestamp_ns': message.header.stamp.sec * 1_000_000_000
                + message.header.stamp.nanosec,
            }
            for column in COLUMNS[2:]:
                value = int(getattr(message, FIELD_TO_MSG.get(column, column)))
                row[column] = value
                counts[column][value] += 1
            writer.writerow(row)
            rows += 1
    del reader
    if rows != expected:
        raise RuntimeError(f'{session.name}: read {rows} status messages, expected {expected}')
    temp.replace(path)
    summary = {
        'source_topic': STATUS_TOPIC,
        'message_count': rows,
        'fields': {name: dict(sorted(values.items())) for name, values in counts.items()},
        'note': 'Raw numeric status codes. See Fixposition FP_A-ODOMSTATUS documentation.',
    }
    (session / 'rtk_status_fields_summary.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(f'OK {session.name}: {rows} status rows', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('export_root', type=Path,
                        nargs='?', default=Path(
                            '/mnt/nvme0n1/autoware/bags/fixposition_only_20261024_20261025'))
    args = parser.parse_args()
    sessions = sorted(path for path in args.export_root.glob('scout_dataset_*')
                      if (path / 'rosbag/metadata.yaml').is_file())
    if not sessions:
        parser.error('no exported sessions found')
    for session in sessions:
        export_one(session)
    print(f'Completed {len(sessions)} sessions.', flush=True)


if __name__ == '__main__':
    main()
