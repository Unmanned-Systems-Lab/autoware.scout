#!/usr/bin/env python3
"""Copy Fixposition and Scout wheel data into compact MCAP bags.

All message payloads and bag timestamps are copied unchanged. RTK solution
quality is intentionally not used to filter messages.
"""

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import sys

import rosbag2_py
import yaml


EXTRA_FIXPOSITION_TOPICS = {'/sensing/imu/imu_data', '/tf_static'}
SCOUT_WHEEL_TOPICS = {'/scout/status', '/scout/wheel_odometry', '/scout/wheel_tf'}
SCOUT_RC_TOPICS = {'/scout/rc_state', '/scout/rc_cmd_vel_estimated'}


def selected_topic(name):
    return (name.startswith('/fixposition/') or name in EXTRA_FIXPOSITION_TOPICS
            or name in SCOUT_WHEEL_TOPICS or name in SCOUT_RC_TOPICS)


def topic_counts(bag_dir):
    metadata = yaml.safe_load((bag_dir / 'metadata.yaml').read_text())[
        'rosbag2_bagfile_information']
    counts = {
        item['topic_metadata']['name']: item['message_count']
        for item in metadata['topics_with_message_count']
    }
    return metadata, counts


def export_one(source, destination_root, update_existing=False):
    source_bag = source / 'rosbag'
    source_meta, source_counts = topic_counts(source_bag)
    expected = {name: count for name, count in source_counts.items()
                if selected_topic(name) and count > 0}
    if not expected:
        raise RuntimeError(f'{source.name}: no Fixposition messages found')

    destination = destination_root / source.name
    partial = destination_root / f'{source.name}.partial'
    replacing = destination.exists()
    if replacing:
        output_meta, output_counts = topic_counts(destination / 'rosbag')
        actual = {name: count for name, count in output_counts.items() if count > 0}
        if actual == expected and output_meta['message_count'] == sum(expected.values()):
            print(f'SKIP {source.name}: verified existing output', flush=True)
            return
        if not update_existing:
            raise RuntimeError(f'{destination} exists but its counts do not match the source; '
                               'use --update-existing after review')
        rtk_only = {name: count for name, count in expected.items()
                    if name not in SCOUT_WHEEL_TOPICS | SCOUT_RC_TOPICS}
        with_wheels = {name: count for name, count in expected.items()
                       if name not in SCOUT_RC_TOPICS}
        if actual == rtk_only:
            backup_name = '.rtk_only_backup'
        elif actual == with_wheels:
            backup_name = '.rtk_wheels_backup'
        else:
            raise RuntimeError(f'{destination} has unexpected topic counts')
    if partial.exists():
        raise RuntimeError(f'{partial} exists from an incomplete export; inspect it first')

    partial.mkdir(parents=True)
    output_bag = partial / 'rosbag'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(source_bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    topics = [topic for topic in reader.get_all_topics_and_types()
              if topic.name in expected]
    if {topic.name for topic in topics} != set(expected):
        raise RuntimeError(f'{source.name}: topic metadata does not match topic counts')

    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(output_bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in topics:
        writer.create_topic(topic)
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(expected)))
    copied = Counter()
    first_time = last_time = None
    while reader.has_next():
        name, payload, timestamp = reader.read_next()
        writer.write(name, payload, timestamp)
        copied[name] += 1
        if first_time is None:
            first_time = timestamp
        last_time = timestamp
    writer.close()
    del writer
    del reader

    output_meta, output_counts = topic_counts(output_bag)
    actual = {name: count for name, count in output_counts.items() if count > 0}
    if (dict(copied) != expected or actual != expected or
            output_meta['message_count'] != sum(expected.values())):
        raise RuntimeError(f'{source.name}: exported topic counts differ from source')

    source_config = source / 'config/fixposition.yaml'
    if source_config.is_file():
        (partial / 'config').mkdir()
        shutil.copy2(source_config, partial / 'config/fixposition.yaml')
    rc_config = source / 'config/scout_rc_command.yaml'
    if rc_config.is_file():
        (partial / 'config').mkdir(exist_ok=True)
        shutil.copy2(rc_config, partial / 'config/scout_rc_command.yaml')
    for name in ('session.json', 'quality_report.json', 'gnss_quality_report.json',
                 'rtk_status_report.json', 'quick_quality_report.json'):
        path = source / name
        if path.is_file():
            shutil.copy2(path, partial / f'source_{name}')
    manifest = {
        'source_session': str(source),
        'created_at_utc': datetime.now(timezone.utc).isoformat(),
        'selection': '/fixposition/*, /sensing/imu/imu_data, /tf_static, '
                     '/scout/status, /scout/wheel_odometry, /scout/wheel_tf, '
                     '/scout/rc_state, /scout/rc_cmd_vel_estimated; all RTK states',
        'timestamps_and_payloads': 'unchanged from source MCAP',
        'topic_message_counts': dict(sorted(expected.items())),
        'total_messages': sum(expected.values()),
        'first_bag_timestamp_ns': first_time,
        'last_bag_timestamp_ns': last_time,
        'source_total_messages': source_meta['message_count'],
    }
    (partial / 'export_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    if replacing:
        backup = destination_root / backup_name / source.name / 'rosbag'
        backup.parent.mkdir(parents=True, exist_ok=True)
        if backup.exists():
            raise RuntimeError(f'{backup} already exists; inspect previous update first')
        old_bag = destination / 'rosbag'
        old_bag.rename(backup)
        try:
            output_bag.rename(old_bag)
            installed_meta, installed_counts = topic_counts(old_bag)
            installed = {name: count for name, count in installed_counts.items()
                         if count > 0}
            if installed != expected or installed_meta['message_count'] != sum(expected.values()):
                raise RuntimeError(f'{source.name}: installed bag counts do not match')
            (partial / 'export_manifest.json').replace(destination / 'export_manifest.json')
            if (partial / 'config').is_dir():
                (destination / 'config').mkdir(exist_ok=True)
                for config_file in (partial / 'config').iterdir():
                    if config_file.is_file():
                        shutil.copy2(config_file, destination / 'config' / config_file.name)
            shutil.rmtree(partial)
        except Exception:
            if old_bag.exists():
                old_bag.rename(partial / 'rosbag')
            backup.rename(old_bag)
            raise
    else:
        partial.rename(destination)
    print(f'OK {source.name}: {sum(expected.values())} messages, '
          f'{len(expected)} topics, {shutil.disk_usage(destination_root).free / 1e9:.1f} GB free',
          flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path,
                        default=Path('/mnt/nvme0n1/autoware/bags'))
    parser.add_argument('--output-root', type=Path,
                        default=Path('/mnt/nvme0n1/autoware/bags/fixposition_only_20261024_20261025'))
    parser.add_argument('--dates', nargs='+', default=['20261024', '20261025'])
    parser.add_argument('--update-existing', action='store_true',
                        help='replace verified older MCAPs, keeping old ones in hidden backups')
    args = parser.parse_args()
    sources = sorted(source for date in args.dates
                     for source in args.source_root.glob(f'scout_dataset_{date}_*')
                     if (source / 'rosbag/metadata.yaml').is_file())
    if not sources:
        parser.error('no source sessions found for requested dates')
    args.output_root.mkdir(parents=True, exist_ok=True)
    readme = Path(__file__).resolve().parent.parent / 'FIXPOSITION_EXPORT.md'
    if readme.is_file():
        shutil.copy2(readme, args.output_root / 'README.md')
    print(f'Exporting {len(sources)} sessions to {args.output_root}', flush=True)
    for source in sources:
        export_one(source, args.output_root, args.update_existing)
    print('All Fixposition, Scout wheel, and RC bags exported.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        print(f'EXPORT FAILED: {error}', file=sys.stderr, flush=True)
        sys.exit(1)
