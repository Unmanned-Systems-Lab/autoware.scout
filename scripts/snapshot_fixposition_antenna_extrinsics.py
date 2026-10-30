#!/usr/bin/env python3
"""Read only the antenna lever arms from a Fixposition device and save them."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import urllib.request


def vec3(value, field):
    if (not isinstance(value, list) or len(value) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v)
                    for v in value)):
        raise ValueError(f'{field} is not a finite 3-vector')
    return [float(v) for v in value]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-url', default='http://192.168.1.103')
    parser.add_argument('--output-root', type=Path,
                        default=Path('/mnt/nvme0n1/autoware/bags/'
                                     'fixposition_only_20261024_20261025'))
    args = parser.parse_args()
    endpoint = args.device_url.rstrip('/') + '/api/v2/params/config/get'
    with urllib.request.urlopen(endpoint, timeout=5) as response:
        config = json.load(response)
    if config.get('_ok') is not True:
        raise RuntimeError('Fixposition configuration GET did not succeed')
    params = config['params']
    gps = params['gps']
    antenna1 = vec3(gps['t_sensor_antenna1'], 'gps.t_sensor_antenna1')
    antenna2 = vec3(gps['t_sensor_antenna2'], 'gps.t_sensor_antenna2')
    baseline = math.dist(antenna1, antenna2)
    result = {
        'captured_at_utc': datetime.now(timezone.utc).isoformat(),
        'source': endpoint,
        'source_fields': ['gps.t_sensor_antenna1', 'gps.t_sensor_antenna2',
                          'gps.dual_antenna', 'housing'],
        'coordinate_frame': 'Fixposition sensor frame',
        'units': 'metres',
        'housing': params.get('housing'),
        'dual_antenna': gps.get('dual_antenna'),
        't_sensor_antenna1_m': antenna1,
        't_sensor_antenna2_m': antenna2,
        'configured_baseline_m': baseline,
        'historical_validity': 'Current read-only device configuration; historical '
                               'configuration during each bag is not verified.',
        'phase_center_note': 'Device values refer to GNSS antenna phase centres; '
                             'vehicle URDF antenna mesh origins may differ.',
    }
    args.output_root.mkdir(parents=True, exist_ok=True)
    body = json.dumps(result, ensure_ascii=False, indent=2) + '\n'
    (args.output_root / 'antenna_extrinsics_device_snapshot.json').write_text(body)
    for session in args.output_root.glob('scout_dataset_*'):
        if (session / 'rosbag/metadata.yaml').is_file():
            (session / 'antenna_extrinsics_reference.json').write_text(body)
    print('Saved current device antenna extrinsics: '
          f'GNSS1={antenna1}, GNSS2={antenna2}, baseline={baseline:.3f} m',
          flush=True)


if __name__ == '__main__':
    main()
