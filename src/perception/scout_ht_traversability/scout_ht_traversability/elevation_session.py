"""Bind ELE's persistent elevation database to one physical ENU coordinate frame."""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3

import yaml


def merge_parameters(*files):
    def merge(left, right):
        for key, value in right.items():
            if isinstance(value, dict) and isinstance(left.get(key), dict):
                merge(left[key], value)
            else:
                left[key] = deepcopy(value)
    result = {}
    for filename in files:
        document = yaml.safe_load(Path(filename).read_text())
        merge(result, document["/elevation_mapping_node"]["ros__parameters"])
    return result


def load_mapping_parameters():
    from ament_index_python.packages import get_package_share_directory as share
    parameters = merge_parameters(
        Path(share("elevation_mapping_cupy")) / "config/core/core_param.yaml",
        Path(share("ele_scout_kit")) / "config/elevation_mapping_real_car.yaml",
        Path(share("scout_ht_traversability")) / "config/global_elevation.yaml",
    )
    calibration = Path(share("scout_sensor_kit_description")) / "config/sensors_calibration.yaml"
    contract = {
        "version": 1,
        "frame": parameters["map_frame"],
        "base_frame": parameters["base_frame"],
        "resolution": float(parameters["resolution"]),
        "tile_size_cells": int(parameters.get("global_map_tile_size_cells", 128)),
        "sensor_calibration_sha256": hashlib.sha256(calibration.read_bytes()).hexdigest(),
        "pointcloud_topic": parameters["subscribers"]["lidar"]["topic_name"],
        "ray_origin_frame": parameters.get("ray_origin_frame", ""),
        "ray_evidence_version": 2,
        "fusion": {key: parameters[key] for key in (
            "global_map_fusion_variance_floor", "global_map_fusion_variance_ceiling",
            "global_map_initial_elevation", "global_map_initial_variance",
            "global_map_outlier_sigma", "global_map_max_height_jump",
            "global_map_conflict_confirmations", "global_map_conflict_tolerance",
            "global_map_conflict_max_gap", "global_map_clearance_confirmations",
            "global_map_clearance_margin")},
    }
    return parameters, contract


def identity_for_origin(ecef, contract):
    if len(ecef) != 3 or not all(math.isfinite(float(v)) for v in ecef):
        raise ValueError("Invalid FP_ENU0 ECEF origin")
    if not 6_000_000 <= math.sqrt(sum(float(v) ** 2 for v in ecef)) <= 7_000_000:
        raise ValueError("FP_ENU0 origin is outside the expected Earth radius")
    # Fixed device origin is stable. Millimetre quantization ignores float serialization noise.
    return {"fp_enu0_ecef_m": [round(float(v), 3) for v in ecef],
            "mapping_contract": deepcopy(contract)}


def prepare_session(root, identity):
    canonical = json.dumps(identity, sort_keys=True, separators=(",", ":"), allow_nan=False)
    key = hashlib.sha256(canonical.encode()).hexdigest()[:24]
    directory = Path(root).expanduser().resolve() / key
    directory.mkdir(parents=True, exist_ok=True)
    manifest = directory / "session.json"
    database = directory / "tiles.sqlite3"
    if manifest.exists():
        if json.loads(manifest.read_text())["identity"] != identity:
            raise ValueError("Elevation session identity mismatch")
    else:
        if database.exists():
            raise ValueError("Refusing to reuse an elevation database without coordinate metadata")
        temporary = manifest.with_suffix(".tmp")
        temporary.write_text(json.dumps({"session_id": key, "identity": identity}, indent=2) + "\n")
        os.replace(temporary, manifest)
    return key, directory, database


def snapshot_database(source, destination):
    """Consistent SQLite backup, including committed WAL data; never overwrite a session."""
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if destination.exists() or not source.is_file():
        raise ValueError("Backup needs an existing source and a new destination")
    temporary = destination.with_suffix(".backup.tmp")
    if temporary.exists():
        raise ValueError("A previous incomplete elevation backup needs inspection")
    try:
        with sqlite3.connect(source.as_uri() + "?mode=ro", uri=True) as src:
            with sqlite3.connect(str(temporary)) as dst:
                src.backup(dst)
                if dst.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
                    raise ValueError("Elevation backup integrity check failed")
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
