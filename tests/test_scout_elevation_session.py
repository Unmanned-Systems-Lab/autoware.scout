"""Coordinate-bound persistence, using real SQLite backups including WAL data."""
import json
from pathlib import Path
import sqlite3
import tempfile
import time
import unittest
from unittest.mock import patch, Mock

from scout_ht_traversability.elevation_session import (
    identity_for_origin, load_mapping_parameters, merge_parameters, prepare_session, snapshot_database,
)


class ElevationSessionTest(unittest.TestCase):
    origin = [-1556477.55692, 5327797.52122, 3132048.94151]
    contract = {"frame": "map", "resolution": .04, "sensor_calibration_sha256": "test"}

    def test_same_origin_resumes_and_new_origin_or_calibration_stays_separate(self):
        with tempfile.TemporaryDirectory() as root:
            identity = identity_for_origin(self.origin, self.contract)
            key, _, database = prepare_session(root, identity)
            database.write_bytes(b"persisted height tiles")
            self.assertEqual(prepare_session(root, identity)[2].read_bytes(), b"persisted height tiles")
            moved = list(self.origin)
            moved[0] += 1
            other, _, other_db = prepare_session(root, identity_for_origin(moved, self.contract))
            self.assertNotEqual(key, other)
            self.assertFalse(other_db.exists())
            changed = dict(self.contract, sensor_calibration_sha256="new calibration")
            self.assertNotEqual(key, prepare_session(root, identity_for_origin(self.origin, changed))[0])

    def test_invalid_or_unbound_database_is_rejected(self):
        for origin in ([0, 0, 0], [float("nan"), 1, 2], [1, 2]):
            with self.assertRaises(ValueError):
                identity_for_origin(origin, self.contract)
        with tempfile.TemporaryDirectory() as root:
            identity = identity_for_origin(self.origin, self.contract)
            _, directory, database = prepare_session(root, identity)
            database.touch()
            (directory / "session.json").unlink()
            with self.assertRaisesRegex(ValueError, "without coordinate metadata"):
                prepare_session(root, identity)

    def test_manifest_cannot_be_silently_rebound(self):
        with tempfile.TemporaryDirectory() as root:
            identity = identity_for_origin(self.origin, self.contract)
            _, directory, _ = prepare_session(root, identity)
            manifest = directory / "session.json"
            manifest.write_text(json.dumps({"identity": {"wrong": "frame"}}))
            with self.assertRaisesRegex(ValueError, "identity mismatch"):
                prepare_session(root, identity)

    def test_backup_preserves_committed_wal_and_does_not_overwrite(self):
        with tempfile.TemporaryDirectory() as root:
            src, dst = Path(root) / "source.sqlite3", Path(root) / "backup.sqlite3"
            connection = sqlite3.connect(src)
            connection.execute("PRAGMA journal_mode=WAL")
            connection.execute("CREATE TABLE observations (height REAL, count INTEGER)")
            connection.execute("INSERT INTO observations VALUES (.7, 42)")
            connection.commit()
            snapshot_database(src, dst)
            with sqlite3.connect(dst) as copy:
                self.assertEqual(copy.execute("SELECT * FROM observations").fetchall(), [(.7, 42)])
            with self.assertRaises(ValueError):
                snapshot_database(src, dst)
            connection.close()

    def test_configuration_keeps_nested_layers_and_explicit_scout_overrides(self):
        import yaml
        with tempfile.TemporaryDirectory() as root:
            files = []
            for i, data in enumerate([
                {"publishers": {"raw": {"fps": 10, "layers": ["elevation"]}},
                 "global_map_resume_from_disk": False},
                {"publishers": {"raw": {"fps": 5}}, "global_map_resume_from_disk": True},
            ]):
                p = Path(root) / f"{i}.yaml"
                p.write_text(yaml.safe_dump({"/elevation_mapping_node": {"ros__parameters": data}}))
                files.append(p)
            parameters = merge_parameters(*files)
            self.assertEqual(parameters["publishers"]["raw"], {"fps": 5, "layers": ["elevation"]})
            self.assertTrue(parameters["global_map_resume_from_disk"])

    def test_scout_prior_overrides_are_bound_to_database_identity(self):
        import yaml
        with tempfile.TemporaryDirectory() as root:
            root = Path(root)
            fixture = {
                "elevation_mapping_cupy/config/core/core_param.yaml": {
                    "resolution": .04, "global_map_initial_variance": 1000.0},
                "ele_scout_kit/config/elevation_mapping_real_car.yaml": {},
            }
            for name, parameters in fixture.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(yaml.safe_dump({"/elevation_mapping_node": {"ros__parameters": parameters}}))
            scout = Path(__file__).resolve().parents[1] / "src/perception/scout_ht_traversability/config/global_elevation.yaml"
            destination = root / "scout_ht_traversability/config/global_elevation.yaml"
            destination.parent.mkdir(parents=True)
            destination.write_text(scout.read_text())
            calibration = root / "scout_sensor_kit_description/config/sensors_calibration.yaml"
            calibration.parent.mkdir(parents=True)
            calibration.write_text("calibration: fixture\n")
            with patch("ament_index_python.packages.get_package_share_directory",
                       side_effect=lambda name: str(root / name)):
                parameters, contract = load_mapping_parameters()
            self.assertEqual(parameters["global_map_initial_elevation"], -1.0)
            self.assertEqual(parameters["global_map_initial_variance"], 10000.0)
            self.assertEqual(parameters["global_map_view_length_x"], 15.0)
            self.assertEqual(parameters["global_map_view_length_y"], 15.0)
            self.assertEqual(parameters["global_map_local_length_x"], 10.0)
            self.assertEqual(parameters["global_map_local_length_y"], 10.0)
            self.assertEqual(contract["fusion"]["global_map_initial_variance"], 10000.0)
            self.assertEqual(contract["fusion"]["global_map_conflict_confirmations"], 10)
            self.assertEqual(contract["fusion"]["global_map_conflict_tolerance"], .10)
            legacy = json.loads(json.dumps(contract))
            for name in ("global_map_conflict_confirmations", "global_map_conflict_tolerance",
                         "global_map_conflict_max_gap"):
                legacy["fusion"].pop(name)
            self.assertNotEqual(identity_for_origin(self.origin, legacy),
                                identity_for_origin(self.origin, contract))
            before = json.loads(json.dumps(contract))
            before["fusion"].pop("global_map_initial_elevation")
            before["fusion"].pop("global_map_initial_variance")
            old_key, _, old_db = prepare_session(root / "maps", identity_for_origin(self.origin, before))
            old_db.write_bytes(b"retained old map")
            new_key, _, new_db = prepare_session(root / "maps", identity_for_origin(self.origin, contract))
            self.assertNotEqual(old_key, new_key)
            self.assertFalse(new_db.exists())
            self.assertEqual(old_db.read_bytes(), b"retained old map")


class ElevationManagerTest(unittest.TestCase):
    def setUp(self):
        import rclpy
        from rclpy.parameter import Parameter
        from scout_ht_traversability.elevation_manager import ElevationManager
        self.root = tempfile.TemporaryDirectory()
        self.mapping = patch(
            "scout_ht_traversability.elevation_manager.load_mapping_parameters",
            return_value=({"global_map_local_topic": "/test/crop", "global_map_resume_from_disk": True},
                          ElevationSessionTest.contract))
        self.mapping.start()
        self.prefix = patch("scout_ht_traversability.elevation_manager.get_package_prefix",
                            return_value=self.root.name)
        self.prefix.start()
        self.popen = patch("scout_ht_traversability.elevation_manager.subprocess.Popen")
        self.spawn = self.popen.start()
        self.spawn.side_effect = lambda *a, **kw: Mock(poll=Mock(return_value=None))
        rclpy.init()
        self.node = ElevationManager(parameter_overrides=[Parameter("storage_root", value=self.root.name)])

    def tearDown(self):
        import rclpy
        self.node.stop_mapper()
        self.node.destroy_node()
        rclpy.shutdown()
        self.popen.stop()
        self.prefix.stop()
        self.mapping.stop()
        self.root.cleanup()

    def initialize(self):
        self.node.origin = tuple(ElevationSessionTest.origin)
        self.node.init_status = 2
        self.node.status_received = time.monotonic()
        self.node.tick()

    def test_mapper_waits_for_valid_global_initialization(self):
        self.node.tick()
        self.spawn.assert_not_called()
        self.assertEqual(self.node.last_state, "WAITING_GLOBAL_LOCALIZATION")

    def test_origin_change_stops_old_mapper_and_uses_another_database(self):
        import yaml
        self.initialize()
        original = self.node.process
        first = self.node.database
        self.node.tick()
        self.assertEqual(self.spawn.call_count, 1)
        self.node.origin = tuple(v + (1 if i == 0 else 0) for i, v in enumerate(self.node.origin))
        self.node.tick()
        original.send_signal.assert_called_once()
        self.assertNotEqual(first, self.node.database)
        self.assertEqual(self.spawn.call_count, 2)
        parameters = yaml.safe_load((self.node.database.parent / "mapping.params.yaml").read_text())
        self.assertTrue(parameters["/elevation_mapping_node"]["ros__parameters"]["global_map_resume_from_disk"])

    def test_lost_initialization_stops_mapper_and_invalidates_origin(self):
        from fixposition_driver_msgs.msg import FpaOdomstatus
        self.initialize()
        process = self.node.process
        self.node.on_fix(FpaOdomstatus(init_status=0))
        self.node.tick()
        process.send_signal.assert_called_once()
        self.assertIsNone(self.node.process)
        self.assertIsNone(self.node.origin)

    def test_mapper_crash_retries_after_backoff_in_same_session(self):
        self.initialize()
        database = self.node.database
        self.node.process.poll.return_value = 1
        self.node.tick()
        self.assertEqual(self.node.last_state, "MAPPER_EXITED")
        self.node.tick()
        self.assertEqual(self.spawn.call_count, 1)
        self.node.retry_after = 0
        self.node.tick()
        self.assertEqual(self.spawn.call_count, 2)
        self.assertEqual(self.node.database, database)


if __name__ == "__main__":
    unittest.main()
