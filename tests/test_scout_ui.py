import importlib.util
import http.client
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from functools import partial
from unittest.mock import MagicMock, patch
from xml.etree import ElementTree

from builtin_interfaces.msg import Time
from fixposition_driver_msgs.msg import FpaOdomstatus
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage


UI_PATH = (
    Path(__file__).resolve().parents[1]
    / "src/launcher/autoware_launch/ui/generate_autoware_map.py"
)
SPEC = importlib.util.spec_from_file_location("scout_ui_backend", UI_PATH)
UI = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = UI
SPEC.loader.exec_module(UI)


class ScoutUiTest(unittest.TestCase):
    def setUp(self):
        self.client = UI.AutowareRuntimeClient()

    def test_fixposition_topics(self):
        self.assertEqual(UI.FIXPOSITION_LLH_TOPIC, "/fixposition/odometry_llh")
        self.assertEqual(UI.FIXPOSITION_STATUS_TOPIC, "/fixposition/fpa/odomstatus")

    def test_scout_turn_radius_respects_geometry_and_command_limits(self):
        self.assertAlmostEqual(UI.DEFAULT_MIN_TURN_RADIUS_M, 1.25)
        speed_mps = UI.DEFAULT_SPEED_LIMIT_KPH / 3.6
        self.assertLessEqual(speed_mps / UI.DEFAULT_MIN_TURN_RADIUS_M, 0.8)
        self.assertLessEqual(math.atan(0.498 / UI.DEFAULT_MIN_TURN_RADIUS_M), 0.7)
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "vehicle_info.param.yaml"
            path.write_text(
                "/**:\n  ros__parameters:\n    wheel_base: 2.0\n    max_steer_angle: 0.7\n",
                encoding="utf-8",
            )
            with patch.object(UI, "SCOUT_VEHICLE_INFO_PATH", path):
                self.assertAlmostEqual(UI.load_scout_min_turn_radius_m(), 2.0 / math.tan(0.7))
            path.write_text(
                "/**:\n  ros__parameters:\n    wheel_base: 0.498\n    max_steer_angle: 0.0\n",
                encoding="utf-8",
            )
            with patch.object(UI, "SCOUT_VEHICLE_INFO_PATH", path):
                with self.assertRaisesRegex(UI.MapGenerationError, "max_steer_angle"):
                    UI.load_scout_min_turn_radius_m()

    def test_scout_two_meter_corner_uses_dense_arc_and_rejects_shorter_corner(self):
        points = [UI.LocalPoint(0.0, 0.0, 0.0), UI.LocalPoint(2.0, 0.0, 0.0),
                  UI.LocalPoint(2.0, 2.0, 0.0)]
        radius = UI.DEFAULT_MIN_TURN_RADIUS_M
        smoothed = UI.smooth_centerline_with_turn_radius(points, radius, UI.DEFAULT_TURN_ARC_STEP_M)
        self.assertEqual(smoothed[0], points[0])
        self.assertEqual(smoothed[-1], points[-1])
        arc = smoothed[1:-1]
        self.assertGreater(len(arc), 8)
        for point in arc:
            self.assertAlmostEqual(math.hypot(point.x - (2.0 - radius), point.y - radius), radius)
        for start, end in zip(arc, arc[1:]):
            self.assertLessEqual(UI.distance_xy(start, end), 0.2)
        short_points = [UI.LocalPoint(0.0, 0.0, 0.0), UI.LocalPoint(1.0, 0.0, 0.0),
                        UI.LocalPoint(1.0, 1.0, 0.0)]
        with self.assertRaisesRegex(UI.MapGenerationError, "minimum turning radius of 1.25 m"):
            UI.smooth_centerline_with_turn_radius(short_points, radius, UI.DEFAULT_TURN_ARC_STEP_M)

    def test_runtime_map_uses_launch_map_path(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            map_dir = Path(temp_dir)
            (map_dir / "map_projector_info.yaml").write_text(
                "projector_type: LocalCartesian\n"
                "map_origin:\n"
                "  latitude: 30.0\n"
                "  longitude: 114.0\n"
                "  altitude: 50.0\n",
                encoding="utf-8",
            )
            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": temp_dir}):
                context = UI.load_runtime_map_context({"map_name": "autoware_init_map"})
                self.assertEqual(context.output_dir, map_dir)
                self.assertEqual(context.origin, UI.GeoPoint(30.0, 114.0, 50.0))
                self.assertEqual(
                    UI.resolve_runtime_map_dir({"runtime_map_dir": "/tmp/other_map"}),
                    Path("/tmp/other_map"),
                )
                self.assertEqual(
                    UI.resolve_runtime_map_dir({}, include_launch_map=False),
                    (UI.DEFAULT_OUTPUT_ROOT / UI.DEFAULT_MAP_NAME).resolve(),
                )

    def test_autoware_launch_passes_map_path_to_ui(self):
        launch_path = (
            UI_PATH.parents[1]
            / "autoware_launch/launch/autoware.launch.xml"
        )
        launch = ElementTree.parse(launch_path)
        ui = launch.find(".//executable[@name='autoware_runtime_ui']")
        self.assertIsNotNone(ui)
        self.assertTrue(ui.attrib["cmd"].endswith("$(var map_path)"))

    def test_map_generation_guidance_does_not_reference_ez21_script(self):
        page = (UI_PATH.parent / "index.html").read_text(encoding="utf-8")
        self.assertNotIn("/media/nvidia/program/autoware.ez21", page)
        self.assertIn('"use_live_fp_enu0_origin": true', page)
        self.assertIn('"reload_autoware": true', page)
        self.assertIn("/activate_map", page)

    def test_live_fp_enu0_origin_requires_ready_fix_and_static_tf(self):
        transform = TransformStamped()
        transform.header.frame_id = "FP_ECEF"
        transform.child_frame_id = "FP_ENU0"
        transform.transform.translation.x = -1556480.77455
        transform.transform.translation.y = 5327800.95395
        transform.transform.translation.z = 3132041.56285
        message = TFMessage(transforms=[transform])
        self.client._on_fixposition_static_tf(message)

        with patch.object(self.client, "_ensure_ready"), patch.object(self.client, "_spin_locked"):
            with self.assertRaisesRegex(UI.MapGenerationError, "global initialization"):
                self.client.current_fp_enu0_origin()

            status = FpaOdomstatus()
            status.init_status = status.consts.INIT_STATUS_GLOBAL_INIT
            status.gnss1_status = status.consts.GNSS_STATUS_RTK_FIXED
            self.client._on_fixposition_status(status)
            origin = self.client.current_fp_enu0_origin()

        self.assertAlmostEqual(origin.latitude, 29.6001867948, places=6)
        self.assertAlmostEqual(origin.longitude, 106.2853638033, places=6)
        self.assertAlmostEqual(origin.altitude, 255.386, places=2)

    def test_live_fp_enu0_generation_writes_separate_map_without_reload(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir) / "scout_map"
            origin = UI.GeoPoint(29.6001867948, 106.2853638033, 255.386)
            payload = {
                "map_name": "scout_map",
                "output_dir": str(output_dir),
                "use_live_fp_enu0_origin": True,
                "reload_autoware": False,
                "data": [
                    {"latitude": 29.60019, "longitude": 106.28536},
                    {"latitude": 29.60029, "longitude": 106.28546},
                ],
            }
            with patch.object(
                UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=origin
            ), patch.object(UI, "reload_autoware_lanelet_map") as reload_lanelet:
                result = UI.generate_map_from_payload(payload)

            self.assertEqual(result["origin"], vars(origin))
            self.assertEqual(result["autoware_reload"]["status"], "skipped")
            reload_lanelet.assert_not_called()
            self.assertTrue((output_dir / "map_projector_info.yaml").exists())
            self.assertTrue((output_dir / "lanelet2_map.osm").exists())
            self.assertTrue((output_dir / "pointcloud_map.pcd").exists())

            context = UI.load_runtime_map_context({"runtime_map_dir": str(output_dir)})
            path_points = UI.parse_path_points(payload, context.elevation_m)
            route_poses = UI.build_route_poses(path_points, context)
            original_goal = route_poses[-1].position
            original_xy = (original_goal.x, original_goal.y)
            self.assertEqual(
                UI.move_generated_map_endpoint_goal_inside_lane(route_poses, context), 1.0
            )
            adjusted_goal = route_poses[-1].position
            self.assertAlmostEqual(
                ((adjusted_goal.x - original_xy[0]) ** 2
                 + (adjusted_goal.y - original_xy[1]) ** 2) ** 0.5,
                1.0,
                places=4,
            )

            near_end_poses = UI.build_route_poses(path_points, context)
            start = near_end_poses[0].position
            near_goal = near_end_poses[-1].position
            length = ((near_goal.x - start.x) ** 2 + (near_goal.y - start.y) ** 2) ** 0.5
            unit_x = (near_goal.x - start.x) / length
            unit_y = (near_goal.y - start.y) / length
            near_goal.x -= 0.5 * unit_x
            near_goal.y -= 0.5 * unit_y
            self.assertAlmostEqual(
                UI.move_generated_map_endpoint_goal_inside_lane(near_end_poses, context),
                0.5,
                places=4,
            )
            self.assertAlmostEqual(
                ((near_end_poses[-1].position.x - original_xy[0]) ** 2
                 + (near_end_poses[-1].position.y - original_xy[1]) ** 2) ** 0.5,
                1.0,
                places=4,
            )

            off_lane_poses = UI.build_route_poses(path_points, context)
            off_lane_poses[-1].position.x += 0.5 * unit_y
            off_lane_poses[-1].position.y -= 0.5 * unit_x
            self.assertEqual(
                UI.move_generated_map_endpoint_goal_inside_lane(off_lane_poses, context), 0.0
            )

            interior_poses = UI.build_route_poses(path_points, context)
            interior_poses[-1].position.x -= 0.5
            self.assertEqual(
                UI.move_generated_map_endpoint_goal_inside_lane(interior_poses, context), 0.0
            )

            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": str(output_dir)}):
                with patch.object(
                    UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=origin
                ):
                    with self.assertRaisesRegex(UI.MapGenerationError, "cannot overwrite"):
                        UI.generate_map_from_payload(payload)

    def test_map_lane_start_extends_five_meters_behind_vehicle(self):
        centerline = [UI.LocalPoint(0.0, 0.0, 0.0), UI.LocalPoint(0.0, -10.0, 0.0)]
        vehicle = {
            "received": True,
            "frame_id": "map",
            "age_ms": 0,
            "pose": {
                "position": {"x": 0.05, "y": 0.45},
                "orientation": {"yaw": -1.5707963267948966},
            },
        }

        extended, extension_m, margin_m = UI.extend_centerline_start_for_vehicle(
            centerline, 1.7, vehicle
        )
        self.assertAlmostEqual(extension_m, 5.45)
        self.assertAlmostEqual(extended[0].y, 5.45)
        self.assertAlmostEqual(margin_m, 5.0)
        self.assertEqual(centerline[0].y, 0.0)

        vehicle["pose"]["position"]["y"] = -6.0
        unchanged, extension_m, margin_m = UI.extend_centerline_start_for_vehicle(
            centerline, 1.7, vehicle
        )
        self.assertIs(unchanged, centerline)
        self.assertEqual(extension_m, 0.0)
        self.assertAlmostEqual(margin_m, 6.0)

        vehicle["pose"]["position"]["y"] = 3.0
        with self.assertRaisesRegex(UI.MapGenerationError, "more than 2 m ahead"):
            UI.extend_centerline_start_for_vehicle(centerline, 1.7, vehicle)

        vehicle["pose"]["position"] = {"x": 1.0, "y": 0.45}
        with self.assertRaisesRegex(UI.MapGenerationError, "not aligned"):
            UI.extend_centerline_start_for_vehicle(centerline, 1.7, vehicle)

        vehicle["pose"]["position"] = {"x": 1.0, "y": -0.5}
        with self.assertRaisesRegex(UI.MapGenerationError, "not aligned"):
            UI.extend_centerline_start_for_vehicle(centerline, 1.7, vehicle)

        curved_centerline = [
            UI.LocalPoint(0.0, 0.0, 0.0),
            UI.LocalPoint(0.0, -4.0, 0.0),
            UI.LocalPoint(4.0, -8.0, 0.0),
        ]
        vehicle["pose"]["position"] = {"x": 3.0, "y": -7.0}
        vehicle["pose"]["orientation"]["yaw"] = -0.7853981633974483
        _, extension_m, margin_m = UI.extend_centerline_start_for_vehicle(
            curved_centerline, 1.7, vehicle
        )
        self.assertEqual(extension_m, 0.0)
        self.assertGreater(margin_m, 5.0)

    def test_route_uses_current_vehicle_as_start_not_first_ui_point(self):
        client = self.client
        client._node = MagicMock()
        client._node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        client._route_client = object()
        client._clear_route_client = object()
        poses = [UI.Pose(), UI.Pose(), UI.Pose()]
        poses[0].position.x = 1.0
        poses[1].position.x = 2.0
        poses[2].position.x = 3.0
        with patch.object(client, "_ensure_ready"), patch.object(
            client, "_call_service_locked", return_value={"status": "ok"}
        ) as call_service:
            result = client.set_route(poses, clear_existing=False)
        request = call_service.call_args.args[2]
        self.assertEqual([pose.position.x for pose in request.waypoints], [2.0])
        self.assertEqual(request.goal.position.x, 3.0)
        self.assertEqual(result["waypoint_count"], 1)

        with patch.object(client, "_ensure_ready"), patch.object(
            client, "_call_service_locked", return_value={"status": "ok"}
        ) as call_service:
            result = client.set_route(poses[:1] + poses[2:], clear_existing=False)
        request = call_service.call_args.args[2]
        self.assertEqual(list(request.waypoints), [])
        self.assertEqual(result["waypoint_count"], 0)

    def test_hot_reload_requires_live_origin_and_stopped_vehicle(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            active_dir = Path(temp_dir)
            (active_dir / "map_projector_info.yaml").write_text(
                "projector_type: Local\n"
                "map_origin:\n"
                "  latitude: 29.5999\n"
                "  longitude: 106.28\n"
                "  altitude: 255.0\n",
                encoding="utf-8",
            )
            snapshot = {
                "operation_mode": {
                    "received": True,
                    "label": "STOP",
                    "is_autoware_control_enabled": False,
                    "is_in_transition": False,
                },
                "route_state": {"received": True, "label": "UNSET"},
            }
            vehicle = {
                "received": True,
                "age_ms": 100,
                "twist": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}},
            }
            live_origin = UI.GeoPoint(29.6001, 106.28, 255.0)
            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": temp_dir}), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=live_origin), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "snapshot", return_value=snapshot), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "vehicle_state_snapshot", return_value=vehicle):
                UI.prepare_runtime_map_switch(live_origin)
                with self.assertRaisesRegex(UI.MapGenerationError, "current Fixposition FP_ENU0"):
                    UI.prepare_runtime_map_switch(UI.GeoPoint(29.6, 106.28, 255.0))
                snapshot["operation_mode"]["label"] = "AUTONOMOUS"
                with self.assertRaisesRegex(UI.MapGenerationError, "stop the vehicle"):
                    UI.prepare_runtime_map_switch(live_origin)
                snapshot["operation_mode"]["label"] = "STOP"
                vehicle["twist"]["linear"]["x"] = 0.2
                with self.assertRaisesRegex(UI.MapGenerationError, "stationary"):
                    UI.prepare_runtime_map_switch(live_origin)

    def test_existing_generated_map_can_be_selected_but_old_origin_is_disabled(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            live_origin = UI.GeoPoint(29.6001, 106.28, 255.0)
            for name, latitude in (("current", 29.6001), ("old", 29.6)):
                map_dir = root / name
                map_dir.mkdir()
                (map_dir / "map_projector_info.yaml").write_text(
                    "projector_type: Local\nmap_origin:\n"
                    f"  latitude: {latitude}\n  longitude: 106.28\n  altitude: 255.0\n",
                    encoding="utf-8",
                )
                for filename in ("lanelet2_map.osm", "pointcloud_map.pcd", "pointcloud_map_metadata.yaml"):
                    (map_dir / filename).write_text("test", encoding="utf-8")

            with patch.object(UI, "DEFAULT_OUTPUT_ROOT", root), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=live_origin), \
                 patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": str(root / "old")}), \
                 patch.object(UI, "switch_autoware_map", return_value={"status": "ok"}) as switch:
                maps = {item["map_name"]: item for item in UI.list_generated_maps()["maps"]}
                self.assertTrue(maps["current"]["compatible_with_fp_enu0"])
                self.assertFalse(maps["old"]["compatible_with_fp_enu0"])
                result = UI.activate_generated_map_from_payload({"map_name": "current"})
                self.assertEqual(result["status"], "ok")
                switch.assert_called_once_with(root / "current")
                with self.assertRaisesRegex(UI.MapGenerationError, "valid generated map name"):
                    UI.activate_generated_map_from_payload({"map_name": "../current"})

    def test_route_is_blocked_when_active_map_origin_is_stale(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            map_dir = Path(temp_dir)
            (map_dir / "map_projector_info.yaml").write_text(
                "projector_type: Local\nmap_origin:\n"
                "  latitude: 29.6\n  longitude: 106.28\n  altitude: 255.0\n",
                encoding="utf-8",
            )
            payload = {
                "map_name": "old",
                "data": [
                    {"latitude": 29.6, "longitude": 106.28},
                    {"latitude": 29.6001, "longitude": 106.2801},
                ],
            }
            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": temp_dir}), \
                 patch.object(
                     UI.AUTOWARE_RUNTIME_CLIENT,
                     "current_fp_enu0_origin",
                     return_value=UI.GeoPoint(29.6001, 106.28, 255.0),
                 ), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "set_route") as set_route:
                with self.assertRaisesRegex(UI.MapGenerationError, "active map origin differs"):
                    UI.set_route_from_payload(payload)
                set_route.assert_not_called()

    def test_existing_map_http_endpoints(self):
        with tempfile.TemporaryDirectory() as temp_dir, \
             patch.object(UI, "list_generated_maps", return_value={"status": "ok", "maps": []}), \
             patch.object(
                 UI,
                 "activate_generated_map_from_payload",
                 return_value={"status": "ok", "map_name": "selected"},
             ) as activate, \
             patch.object(UI, "SCOUT_UI_ALLOW_VEHICLE_ACTIONS", True):
            handler = partial(UI.MapRequestHandler, directory=temp_dir)
            server = UI.UiThreadingHTTPServer(("127.0.0.1", 0), handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                connection = http.client.HTTPConnection("127.0.0.1", server.server_port)
                connection.request("GET", "/generated_maps")
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(json.loads(response.read())["maps"], [])
                connection.request(
                    "POST", "/activate_map", body=json.dumps({"map_name": "selected"}),
                    headers={"Content-Type": "application/json"},
                )
                response = connection.getresponse()
                self.assertEqual(response.status, 200)
                self.assertEqual(json.loads(response.read())["map_name"], "selected")
                activate.assert_called_once_with({"map_name": "selected"})
                connection.close()
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)

    def test_live_origin_hot_reload_updates_runtime_map_only_on_success(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            active_dir = Path(temp_dir) / "active"
            active_dir.mkdir()
            (active_dir / "map_projector_info.yaml").write_text(
                "projector_type: Local\n"
                "map_origin:\n"
                "  latitude: 29.5999\n"
                "  longitude: 106.28\n"
                "  altitude: 255.0\n",
                encoding="utf-8",
            )
            output_dir = Path(temp_dir) / "new_map"
            payload = {
                "map_name": "new_map",
                "output_dir": str(output_dir),
                "use_live_fp_enu0_origin": True,
                "reload_autoware": True,
                "data": [
                    {"latitude": 29.6, "longitude": 106.28},
                    {"latitude": 29.6001, "longitude": 106.2801},
                ],
            }
            vehicle = {
                "received": True,
                "frame_id": "map",
                "age_ms": 0,
                "pose": {
                    "position": {"x": -0.5, "y": -0.6},
                    "orientation": {"yaw": 0.85},
                },
            }
            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": str(active_dir)}), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=UI.GeoPoint(29.6, 106.28, 255.0)), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "vehicle_state_snapshot", return_value=vehicle), \
                 patch.object(UI, "prepare_runtime_map_switch"), \
                 patch.object(UI, "reload_autoware_lanelet_map", return_value={"status": "ok"}), \
                 patch.object(UI, "reload_autoware_pointcloud_map", return_value={"status": "ok"}):
                result = UI.generate_map_from_payload(payload)
                self.assertEqual(result["autoware_reload"]["status"], "ok")
                request = json.loads((output_dir / "map_generation_request.json").read_text())
                self.assertGreater(request["start_extension_m"], 5.0)
                self.assertAlmostEqual(request["lane_start_margin_m"], 5.0)
                self.assertEqual(os.environ["AUTOWARE_UI_RUNTIME_MAP_DIR"], str(output_dir))
                self.assertFalse(UI.MAP_SWITCH_FAILED)

            with patch.dict(os.environ, {"AUTOWARE_UI_RUNTIME_MAP_DIR": str(active_dir)}), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "current_fp_enu0_origin", return_value=UI.GeoPoint(29.6, 106.28, 255.0)), \
                 patch.object(UI.AUTOWARE_RUNTIME_CLIENT, "vehicle_state_snapshot", return_value=vehicle), \
                 patch.object(UI, "prepare_runtime_map_switch"), \
                 patch.object(UI, "reload_autoware_lanelet_map", return_value={"status": "error", "error": "failed"}):
                try:
                    result = UI.generate_map_from_payload(payload)
                    self.assertEqual(result["autoware_reload"]["status"], "error")
                    self.assertEqual(os.environ["AUTOWARE_UI_RUNTIME_MAP_DIR"], str(active_dir))
                    self.assertTrue(UI.MAP_SWITCH_FAILED)
                    with self.assertRaisesRegex(UI.MapGenerationError, "map switch failed"):
                        UI.set_route_from_payload(payload)
                finally:
                    UI.MAP_SWITCH_FAILED = False

    def test_lanelet_reload_updates_path_before_triggering_loader(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            map_dir = Path(temp_dir)
            (map_dir / "lanelet2_map.osm").write_text("<osm/>", encoding="utf-8")
            (map_dir / "map_projector_info.yaml").write_text(
                "projector_type: Local\n"
                "map_origin:\n"
                "  latitude: 29.6\n"
                "  longitude: 106.28\n"
                "  altitude: 255.0\n",
                encoding="utf-8",
            )
            with patch.object(
                UI.subprocess,
                "run",
                side_effect=[
                    subprocess.CompletedProcess(["bash"], 0, "Boolean value is: True\n", ""),
                    subprocess.CompletedProcess(["bash"], 0, "Set parameter successful\n", ""),
                ],
            ) as run, patch.object(UI, "wait_for_lanelet_map_after", return_value=True) as wait:
                result = UI.reload_autoware_lanelet_map(map_dir)

            self.assertEqual(result["status"], "ok")
            wait.assert_called_once()
            command = run.call_args_list[1].args[0][-1]
            first_set = command.index("ros2 param set")
            self.assertLess(first_set, command.index("ros2 param set", first_set + 1))
            self.assertIn(str(map_dir / "lanelet2_map.osm"), command)
            self.assertIn(str(map_dir / "map_projector_info.yaml"), command)
            self.assertIn(" && ", command)

    def test_lanelet_reload_rejects_old_projection_loader_without_mutating_map(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            map_dir = Path(temp_dir)
            (map_dir / "lanelet2_map.osm").write_text("<osm/>", encoding="utf-8")
            (map_dir / "map_projector_info.yaml").write_text("projector_type: Local\n", encoding="utf-8")
            with patch.object(
                UI.subprocess,
                "run",
                return_value=subprocess.CompletedProcess(["bash"], 1, "", "Parameter not set"),
            ) as run:
                result = UI.reload_autoware_lanelet_map(map_dir)
            self.assertEqual(result["status"], "error")
            self.assertIn("restart Autoware once", result["error"])
            self.assertEqual(run.call_count, 1)

    def test_pointcloud_reload_replaces_loader_and_checks_publisher(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            map_dir = Path(temp_dir)
            (map_dir / "pointcloud_map.pcd").write_text("pcd", encoding="utf-8")
            (map_dir / "pointcloud_map_metadata.yaml").write_text("{}", encoding="utf-8")
            calls = [
                subprocess.CompletedProcess(["bash"], 0, "1  /map/pointcloud_map_loader\n", ""),
                subprocess.CompletedProcess(["bash"], 0, "Loaded component", ""),
                subprocess.CompletedProcess(["bash"], 0, "Publisher count: 1\n", ""),
            ]
            with patch.object(UI.subprocess, "run", side_effect=calls) as run:
                result = UI.reload_autoware_pointcloud_map(map_dir)

            self.assertEqual(result["status"], "ok")
            command = run.call_args_list[1].args[0][-1]
            self.assertIn("ros2 component unload", command)
            self.assertIn("ros2 component load", command)
            self.assertIn("--parameter", command)
            self.assertIn("--remap-rule", command)
            self.assertIn(" && ", command)

    def test_non_generated_map_goal_is_unchanged(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            context = UI.RuntimeMapContext(
                output_dir=Path(temp_dir),
                projector_type="Local",
                origin=UI.GeoPoint(29.6, 106.28, 0.0),
                elevation_m=0.0,
                origin_source="map_projector_info.yaml:map_origin",
            )
            poses = UI.build_route_poses(
                [UI.GeoPoint(29.6, 106.28, 0.0), UI.GeoPoint(29.6001, 106.2801, 0.0)],
                context,
            )
            original_goal = (poses[-1].position.x, poses[-1].position.y)
            self.assertEqual(UI.move_generated_map_endpoint_goal_inside_lane(poses, context), 0.0)
            self.assertEqual((poses[-1].position.x, poses[-1].position.y), original_goal)

    def test_llh_requires_valid_recent_fix(self):
        msg = NavSatFix()
        msg.latitude = 30.0
        msg.longitude = 114.0
        msg.altitude = 50.0
        msg.position_covariance_type = msg.COVARIANCE_TYPE_KNOWN
        now = time.monotonic()

        msg.status.status = msg.status.STATUS_NO_FIX
        self.assertIsNone(self.client._raw_gps_to_geo_point(msg, now))

        msg.status.status = msg.status.STATUS_FIX
        self.assertEqual(
            self.client._raw_gps_to_geo_point(msg, now), UI.GeoPoint(30.0, 114.0, 50.0)
        )
        self.assertIsNone(
            self.client._raw_gps_to_geo_point(msg, now - UI.FIXPOSITION_DATA_TIMEOUT_S - 1)
        )

        msg.position_covariance_type = msg.COVARIANCE_TYPE_UNKNOWN
        self.assertIsNone(self.client._raw_gps_to_geo_point(msg, now))

    def test_fusion_requires_global_initialization_and_gnss(self):
        msg = FpaOdomstatus()
        now = time.monotonic()
        msg.init_status = msg.consts.INIT_STATUS_LOCAL_INIT
        msg.gnss1_status = msg.consts.GNSS_STATUS_SPP
        self.assertFalse(self.client._fixposition_ready(msg, now))

        msg.init_status = msg.consts.INIT_STATUS_GLOBAL_INIT
        self.assertTrue(self.client._fixposition_ready(msg, now))
        msg.gnss1_status = msg.consts.GNSS_STATUS_NO_FIX
        self.assertFalse(self.client._fixposition_ready(msg, now))

    def test_map_origin_does_not_fall_back_to_route_start(self):
        origin = {"latitude": 30.0, "longitude": 114.0, "altitude": 50.0}
        self.assertEqual(
            UI.resolve_generation_origin({"origin": origin}),
            (UI.GeoPoint(30.0, 114.0, 50.0), "payload:origin"),
        )
        with patch.object(UI, "load_runtime_map_context", side_effect=UI.MapGenerationError("missing")):
            with self.assertRaisesRegex(UI.MapGenerationError, "FP_ENU0"):
                UI.resolve_generation_origin({})


if __name__ == "__main__":
    unittest.main()
