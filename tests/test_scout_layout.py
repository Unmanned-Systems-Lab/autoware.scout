import importlib.util
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchContext


ROOT = Path(__file__).resolve().parents[1]


class ScoutLayoutTest(unittest.TestCase):
    def self_crop_bounds(self, horizontal_margin, vertical_margin):
        path = ROOT / "src/launcher/autoware_launch/autoware_launch/launch/components/robosense_m1_pointcloud_preprocessor.launch.py"
        spec = importlib.util.spec_from_file_location("scout_pointcloud_preprocessor", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = LaunchContext()
        context.launch_configurations.update({
            "vehicle_param_file": str(ROOT / "src/vehicle/scout_vehicle_description/config/vehicle_info.param.yaml"),
            "self_crop_horizontal_margin_m": str(horizontal_margin),
            "self_crop_vertical_margin_m": str(vertical_margin),
        })
        return module.get_vehicle_info(context)

    def test_self_crop_removes_body_edge_returns_but_preserves_outside_points(self):
        root = ET.parse(ROOT / "src/launcher/autoware_launch/autoware_launch/launch/real_scout.launch.xml").getroot()
        args = {arg.attrib["name"]: arg.attrib.get("default") for arg in root.findall("arg")}
        horizontal = args["pointcloud_self_crop_horizontal_margin_m"]
        vertical = args["pointcloud_self_crop_vertical_margin_m"]
        bounds = self.self_crop_bounds(horizontal, vertical)
        minimum = [bounds[key] for key in ("min_longitudinal_offset", "min_lateral_offset", "min_height_offset")]
        maximum = [bounds[key] for key in ("max_longitudinal_offset", "max_lateral_offset", "max_height_offset")]
        for actual, expected in zip(minimum + maximum, [-0.3135, -0.449795, -0.05, 0.8115, 0.449795, 1.0]):
            self.assertAlmostEqual(actual, expected)
        def removed(point):
            return all(low <= value <= high for value, low, high in zip(point, minimum, maximum))
        self.assertTrue(removed((0.42, 0.365, 0.61)))
        self.assertTrue(removed((0.42, -0.43, 0.61)))
        self.assertFalse(removed((0.42, 0.50, 0.61)))
        self.assertFalse(removed((0.90, 0.0, 0.61)))
        self.assertFalse(removed((0.42, 0.0, 1.10)))
        include = next(node for node in root.findall("include")
                       if "robosense_m1_pointcloud_preprocessor" in node.attrib["file"])
        forwarded = {arg.attrib["name"]: arg.attrib["value"] for arg in include.findall("arg")}
        self.assertEqual(forwarded["self_crop_horizontal_margin_m"], "$(var pointcloud_self_crop_horizontal_margin_m)")
        self.assertEqual(forwarded["self_crop_vertical_margin_m"], "$(var pointcloud_self_crop_vertical_margin_m)")

    def test_self_crop_zero_margin_keeps_original_vehicle_bounds(self):
        bounds = self.self_crop_bounds(0.0, 0.0)
        self.assertEqual(bounds["min_longitudinal_offset"], -0.2135)
        self.assertEqual(bounds["max_longitudinal_offset"], 0.7115)
        self.assertAlmostEqual(bounds["max_lateral_offset"], 0.349795)

    def test_self_crop_rejects_invalid_margins(self):
        for horizontal, vertical in ((-0.1, 0.0), (0.1, -0.05), ("nan", 0.0), (0.0, "inf")):
            with self.subTest(horizontal=horizontal, vertical=vertical):
                with self.assertRaisesRegex(ValueError, "finite and nonnegative"):
                    self.self_crop_bounds(horizontal, vertical)

    def test_combined_model_and_extrinsics(self):
        model = subprocess.check_output([
            "xacro", str(ROOT / "src/launcher/autoware_launch/tier4_universe_launch/"
                         "tier4_vehicle_launch/urdf/vehicle.xacro"),
            "vehicle_model:=scout_vehicle", "sensor_model:=scout_sensor_kit",
        ], text=True)
        robot = ET.fromstring(model)
        links = [link.attrib["name"] for link in robot.findall("link")]
        self.assertEqual(len(links), len(set(links)))
        parents = {}
        for joint in robot.findall("joint"):
            child = joint.find("child").attrib["link"]
            self.assertNotIn(child, parents)
            xyz = [float(x) for x in joint.find("origin").attrib.get("xyz", "0 0 0").split()]
            parents[child] = (joint.find("parent").attrib["link"], xyz)
        def position(link):
            xyz = [0.0, 0.0, 0.0]
            seen = set()
            while link != "base_link":
                self.assertNotIn(link, seen)
                seen.add(link)
                link, offset = parents[link]
                xyz = [a + b for a, b in zip(xyz, offset)]
            return xyz
        for actual, expected in zip(position("FP_POI"), (0.4495158179045505, 0, 0.65899)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(position("rslidar"), (0.4215, 0, 0.78499)):
            self.assertAlmostEqual(actual, expected)

    def test_sensor_endpoints(self):
        config = ROOT / "src/sensor_component/scout_sensor_kit_launch/config"
        lidar = yaml.safe_load((config / "rslidar.yaml").read_text())["lidar"][0]
        self.assertEqual(lidar["driver"]["lidar_type"], "RSHELIOS")
        self.assertEqual((lidar["driver"]["msop_port"], lidar["driver"]["difop_port"]), (3344, 5566))
        self.assertEqual(lidar["ros"]["ros_frame_id"], "rslidar")
        self.assertTrue(lidar["ros"]["ros_send_autoware_pointcloud"])
        fp = yaml.safe_load((config / "fixposition.yaml").read_text())["/**"]["ros__parameters"]
        self.assertEqual(fp["stream"], "tcpcli://192.168.1.103:21000")
        self.assertFalse(fp["nav2_mode"])

    def test_scout_launch_disables_ez21_can_ins(self):
        path = ROOT / "src/launcher/autoware_launch/autoware_launch/launch/real_scout.launch.xml"
        root = ET.parse(path).getroot()
        args = {a.attrib["name"]: a.attrib.get("default") for a in root.findall("arg")}
        self.assertEqual(args["vehicle_model"], "scout_vehicle")
        self.assertEqual(args["sensor_model"], "scout_sensor_kit")
        self.assertEqual(args["localization_input_odom_topic"], "/fixposition/odometry_enu")
        self.assertTrue(any(a.attrib.get("name") == "launch_ins_driver" and a.attrib.get("value") == "false"
                            for a in root.iter("arg")))


if __name__ == "__main__":
    unittest.main()
