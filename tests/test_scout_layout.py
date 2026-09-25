from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class ScoutLayoutTest(unittest.TestCase):
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
