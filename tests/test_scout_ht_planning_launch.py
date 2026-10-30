"""Evaluate launch configuration only; never execute nodes or services."""
from pathlib import Path
import unittest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import get_launch_description_from_any_launch_file
from launch_ros.actions import ComposableNodeContainer
from launch_ros.utilities import evaluate_parameters


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "src/launcher/autoware_launch"


def fold(path, context, follow_includes=False):
    description = get_launch_description_from_any_launch_file(str(path))
    for action in description.entities:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(context)
        elif follow_includes and isinstance(action, IncludeLaunchDescription):
            child = action.launch_description_source.get_launch_description(context)
            for entry in child.entities:
                if isinstance(entry, DeclareLaunchArgument):
                    entry.execute(context)


class HtPlanningLaunch(unittest.TestCase):
    def real_config(self, enabled):
        context = LaunchContext()
        context.launch_configurations.update(map_path="/offline/map", scout_ht_enforce=enabled)
        fold(LAUNCH / "autoware_launch/launch/real_scout.launch.xml", context)
        return context

    def test_disabled_preserves_the_old_preset_and_bypass(self):
        c = self.real_config("false").launch_configurations
        self.assertEqual(c["planning_module_preset"], "robosense_m1_obstacle_stop_inlane_avoidance")
        self.assertEqual(c["launch_freespace_bypass_coordinator"], "true")

    def test_enforce_disables_the_unguarded_parking_branch(self):
        c = self.real_config("true").launch_configurations
        self.assertEqual(c["planning_module_preset"], "scout_ht_terrain")
        self.assertEqual(c["launch_freespace_bypass_coordinator"], "false")
        self.assertEqual(c["launch_parking_module"], "false")

    def test_scout_keeps_lane_costmap_publication_even_without_ht(self):
        self.assertEqual(self.real_config("false").launch_configurations["publish_lane_driving_costmap"], "true")

    def test_boolean_flag_is_case_insensitive(self):
        self.assertEqual(self.real_config("True").launch_configurations["planning_module_preset"], "scout_ht_terrain")

    def test_costmap_is_kept_without_loading_the_parking_planner(self):
        context = self.real_config("true")
        context.launch_configurations.update(
            container_type="component_container", vehicle_param_file="/unused/vehicle.yaml",
            costmap_generator_param_path="/unused/costmap.yaml",
            freespace_planner_param_path="/unused/parking.yaml")
        path = LAUNCH / "tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/parking.launch.xml"
        fold(path, context)
        description = get_launch_description_from_any_launch_file(str(path))
        group = next(a for a in description.entities if isinstance(a, GroupAction))
        self.assertTrue(group.condition.evaluate(context))
        container = next(a for a in group.get_sub_entities() if isinstance(a, ComposableNodeContainer))
        # Read descriptions only; executing the container would start ROS nodes.
        nodes = container._ComposableNodeContainer__composable_node_descriptions
        active = [n for n in nodes if n.condition() is None or n.condition().evaluate(context)]
        packages = [context.perform_substitution(n.package[0]) for n in active]
        self.assertIn("autoware_costmap_generator", packages)
        self.assertNotIn("autoware_freespace_planner", packages)
        costmap = next(n for n in active if context.perform_substitution(n.package[0]) == "autoware_costmap_generator")
        parameters = evaluate_parameters(context, costmap.parameters)
        self.assertTrue(any(isinstance(p, dict) and p.get("activate_in_lane_driving") is True for p in parameters))
        self.assertTrue(any(isinstance(p, dict) and p.get("publish_continuously") is True for p in parameters))

    def modules(self, enabled):
        context = self.real_config("true" if enabled else "false")
        context.launch_configurations.update(input_objects_topic_name="/offline/objects",
                                             input_pointcloud_topic_name="/offline/cloud")
        preset = context.launch_configurations["planning_module_preset"]
        fold(LAUNCH / f"autoware_launch/config/planning/preset/{preset}_preset.yaml",
             context, follow_includes=True)
        fold(LAUNCH / "tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/"
             "lane_driving/motion_planning/motion_planning.launch.xml", context)
        return context.launch_configurations

    def test_enforce_selects_sampler_and_loads_velocity_plugin(self):
        c = self.modules(True)
        self.assertEqual(c["motion_path_planner_type"], "path_sampler")
        self.assertIn("scout_ht_planning::VelocityModule", c["motion_velocity_planner_launch_modules"])
        self.assertIn("ObstacleStopModule", c["motion_velocity_planner_launch_modules"])
        self.assertIn("DynamicObstacleStopModule", c["motion_velocity_planner_launch_modules"])

    def test_disabled_keeps_optimizer_and_does_not_load_ht(self):
        c = self.modules(False)
        self.assertEqual(c["motion_path_planner_type"], "path_optimizer")
        self.assertNotIn("scout_ht_planning", c["motion_velocity_planner_launch_modules"])


if __name__ == "__main__":
    unittest.main()
