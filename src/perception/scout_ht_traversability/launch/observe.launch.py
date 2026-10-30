"""Start fusion and, optionally, ELE's perception only. Never starts a chassis."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory("scout_ht_traversability")
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=os.path.join(share, "config", "fusion.yaml")),
        DeclareLaunchArgument("base_costmap_topic", default_value=
                              "/planning/scenario_planning/parking/costmap_generator/occupancy_grid"),
        DeclareLaunchArgument("obstacle_threshold", default_value="100"),
        DeclareLaunchArgument("start_elevation", default_value="false"),
        DeclareLaunchArgument("global_map_root", default_value=os.environ.get(
            "SCOUT_HT_MAP_ROOT", os.path.expanduser("~/autoware_data/scout_elevation_maps"))),
        DeclareLaunchArgument("start_ht", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="scout_ht_traversability", executable="global_elevation_mapper",
             name="scout_global_elevation_manager", output="screen",
             condition=IfCondition(lc("start_elevation")),
             parameters=[{"storage_root": lc("global_map_root"),
                          "use_sim_time": ParameterValue(lc("use_sim_time"), value_type=bool)}]),
        Node(package="ht_traversability_ros2", executable="ht_traversability_node",
             name="ht_traversability_node", output="screen",
             condition=IfCondition(lc("start_ht")),
             parameters=[
                 PathJoinSubstitution([FindPackageShare("ele_scout_kit"), "config",
                                       "ht_traversability_real_car.yaml"]),
                 {"use_sim_time": ParameterValue(lc("use_sim_time"), value_type=bool),
                  "output_topic": "/scout/ht/raw", "aggregate_only": False,
                  "publish_direction_probabilities": True,
                  "publish_direction_costmaps": True,
                  "publish_merged_traversability": False,
                  "publish_valid_layer": True, "inference_fps": 5.0},
                 {"unobserved_is_free": True},
             ]),
        Node(package="scout_ht_traversability", executable="costmap_ht_fusion",
             name="scout_costmap_ht_fusion", output="screen",
             parameters=[lc("config"), {
                 "base_costmap_topic": lc("base_costmap_topic"),
                 "obstacle_threshold": ParameterValue(lc("obstacle_threshold"), value_type=int),
                 "use_sim_time": ParameterValue(lc("use_sim_time"), value_type=bool),
             }]),
        Node(package="rviz2", executable="rviz2", name="scout_ht_observer_rviz",
             condition=IfCondition(lc("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "fusion.rviz")],
             parameters=[{"use_sim_time": ParameterValue(lc("use_sim_time"), value_type=bool)}]),
    ])
