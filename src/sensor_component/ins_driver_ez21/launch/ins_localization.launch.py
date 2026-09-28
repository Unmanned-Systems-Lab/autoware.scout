from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def parse_bool(text: str) -> bool:
    return text.strip().lower() in {"1", "true", "on", "yes"}


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context).strip()
    input_odom_topic = LaunchConfiguration("input_odom_topic").perform(context).strip()
    output_odom_topic = LaunchConfiguration("output_odom_topic").perform(context).strip()
    output_base_link_odom_topic = (
        LaunchConfiguration("output_base_link_odom_topic").perform(context).strip()
    )
    output_accel_topic = LaunchConfiguration("output_accel_topic").perform(context).strip()
    output_pose_topic = LaunchConfiguration("output_pose_topic").perform(context).strip()
    output_initialization_state_topic = (
        LaunchConfiguration("output_initialization_state_topic").perform(context).strip()
    )
    map_projector_info_path = (
        LaunchConfiguration("map_projector_info_path").perform(context).strip()
    )
    output_frame_id = LaunchConfiguration("output_frame_id").perform(context).strip()
    output_child_frame_id = LaunchConfiguration("output_child_frame_id").perform(context).strip()
    publish_tf = parse_bool(LaunchConfiguration("publish_tf").perform(context))
    use_sim_time = parse_bool(LaunchConfiguration("use_sim_time").perform(context))
    base_link_to_ins_x = float(LaunchConfiguration("base_link_to_ins_x").perform(context))
    base_link_to_ins_y = float(LaunchConfiguration("base_link_to_ins_y").perform(context))
    base_link_to_ins_z = float(LaunchConfiguration("base_link_to_ins_z").perform(context))
    base_link_to_ins_roll = float(LaunchConfiguration("base_link_to_ins_roll").perform(context))
    base_link_to_ins_pitch = float(LaunchConfiguration("base_link_to_ins_pitch").perform(context))
    base_link_to_ins_yaw = float(LaunchConfiguration("base_link_to_ins_yaw").perform(context))

    driver_parameters = [params_file, {"topic_name": input_odom_topic, "use_sim_time": use_sim_time}]
    if can_interface:
        driver_parameters.append({"can_interface": can_interface})
    if map_projector_info_path:
        driver_parameters.append({"map_projector_info_path": map_projector_info_path})

    bridge_parameters = [
        {
            "use_sim_time": use_sim_time,
            "input_odom_topic": input_odom_topic,
            "output_odom_topic": output_odom_topic,
            "output_base_link_odom_topic": output_base_link_odom_topic,
            "output_accel_topic": output_accel_topic,
            "output_pose_topic": output_pose_topic,
            "output_initialization_state_topic": output_initialization_state_topic,
            "output_frame_id": output_frame_id,
            "output_child_frame_id": output_child_frame_id,
            "publish_tf": publish_tf,
            "base_link_to_ins_x": base_link_to_ins_x,
            "base_link_to_ins_y": base_link_to_ins_y,
            "base_link_to_ins_z": base_link_to_ins_z,
            "base_link_to_ins_roll": base_link_to_ins_roll,
            "base_link_to_ins_pitch": base_link_to_ins_pitch,
            "base_link_to_ins_yaw": base_link_to_ins_yaw,
        }
    ]

    return [
        Node(
            package="ins_driver_ez21",
            executable="ins_driver_node",
            name="ins_driver_ez21",
            output="screen",
            parameters=driver_parameters,
        ),
        Node(
            package="ins_driver_ez21",
            executable="ins_localization_bridge",
            name="ins_localization_bridge",
            output="screen",
            parameters=bridge_parameters,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("ins_driver_ez21"), "config", "driver.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Path to ins_driver_ez21 parameter file.",
            ),
            DeclareLaunchArgument(
                "can_interface",
                default_value="",
                description="Optional SocketCAN interface override (e.g. can2).",
            ),
            DeclareLaunchArgument(
                "input_odom_topic",
                default_value="/odom",
                description="Input topic produced by INS driver in INS frame.",
            ),
            DeclareLaunchArgument(
                "output_odom_topic",
                default_value="/localization/kinematic_state",
                description="Localization odometry output topic in base_link frame.",
            ),
            DeclareLaunchArgument(
                "output_base_link_odom_topic",
                default_value="/odom_ins_at_base_link",
                description="Optional base_link odom output topic before frame remap to map.",
            ),
            DeclareLaunchArgument(
                "output_accel_topic",
                default_value="/localization/acceleration",
                description="Localization acceleration output topic.",
            ),
            DeclareLaunchArgument(
                "output_pose_topic",
                default_value="/localization/pose_twist_fusion_filter/pose",
                description="Pose output topic for component state monitor compatibility.",
            ),
            DeclareLaunchArgument(
                "output_initialization_state_topic",
                default_value="/localization/initialization_state",
                description="Localization initialization state output topic.",
            ),
            DeclareLaunchArgument(
                "map_projector_info_path",
                default_value="",
                description="Optional map_projector_info.yaml path used as a fixed ENU origin.",
            ),
            DeclareLaunchArgument(
                "output_frame_id",
                default_value="map",
                description="frame_id for /localization/kinematic_state output.",
            ),
            DeclareLaunchArgument(
                "output_child_frame_id",
                default_value="base_link",
                description="child_frame_id for converted odometry.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Whether bridge publishes TF.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="use_sim_time for both driver and bridge nodes.",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_x",
                default_value="-0.11741312220692635",
                description="INS mounting x in base_link frame [m].",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_y",
                default_value="0.22231443971395493",
                description="INS mounting y in base_link frame [m].",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_z",
                default_value="0.3301381915807724",
                description="INS mounting z in base_link frame [m].",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_roll",
                default_value="0.0",
                description="INS mounting roll in base_link frame [rad].",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_pitch",
                default_value="0.0",
                description="INS mounting pitch in base_link frame [rad].",
            ),
            DeclareLaunchArgument(
                "base_link_to_ins_yaw",
                default_value="0.0",
                description="INS mounting yaw in base_link frame [rad].",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
