from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


OVERRIDABLE_ARGUMENTS = (
    ("can_interface", "Optional SocketCAN interface override, for example can0."),
    ("topic_name", "Optional odometry topic override, for example /odom."),
    ("frame_id", "Optional odometry frame override, for example odom."),
    ("child_frame_id", "Optional child frame override, for example base_link."),
    (
        "publish_raw_nav_sat_fix",
        "Optional raw NavSatFix publishing switch override: true/false.",
    ),
    (
        "raw_nav_sat_fix_topic",
        "Optional raw NavSatFix topic override, for example /sensing/ins/raw_nav_sat_fix.",
    ),
    (
        "raw_nav_sat_fix_frame_id",
        "Optional raw NavSatFix frame_id override, for example ins_link.",
    ),
    (
        "map_projector_info_path",
        "Optional map_projector_info.yaml path used as a fixed ENU origin.",
    ),
    ("socket_timeout_sec", "Optional CAN socket timeout override in seconds."),
    ("log_enabled", "Optional log switch override: true/false."),
    ("log_path", "Optional log output directory override, or an explicit file path."),
    ("log_name", "Optional output file name override. Use auto to keep timestamp naming."),
    ("log_name_format", "Optional strftime format override used when log_name is auto."),
    ("log_format", "Optional log format override: can or odom_csv."),
)


def parse_bool(text: str) -> bool:
    return text.strip().lower() in {"1", "true", "on", "yes"}


def launch_setup(context, *args, **kwargs):
    overrides = {}
    for name, _description in OVERRIDABLE_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context).strip()
        if not value:
            continue

        if name == "socket_timeout_sec":
            overrides[name] = float(value)
        elif name in {"log_enabled", "publish_raw_nav_sat_fix"}:
            overrides[name] = parse_bool(value)
        else:
            overrides[name] = value

    parameters = [LaunchConfiguration("params_file").perform(context)]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="ins_driver_ez21",
            executable="ins_driver_node",
            name="ins_driver_ez21",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("ins_driver_ez21"), "config", "driver.yaml"]
    )

    launch_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to the ROS 2 parameter YAML file.",
        )
    ]
    launch_arguments.extend(
        DeclareLaunchArgument(name, default_value="", description=description)
        for name, description in OVERRIDABLE_ARGUMENTS
    )
    launch_arguments.append(OpaqueFunction(function=launch_setup))

    return LaunchDescription(launch_arguments)
