from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    xsens_share = Path(get_package_share_directory("xsens_mti_ros2_driver"))
    ublox_share = Path(get_package_share_directory("ublox_dgnss"))

    enable_ntrip_arg = DeclareLaunchArgument(
        "enable_ntrip",
        default_value=TextSubstitution(text="true"),
        description="Start the NTRIP client alongside Xsens and u-blox",
    )
    ublox_log_level_arg = DeclareLaunchArgument(
        "ublox_log_level",
        default_value=TextSubstitution(text="INFO"),
        description="Log level for u-blox launch containers",
    )
    ublox_device_family_arg = DeclareLaunchArgument(
        "ublox_device_family",
        default_value=TextSubstitution(text="F9P"),
        description="u-blox device family, e.g. F9P, F9R, X20P",
    )
    ublox_device_serial_string_arg = DeclareLaunchArgument(
        "ublox_device_serial_string",
        default_value=TextSubstitution(text=""),
        description="Optional serial string to bind to a specific u-blox device",
    )
    ublox_frame_id_arg = DeclareLaunchArgument(
        "ublox_frame_id",
        default_value=TextSubstitution(text="ubx"),
        description="frame_id for u-blox messages",
    )
    ublox_namespace_arg = DeclareLaunchArgument(
        "ublox_namespace",
        default_value=TextSubstitution(text=""),
        description="Optional namespace for u-blox nodes",
    )
    enable_gnss_pose_bridge_arg = DeclareLaunchArgument(
        "enable_gnss_pose_bridge",
        default_value=TextSubstitution(text="true"),
        description="Publish /gnss_pose from /fix and /filter/quaternion",
    )
    enable_gnss_odometry_bridge_arg = DeclareLaunchArgument(
        "enable_gnss_odometry_bridge",
        default_value=TextSubstitution(text="true"),
        description="Publish /gnss_odometry from /fix and /imu/data",
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(xsens_share / "launch" / "xsens_mti_node.launch.py"))
    )

    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ublox_share / "launch" / "ublox_rover_hpposllh_navsatfix.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("ublox_log_level"),
            "device_family": LaunchConfiguration("ublox_device_family"),
            "device_serial_string": LaunchConfiguration("ublox_device_serial_string"),
            "frame_id": LaunchConfiguration("ublox_frame_id"),
            "namespace": LaunchConfiguration("ublox_namespace"),
        }.items(),
    )

    ntrip_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ublox_share / "launch" / "ntrip_client_vrs_rtcm32.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("enable_ntrip")),
    )

    gnss_pose_bridge_node = Node(
        package="xsens_mti_ros2_driver",
        executable="gnss_pose_bridge",
        name="gnss_pose_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_gnss_pose_bridge")),
    )

    gnss_odometry_bridge_node = Node(
        package="xsens_mti_ros2_driver",
        executable="gnss_odometry_bridge",
        name="gnss_odometry_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_gnss_odometry_bridge")),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            enable_ntrip_arg,
            ublox_log_level_arg,
            ublox_device_family_arg,
            ublox_device_serial_string_arg,
            ublox_frame_id_arg,
            ublox_namespace_arg,
            enable_gnss_pose_bridge_arg,
            enable_gnss_odometry_bridge_arg,
            xsens_launch,
            ublox_launch,
            ntrip_launch,
            gnss_pose_bridge_node,
            gnss_odometry_bridge_node,
        ]
    )
