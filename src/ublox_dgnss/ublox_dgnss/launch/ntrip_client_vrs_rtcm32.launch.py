"""Launch ntrip client with saved VRS_RTCM32 parameters."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_share = Path(get_package_share_directory("ublox_dgnss"))
    params_file = package_share / "config" / "ntrip_vrs_rtcm32.yaml"

    container = ComposableNodeContainer(
        name="ntrip_client_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        arguments=["--ros-args", "--log-level", "DEBUG"],
        composable_node_descriptions=[
            ComposableNode(
                package="ntrip_client_node",
                plugin="ublox_dgnss::NTRIPClientNode",
                name="ntrip_client",
                parameters=[str(params_file)],
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_USE_STDOUT", "1"),
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            container,
        ]
    )
