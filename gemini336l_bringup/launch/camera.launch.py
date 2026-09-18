"""Launch the official Orbbec wrapper with Pi 5-safe defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    align_mode = LaunchConfiguration("align_mode")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_colored_point_cloud = LaunchConfiguration("enable_colored_point_cloud")
    driver_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument(
                "align_mode",
                default_value="HW",
                description="Use SW if the selected profiles do not support hardware D2C.",
            ),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(driver_launch),
                launch_arguments={
                    "camera_name": camera_name,
                    "enable_color": "true",
                    "color_width": "640",
                    "color_height": "480",
                    "color_fps": "30",
                    "color_format": "RGB",
                    "enable_depth": "true",
                    "depth_width": "640",
                    "depth_height": "480",
                    "depth_fps": "30",
                    "depth_format": "Y16",
                    "depth_registration": "true",
                    "align_mode": align_mode,
                    "align_target_stream": "COLOR",
                    "frame_aggregate_mode": "full_frame",
                    "enable_frame_sync": "true",
                    "enable_point_cloud": enable_point_cloud,
                    "enable_colored_point_cloud": enable_colored_point_cloud,
                    "enable_publish_extrinsic": "true",
                }.items(),
            ),
        ]
    )
