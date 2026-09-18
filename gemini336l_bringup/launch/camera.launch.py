"""Launch the official Orbbec wrapper with Pi 5-safe defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    depth_registration = LaunchConfiguration("depth_registration")
    align_mode = LaunchConfiguration("align_mode")
    enable_frame_sync = LaunchConfiguration("enable_frame_sync")
    enable_ir_auto_exposure = LaunchConfiguration("enable_ir_auto_exposure")
    enable_noise_removal_filter = LaunchConfiguration("enable_noise_removal_filter")
    enable_hardware_noise_removal_filter = LaunchConfiguration(
        "enable_hardware_noise_removal_filter"
    )
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    enable_colored_point_cloud = LaunchConfiguration("enable_colored_point_cloud")
    driver_launch = PathJoinSubstitution(
        [FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("depth_registration", default_value="true"),
            DeclareLaunchArgument(
                "align_mode",
                default_value="SW",
                description="SW is safer with old 336L firmware; use HW after updating firmware.",
            ),
            DeclareLaunchArgument("enable_frame_sync", default_value="false"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value="false"),
            DeclareLaunchArgument("enable_noise_removal_filter", default_value="false"),
            DeclareLaunchArgument(
                "enable_hardware_noise_removal_filter", default_value="false"
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
                    "depth_registration": depth_registration,
                    "align_mode": align_mode,
                    "align_target_stream": "COLOR",
                    "frame_aggregate_mode": "full_frame",
                    "enable_frame_sync": enable_frame_sync,
                    "enable_ir_auto_exposure": enable_ir_auto_exposure,
                    "enable_noise_removal_filter": enable_noise_removal_filter,
                    "enable_hardware_noise_removal_filter": enable_hardware_noise_removal_filter,
                    "enable_point_cloud": enable_point_cloud,
                    "enable_colored_point_cloud": enable_colored_point_cloud,
                    "enable_publish_extrinsic": "true",
                }.items(),
            ),
        ]
    )
