"""Launch Gemini 336L and perception together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution(
        [FindPackageShare("gemini336l_bringup"), "launch"]
    )
    model = LaunchConfiguration("model")
    align_mode = LaunchConfiguration("align_mode")
    camera_fps = LaunchConfiguration("camera_fps")
    inference_hz = LaunchConfiguration("inference_hz")
    overlay_jpeg_quality = LaunchConfiguration("overlay_jpeg_quality")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=EnvironmentVariable(
                    "GEMINI336L_MODEL", default_value="yolo26n-seg.pt"
                ),
            ),
            DeclareLaunchArgument("depth_registration", default_value="true"),
            DeclareLaunchArgument(
                "camera_fps",
                default_value="30",
                description="Raw RGB and depth stream rate.",
            ),
            DeclareLaunchArgument(
                "inference_hz",
                default_value="5.0",
                description="Independent YOLO segmentation rate.",
            ),
            DeclareLaunchArgument(
                "overlay_jpeg_quality",
                default_value="70",
                description="JPEG quality for the optional compressed overlay stream.",
            ),
            DeclareLaunchArgument("align_mode", default_value="SW"),
            DeclareLaunchArgument("enable_frame_sync", default_value="false"),
            DeclareLaunchArgument("enable_ir_auto_exposure", default_value="false"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "camera.launch.py"])
                ),
                launch_arguments={
                    "depth_registration": LaunchConfiguration("depth_registration"),
                    "color_fps": camera_fps,
                    "depth_fps": camera_fps,
                    "align_mode": align_mode,
                    "enable_frame_sync": LaunchConfiguration("enable_frame_sync"),
                    "enable_ir_auto_exposure": LaunchConfiguration(
                        "enable_ir_auto_exposure"
                    ),
                    "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
                    "enable_colored_point_cloud": LaunchConfiguration(
                        "enable_colored_point_cloud"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "perception.launch.py"])
                ),
                launch_arguments={
                    "model": model,
                    "depth_aligned_to_color": LaunchConfiguration("depth_registration"),
                    "inference_hz": inference_hz,
                    "overlay_jpeg_quality": overlay_jpeg_quality,
                }.items(),
            ),
        ]
    )
