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
    overlay_max_width = LaunchConfiguration("overlay_max_width")
    preview_hz = LaunchConfiguration("preview_hz")
    preview_jpeg_quality = LaunchConfiguration("preview_jpeg_quality")
    preview_max_width = LaunchConfiguration("preview_max_width")
    preview_max_seg_age_sec = LaunchConfiguration("preview_max_seg_age_sec")
    retina_masks = LaunchConfiguration("retina_masks")
    max_detections = LaunchConfiguration("max_detections")
    confidence_threshold = LaunchConfiguration("confidence_threshold")
    run_inference_when_unsubscribed = LaunchConfiguration(
        "run_inference_when_unsubscribed"
    )
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
                default_value="50",
                description="JPEG quality for the optional compressed overlay stream.",
            ),
            DeclareLaunchArgument("overlay_max_width", default_value="640"),
            DeclareLaunchArgument("preview_hz", default_value="0.0"),
            DeclareLaunchArgument("preview_jpeg_quality", default_value="45"),
            DeclareLaunchArgument("preview_max_width", default_value="480"),
            DeclareLaunchArgument("preview_max_seg_age_sec", default_value="0.5"),
            DeclareLaunchArgument("retina_masks", default_value="true"),
            DeclareLaunchArgument("max_detections", default_value="15"),
            DeclareLaunchArgument("confidence_threshold", default_value="0.20"),
            DeclareLaunchArgument(
                "run_inference_when_unsubscribed", default_value="false"
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
                    "overlay_max_width": overlay_max_width,
                    "preview_hz": preview_hz,
                    "preview_jpeg_quality": preview_jpeg_quality,
                    "preview_max_width": preview_max_width,
                    "preview_max_seg_age_sec": preview_max_seg_age_sec,
                    "retina_masks": retina_masks,
                    "max_detections": max_detections,
                    "confidence_threshold": confidence_threshold,
                    "run_inference_when_unsubscribed": (
                        run_inference_when_unsubscribed
                    ),
                }.items(),
            ),
        ]
    )
