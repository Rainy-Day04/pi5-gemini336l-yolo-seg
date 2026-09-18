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
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=EnvironmentVariable(
                    "GEMINI336L_MODEL", default_value="yolo26n-seg.pt"
                ),
            ),
            DeclareLaunchArgument("align_mode", default_value="HW"),
            DeclareLaunchArgument("enable_point_cloud", default_value="true"),
            DeclareLaunchArgument("enable_colored_point_cloud", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "camera.launch.py"])
                ),
                launch_arguments={
                    "align_mode": align_mode,
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
                launch_arguments={"model": model}.items(),
            ),
        ]
    )
