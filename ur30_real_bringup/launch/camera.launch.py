from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """The Gemini 335Lg with what the IBVS pipeline needs: the color stream only.

    The default camera_name gives /camera/color/image_raw and /camera/color/camera_info,
    the topics ur30_ibvs.yaml expects. publish_tf is off because robot_state_publisher already
    publishes these camera frames from the URDF, and two publishers of one frame conflict.
    """
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"]
                    )
                ),
                launch_arguments={
                    "color_width": "1280",
                    "color_height": "720",
                    "color_fps": "30",
                    "enable_depth": "false",
                    "enable_point_cloud": "false",
                    "publish_tf": "false",
                }.items(),
            )
        ]
    )
