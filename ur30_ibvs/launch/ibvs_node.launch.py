from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start only the IBVS node, for whatever already provides the robot and the camera."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                description="YAML loaded over config/ur30_ibvs.yaml, with this site's values.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="ur30_ibvs",
                executable="ibvs_node",
                name="ibvs_state_machine",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("ur30_ibvs"), "config", "ur30_ibvs.yaml"]
                    ),
                    LaunchConfiguration("params_file"),
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
