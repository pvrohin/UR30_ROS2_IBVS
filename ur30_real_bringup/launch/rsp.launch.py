from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """robot_state_publisher for the real cell; the driver includes this as description_launchfile.

    It takes the place of ur_robot_driver's ur_rsp.launch.py, which cannot pass the
    camera_mount_params xacro argument.
    """
    def xacro_arg(name):
        return [f" {name}:=", LaunchConfiguration(name)]

    def share_file(package, *path):
        return PathJoinSubstitution([FindPackageShare(package), *path])

    command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        share_file("ur30_cell_description", "urdf", "ur30_cell_real.urdf.xacro"),
        " name:=ur30_cell",
        " script_filename:=",
        share_file("ur_client_library", "resources", "external_control.urscript"),
        " input_recipe_filename:=",
        share_file("ur_robot_driver", "resources", "rtde_input_recipe.txt"),
        " output_recipe_filename:=",
        share_file("ur_robot_driver", "resources", "rtde_output_recipe.txt"),
    ]
    for name in (
        "ur_type",
        "robot_ip",
        "kinematics_params",
        "camera_mount_params",
        "use_mock_hardware",
        "mock_sensor_commands",
        "headless_mode",
    ):
        command += xacro_arg(name)

    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur30"),
            DeclareLaunchArgument("robot_ip"),
            # The driver only hands over robot_ip and ur_type. robot.launch.py sets these two
            # further up, which takes precedence over the defaults here.
            DeclareLaunchArgument(
                "kinematics_params",
                default_value=share_file("ur_description", "config", "ur30", "default_kinematics.yaml"),
            ),
            DeclareLaunchArgument(
                "camera_mount_params",
                default_value=share_file(
                    "ur30_cell_description", "config", "camera_mount_nominal.yaml"
                ),
            ),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
            DeclareLaunchArgument("headless_mode", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[
                    {"robot_description": ParameterValue(Command(command), value_type=str)},
                    {"use_sim_time": False},
                ],
            ),
        ]
    )
