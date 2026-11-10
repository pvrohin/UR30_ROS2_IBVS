import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context):
    # servo_node parses robot_description on its own; it is NOT shared with the
    # robot_description given to Gazebo or the driver and robot_state_publisher.
    # The xacro mappings below must match what those launch files pass
    # (name, ur_type, tf_prefix, calibration files), or the models silently diverge.
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    mappings = {"name": "ur30_cell", "ur_type": "ur30", "tf_prefix": ""}
    for name in ("kinematics_params", "camera_mount_params"):
        if arg(name):
            mappings[name] = arg(name)

    moveit_config = (
        MoveItConfigsBuilder("ur30_cell", package_name="ur30_cell_moveit_config")
        .robot_description(file_path=arg("description_file"), mappings=mappings)
        .robot_description_semantic(file_path="config/ur30_cell.srdf.xacro")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        # Servo does no planning. Naming one pipeline stops the builder from
        # loading every default pipeline, which would pull in Pilz and require
        # a pilz_cartesian_limits.yaml this package doesn't have.
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    servo_params = {
        "moveit_servo": ParameterBuilder("ur30_cell_moveit_config")
        .yaml(arg("servo_params_file"))
        .to_dict()
    }

    # Needed by the AccelerationLimitedPlugin smoothing filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ur_manipulator"}

    return [
        Node(
            package="moveit_servo",
            executable="servo_node",
            parameters=[
                servo_params,
                acceleration_filter_update_period,
                planning_group_name,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": arg("use_sim_time").lower() == "true"},
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("ur30_cell_moveit_config")
    description = get_package_share_directory("ur30_cell_description")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="false on the real robot: there is no /clock, and time would stand still.",
            ),
            DeclareLaunchArgument(
                "description_file",
                default_value=os.path.join(description, "urdf", "ur30_cell.xacro"),
            ),
            DeclareLaunchArgument(
                "servo_params_file",
                default_value=os.path.join(share, "config", "ur30_cell_servo.yaml"),
                description="Absolute path of the Servo parameters.",
            ),
            DeclareLaunchArgument(
                "kinematics_params",
                default_value="",
                description="Calibrated kinematics file; empty for the description's default.",
            ),
            DeclareLaunchArgument(
                "camera_mount_params",
                default_value="",
                description="Camera mount file (real robot); empty for the description's default.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
