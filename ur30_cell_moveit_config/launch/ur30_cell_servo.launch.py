import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # servo_node parses robot_description on its own; it is NOT shared with the
    # robot_description that ur30_cell_bringup gives Gazebo/robot_state_publisher.
    # The xacro mappings below must match what that launch file passes
    # (name, ur_type, tf_prefix), or the two models silently diverge.
    description_xacro = os.path.join(
        get_package_share_directory("ur30_cell_description"), "urdf", "ur30_cell.xacro"
    )
    moveit_config = (
        MoveItConfigsBuilder("ur30_cell", package_name="ur30_cell_moveit_config")
        .robot_description(
            file_path=description_xacro,
            mappings={"name": "ur30_cell", "ur_type": "ur30", "tf_prefix": ""},
        )
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
        .yaml("config/ur30_cell_servo.yaml")
        .to_dict()
    }

    # Needed by the AccelerationLimitedPlugin smoothing filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "ur_manipulator"}

    servo_node = Node(
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
            {"use_sim_time": True},
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])
