from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_ibvs = LaunchConfiguration("run_ibvs")

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur30_cell_bringup"), "launch", "ur30_cell.launch.py"]
            )
        ),
        launch_arguments={"gazebo_gui": gazebo_gui, "launch_rviz": launch_rviz}.items(),
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur30_cell_moveit_config"), "launch", "ur30_cell_servo.launch.py"]
            )
        )
    )

    # ur30_cell_bringup only loads joint_state_broadcaster and
    # joint_trajectory_controller. Load the velocity controller inactive so the
    # state machine can switch to it without a load step.
    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "--inactive", "-c", "/controller_manager"],
    )

    ibvs_node = Node(
        package="ur30_ibvs",
        executable="ibvs_node",
        name="ibvs_state_machine",
        output="screen",
        parameters=[
            PathJoinSubstitution([FindPackageShare("ur30_ibvs"), "config", "ur30_ibvs.yaml"]),
            {"use_sim_time": True},
        ],
        condition=IfCondition(run_ibvs),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "run_ibvs",
                default_value="true",
                description="false brings up sim + Servo only, for testing stages in isolation.",
            ),
            cell,
            servo,
            velocity_controller,
            ibvs_node,
        ]
    )
