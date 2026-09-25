from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    run_ibvs = LaunchConfiguration("run_ibvs")
    bridge_depth = LaunchConfiguration("bridge_depth")
    test_cases = LaunchConfiguration("test_cases")
    test_seed = LaunchConfiguration("test_seed")

    # test_cases > 0: the panel is a separate model that the test runner moves between
    # cases, and the IBVS node waits for the runner's ~/start instead of starting alone.
    testing = PythonExpression(["int('", test_cases, "') > 0"])
    not_testing = PythonExpression(["int('", test_cases, "') <= 0"])

    cell = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ur30_cell_bringup"), "launch", "ur30_cell.launch.py"]
            )
        ),
        launch_arguments={
            "gazebo_gui": gazebo_gui,
            "launch_rviz": launch_rviz,
            "bridge_depth": bridge_depth,
            "include_panel": PythonExpression(["'false' if ", testing, " else 'true'"]),
        }.items(),
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
            {"auto_start": ParameterValue(not_testing, value_type=bool)},
        ],
        condition=IfCondition(run_ibvs),
    )

    # The panel as its own model, at the nominal pose until the runner moves it.
    panel_model = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ur30_cell_description"), "urdf", "door_panel_model.xacro"]
            ),
        ]
    )
    spawn_panel = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", panel_model,
            "-name", "door_panel_test",
            "-x", "0.8", "-y", "0.0", "-z", "0.79",
        ],
        condition=IfCondition(testing),
    )

    test_runner = Node(
        package="ur30_ibvs",
        executable="ibvs_test_runner.py",
        name="ibvs_test_runner",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"num_cases": ParameterValue(test_cases, value_type=int)},
            {"seed": ParameterValue(test_seed, value_type=int)},
        ],
        condition=IfCondition(PythonExpression([testing, " and ", "'", run_ibvs, "' == 'true'"])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "bridge_depth",
                default_value="false",
                description="Also bridge the depth image and point cloud (CPU-heavy, not "
                "used by the IBVS pipeline; enable them to view in RViz).",
            ),
            DeclareLaunchArgument(
                "run_ibvs",
                default_value="true",
                description="false brings up sim + Servo only, for testing stages in isolation.",
            ),
            DeclareLaunchArgument(
                "test_cases",
                default_value="0",
                description="Run this many test cases back to back, each with the door panel "
                "at a new random pose (0 = a single run with the panel at its nominal pose).",
            ),
            DeclareLaunchArgument(
                "test_seed",
                default_value="-1",
                description="Random seed for the test poses (-1 = pick one; it is logged).",
            ),
            cell,
            servo,
            velocity_controller,
            ibvs_node,
            spawn_panel,
            test_runner,
        ]
    )
