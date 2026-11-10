from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def share_file(package, *path):
    return PathJoinSubstitution([FindPackageShare(package), *path])


def generate_launch_description():
    """The real UR30: driver, controllers, robot_state_publisher and MoveIt Servo.

    The driver loads forward_velocity_controller inactive and starts
    scaled_joint_trajectory_controller by itself, so nothing else spawns controllers.
    The robot does not move until the External Control program runs on the pendant
    (headless_mode stays false) and ibvs_node is told to start.
    """
    kinematics = LaunchConfiguration("kinematics_params")
    camera_mount = LaunchConfiguration("camera_mount_params")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(share_file("ur_robot_driver", "launch", "ur_control.launch.py")),
        launch_arguments={
            "ur_type": "ur30",
            "robot_ip": LaunchConfiguration("robot_ip"),
            "description_launchfile": share_file("ur30_real_bringup", "launch", "rsp.launch.py"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "launch_rviz": "false",
            # Read by rsp.launch.py.
            "kinematics_params": kinematics,
            "camera_mount_params": camera_mount,
        }.items(),
    )

    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            share_file("ur30_cell_moveit_config", "launch", "ur30_cell_servo.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "description_file": share_file(
                "ur30_cell_description", "urdf", "ur30_cell_real.urdf.xacro"
            ),
            "servo_params_file": share_file("ur30_real_bringup", "config", "ur30_servo_real.yaml"),
            "kinematics_params": kinematics,
            "camera_mount_params": camera_mount,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP address of the robot."),
            DeclareLaunchArgument(
                "kinematics_params",
                default_value=share_file("ur_description", "config", "ur30", "default_kinematics.yaml"),
                description="This robot's calibration from ur_calibration. The default is the "
                "nominal model: usable for tests, not for accurate work on a real arm.",
            ),
            DeclareLaunchArgument(
                "camera_mount_params",
                default_value=share_file(
                    "ur30_cell_description", "config", "camera_mount_nominal.yaml"
                ),
                description="tool0 -> camera_link. The default is a placeholder, not a real mount.",
            ),
            DeclareLaunchArgument(
                "use_mock_hardware",
                default_value="false",
                description="true replaces the arm with ros2_control's mock, for testing the "
                "launch files and controllers with no robot.",
            ),
            driver,
            servo,
        ]
    )
