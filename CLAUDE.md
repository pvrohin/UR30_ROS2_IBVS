# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace overview

This is a ROS 2 (Jazzy) `colcon` workspace, not a single git repo — the top-level directory
(`/home/pvrohin/ur_ws_ah`) is unversioned, and version control lives inside each package under
`src/` (`ur_description.gz/.git` and `ur_simulation_gz/.git`). When committing, `cd` into the
relevant package first.

Two packages are checked out under `src/`:

- **`ur_description.gz`** (ROS package name: `ur_description`) — URDF/xacro robot description and
  meshes for Universal Robots arms (UR3 through UR30, plus the `e`-series). Upstream:
  `UniversalRobots/Universal_Robots_ROS2_Description`.
- **`ur_simulation_gz`** — Gazebo (`gz_ros2_control`) simulation launch files, controller configs,
  and the ros2_control-augmented URDF that layers on top of `ur_description`. Upstream:
  `UniversalRobots/Universal_Robots_ROS2_GZ_Simulation`.

`build/`, `install/`, and `log/` are colcon-generated output — never hand-edit files there.

## Common commands

Build (from workspace root, `/home/pvrohin/ur_ws_ah`):
```bash
colcon build --symlink-install
source install/setup.bash
```

Build a single package:
```bash
colcon build --symlink-install --packages-select ur_description
colcon build --symlink-install --packages-select ur_simulation_gz
```

Run the full test suite for a package:
```bash
colcon test --packages-select ur_description
colcon test --packages-select ur_simulation_gz
colcon test-result --verbose
```

Run a single test directly with pytest (faster iteration, no colcon overhead) — requires the
workspace to be sourced first:
```bash
source install/setup.bash
python3 -m pytest src/ur_description.gz/test/test_ur_urdf_xacro.py -k ur5e
python3 -m pytest src/ur_simulation_gz/ur_simulation_gz/test/test_description.py
```

Gazebo integration test (`test_gz.py`) is a `launch_test` gated by the CMake option
`UR_SIM_INTEGRATION_TESTS`, which defaults **OFF** (starting/stopping `gz sim` doesn't shut down
cleanly yet). Enable explicitly when needed:
```bash
colcon build --packages-select ur_simulation_gz --cmake-args -DUR_SIM_INTEGRATION_TESTS=ON
```

Visualize a single arm description (no simulation):
```bash
ros2 launch ur_description view_ur.launch.xml ur_type:=ur5e
```

Launch the Gazebo simulation with ros2_control:
```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py ur_type:=ur5e
```

Launch the simulation with MoveIt (requires `ur_moveit_config`, not part of this workspace):
```bash
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
```

Linting/formatting is driven by `pre-commit` inside each package (`black --line-length=99/100`,
`flake8 --ignore=E501`, `ament_lint_cmake`, `ament_copyright`, `codespell`, `doc8` for RST):
```bash
cd src/ur_description.gz && pre-commit run -a
cd src/ur_simulation_gz && pre-commit run -a
```

## Architecture

### Description → simulation layering

The two packages compose in layers, and understanding the chain matters when tracing where a
joint, link, or parameter actually comes from:

1. **`ur_description.gz/urdf/ur_macro.xacro`** defines the `ur_robot` xacro macro — the base
   kinematic chain (links/joints from `base_link` through `tool0`/`flange`/`ft_frame`) shared by
   every UR variant. It takes no hardcoded dimensions; all physical/visual/kinematic/joint-limit
   values are macro parameters pointing at per-robot-type YAML files.
2. **`ur_description.gz/config/<ur_type>/{physical_parameters,visual_parameters,default_kinematics,joint_limits}.yaml`**
   supply those values per robot type (e.g. `ur5e`, `ur10`, `ur20`). Adding/adjusting a robot
   variant means editing this YAML, not the macro.
3. **`ur_description.gz/urdf/inc/*.xacro`** are macro helpers included by `ur_macro.xacro`:
   `ur_common.xacro` (YAML loading, mesh/inertia helper macros), `ur_joint_control.xacro`
   (ros2_control joint interface declarations shared across hardware backends),
   `ur_transmissions.xacro`, `ur_sensors.xacro`.
4. **`ur_description.gz/urdf/ur.urdf.xacro`** is the top-level, hardware-agnostic robot file
   (used by `view_ur.launch.xml` for standalone visualization). `ur_mocked.urdf.xacro` +
   `ros2_control_mock_hardware.xacro` produce a fake-hardware variant for testing without any
   backend.
5. **`ur_simulation_gz/ur_simulation_gz/urdf/ur_gz.urdf.xacro`** is the Gazebo entry point: it
   includes `ur_description`'s `ur_macro.xacro` for the arm itself, adds a world link + ground
   plane, and includes `ur_gz.ros2_control.xacro` (this package) to declare the `gz_ros2_control/GazeboSimSystem`
   hardware plugin and the `gz_ros2_control-system` Gazebo plugin, wiring the ros2_control tag set
   from `ur_joint_control.xacro` (over in `ur_description`) into the Gazebo-specific `<ros2_control>`
   block.
6. **`ur_simulation_gz/ur_simulation_gz/config/ur_controllers.yaml`** declares the
   `controller_manager` and controllers (`joint_state_broadcaster`, `joint_trajectory_controller`,
   `forward_position_controller`, `forward_velocity_controller`, `io_and_status_controller`,
   `speed_scaling_state_broadcaster`) that get spawned against that ros2_control instance.

So: robot-type-specific numbers live in `ur_description.gz/config/<ur_type>/`, the kinematic
structure lives in `ur_macro.xacro`, and everything Gazebo/ros2_control-specific (plugins,
controller wiring) lives in `ur_simulation_gz`.

### Launch flow

`ur_sim_control.launch.py` is the core simulation launch file: it xacro-expands
`ur_gz.urdf.xacro` (embedding the `simulation_controllers` YAML path as a plugin parameter),
starts `robot_state_publisher`, spawns the robot into `gz sim` via `ros_gz_sim`'s `create`
executable, bridges `/clock`, and spawns `joint_state_broadcaster` followed by whichever
`initial_joint_controller` was requested (default `joint_trajectory_controller`) once the state
broadcaster exits successfully (`OnProcessExit` event handler) — RViz is delayed the same way.
`ur_sim_moveit.launch.py` simply includes `ur_sim_control.launch.py` (with RViz disabled) plus an
external `ur_moveit_config` package's MoveIt launch file; that package is not part of this
workspace so MoveIt launches will fail unless it's installed separately.

Both launch files expose `ur_type` as a `choices=[...]` launch argument — new robot variants must
be added to that list in **both** `ur_sim_control.launch.py` and `ur_sim_moveit.launch.py`, and
need a matching `ur_description.gz/config/<ur_type>/` directory.

### Testing patterns

- `ur_description.gz/test/test_ur_urdf_xacro.py` is parametrized over every `ur_type` × prefix
  (`""`, `"my_ur_"`) and validates by shelling out to `xacro` then `check_urdf` on the result —
  it's a structural/syntax check, not a values check.
- `ur_simulation_gz/.../test/test_description.py` does the same, but through `ur_gz.urdf.xacro`
  (the Gazebo-layered file) rather than the plain description.
- `ur_simulation_gz/.../test/test_gz.py` is a `launch_test`/`unittest` that actually boots Gazebo
  headless, spawns the robot, and sends a `FollowJointTrajectory` action goal — this is the test
  gated behind `UR_SIM_INTEGRATION_TESTS` mentioned above.
