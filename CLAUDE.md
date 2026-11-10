# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace overview

This is a ROS 2 (Jazzy) `colcon` workspace for a UR30 workcell: UR30 on a pedestal, a table,
a fixtured automotive door panel, and an eye-in-hand Orbbec Gemini 335Lg camera. It runs in Gazebo,
and is being prepared for the real arm with the IBVS node in a container on a Jetson AGX Orin.
See `README.md` for setup and run instructions, and `docs/DEPLOYMENT.md` for the real robot.

Version control: the workspace root (`/home/pvrohin/ur_ws_ah`) is unversioned, but `src/` is itself the
git repo (`https://github.com/pvrohin/UR30_ROS2_IBVS`). Run git commands from `src/`.

Packages under `src/` (all ours):

- **`ur30_cell_description/`** — xacro for the cell: `pedestal.xacro`, `table.xacro`, `door_panel.xacro`,
  `camera_mount.xacro` (mount only), `camera_gz_sensor.xacro` (Gazebo sensor). `ur30_cell_common.xacro`
  holds what sim and hardware share (world, pedestal, UR30, table, camera tree) and is included, not a macro.
  `ur30_cell.xacro` = common + panel + Gazebo; `ur30_cell_real.urdf.xacro` = common + the UR driver's
  `ros2_control` (args `kinematics_params`, `camera_mount_params`; `config/camera_mount_nominal.yaml` is a
  placeholder mount).
- **`ur30_cell_bringup/`** — simulation only: `launch/ur30_cell.launch.py`, `launch/ur30_ibvs_sim.launch.py`
  (the IBVS demo), `worlds/`, `config/camera_bridge.yaml`, `rviz/`, and the Gazebo-only scripts
  `ibvs_test_runner.py` (random panel poses, back-to-back test cases) and `part_shifter.py` (nudges the
  part during DISPENSE and scores the error, via `gz service .../set_pose`), with their pytests.
- **`ur30_cell_moveit_config/`** — MoveIt Servo setup only (no `move_group`):
  `config/{ur30_cell.srdf.xacro,kinematics.yaml,joint_limits.yaml,ur30_cell_servo.yaml}` and
  `launch/ur30_cell_servo.launch.py` (args `use_sim_time` default true, `description_file`,
  `servo_params_file`, `kinematics_params`, `camera_mount_params`). Servo writes velocities to
  `/forward_velocity_controller/commands`. Smoothing is off, so callers must limit their own twist rates.
- **`ur30_ibvs/`** (C++ with ViSP/OpenCV, no simulator dependency) — image-based visual servoing. Library
  classes in `include/ur30_ibvs` + `src`: `PanelDetector`, `EdgeTracker` (`vpMeLine`), `IbvsController`
  (`vpServo`), `panel_geometry`, `ServoSupervisor`, `virtual_view` (image rotated by k x 90 deg so the
  followed edge is always horizontal), `CornerFinder`, `survey_motion` (survey move duration and refusal),
  `TimingStats`; `IbvsStateMachine` is the node (`ibvs_node`, name `ibvs_state_machine`) with states
  WAIT_READY, IDLE, RESETTING, GO_TO_SURVEY, SEARCH, SWITCH_CONTROL, APPROACH, SERVO, DISPENSE, DONE,
  FAILED. `launch/ibvs_node.launch.py` starts only the node (`params_file`, `use_sim_time` default false).
  Config: `config/ur30_ibvs.yaml` (defaults) and `ur30_ibvs_demo.yaml` (lap overrides: panel rotated
  90 deg, wrist_3 survey -1.8327). See `ur30_ibvs/README.md`.
- **`ur30_real_bringup/`** — the real cell: `launch/robot.launch.py` (UR driver `ur_control.launch.py` +
  Servo, `use_mock_hardware` for testing without a robot), `launch/rsp.launch.py` (the driver's
  `description_launchfile`), `launch/camera.launch.py` (335Lg color only, `publish_tf:=false`),
  `config/ur30_ibvs_real.yaml` (site template, `site_configured: false`), `config/ur30_servo_real.yaml`.
- **`docker/`, `.github/workflows/`** — robot image (hardware packages only), compose files for the Jetson
  and the laptop, host scripts; CI (`ci.yml`) and the arm64 release to GHCR on `v*` tags (`release.yml`).

Source dependencies of the simulation are fetched with `vcs import . < ur30.repos` into `ur_description_gz/`
(ROS package `ur_description`) and `ur_simulation_gz/`; both are git-ignored. `orbbec_description` and, for
the real robot, `ur_description` come from apt via rosdep. Do not edit fetched code; put cell-specific
changes in our packages. Keep `docker/Dockerfile`'s package list in step with the hardware packages.

`build/`, `install/`, and `log/` are colcon-generated output — never hand-edit files there.

`main` contains the cell, `ur30_cell_moveit_config` and `ur30_ibvs`. The feature branches
`custom_ur_30_sim`, `custom_ur30_visp` and `dispense_lap` are merged into `main` and kept for history.
`jetson_deployment` holds the sim/hardware split, `ur30_real_bringup`, Docker and CI (not merged yet).

## Common commands

Build (from workspace root, `/home/pvrohin/ur_ws_ah`):
```bash
colcon build --symlink-install --allow-overriding ur_description --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Always build in Release: without it the vision code is unoptimised. The vision packages also need
`ros-jazzy-moveit-servo` and `ros-jazzy-visp`.

`--allow-overriding ur_description` is required: a system `ros-jazzy-ur-description` can coexist with the
vendored copy, which has the same package name. If colcon complains about a renamed or stale package,
remove `build/ur_description` and `install/ur_description` and rebuild.

Launch the cell (Gazebo + RViz + controllers + camera bridge):
```bash
ros2 launch ur30_cell_bringup ur30_cell.launch.py
# headless: gazebo_gui:=false launch_rviz:=false
```

Launch arguments: `ur_type` (default `ur30`; the cell geometry is sized for its ~1.3 m reach), `gazebo_gui`,
`launch_rviz`, `world_file`, `description_file`, `rviz_config_file`, `camera_bridge_config_file`,
`controllers_file`, `initial_joint_controller` (default `joint_trajectory_controller`),
`activate_joint_controller`, `tf_prefix`, `safety_*`.

Tests: `ur30_ibvs` has 81 gtests, `ur30_cell_bringup` 23 pytests (104 in total):
```bash
colcon test --packages-select ur30_ibvs ur30_cell_bringup && colcon test-result --verbose
```
Vision demo (Gazebo + Servo + IBVS node):
```bash
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py                      # hold 0.2 m above the +y edge
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true           # dispense lap + nudges
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true shifts:=false gazebo_gui:=false launch_rviz:=false
```
Real-robot stack without a robot (mock hardware), and the robot image:
```bash
ros2 launch ur30_real_bringup robot.launch.py robot_ip:=127.0.0.1 use_mock_hardware:=true
docker build -f docker/Dockerfile --target test .     # from src/
```
Other args: `test_cases`, `test_seed`, `shift_seed`, `bridge_depth`, `run_ibvs`. With `demo` or `test_cases`
the panel is spawned as a separate model (`door_panel_test`, `include_panel:=false` on the cell launch) so it
can be moved. Do not run `pkill -f` with a pattern that also matches your own shell command.

Tests of the vendored packages: source the workspace, then:
```bash
colcon test --packages-select ur_description ur_simulation_gz
colcon test-result --verbose
python3 -m pytest src/ur_description_gz/test/test_ur_urdf_xacro.py -k ur30
python3 -m pytest src/ur_simulation_gz/ur_simulation_gz/test/test_description.py
```
The Gazebo integration test `test_gz.py` is gated by the CMake option `UR_SIM_INTEGRATION_TESTS`
(default OFF; `gz sim` does not shut down cleanly in it).

Vendored repos use `pre-commit` (`black`, `flake8`, `ament_lint_cmake`, `codespell`, ...). Run it inside a
vendored directory only if you changed it: `cd src/ur_description_gz && pre-commit run -a`.

## Architecture

### Model assembly

`ur30_cell_description/urdf/ur30_cell.xacro` is the single robot description. It:

1. includes `ur_description`'s `ur_macro.xacro` (the `ur_robot` macro) and `ur_simulation_gz`'s
   `ur_gz.ros2_control.xacro` (`ur_ros2_control` macro),
2. places the pedestal at the world origin (height 0.5, radius 0.15) and the UR30 on `pedestal_top_link`,
3. places the table at world (0.9, 0, 0.75) (1.2 x 1.2 m, top at z = 0.75),
4. places the door panel (0.9 x 0.55 x 0.03 m, armrest, speaker, four corner clamps) on `table_top_link`
   at (-0.1, 0, 0.04), giving a world position (0.8, 0, 0.79) and a top face at z = 0.805,
5. mounts the camera via `eye_in_hand_camera` on `tool0`,
6. adds the `gz_ros2_control-system` Gazebo plugin.

UR-type-specific numbers live in `ur_description_gz/config/<ur_type>/*.yaml`; the kinematic chain lives in
`ur_macro.xacro`. Robot link names are the stock unprefixed UR ones (`base_link` ... `tool0`).

`camera_mount.xacro` includes the Orbbec URDF (`gemini_335_Lg.urdf.xacro`) directly, which is not a macro,
so **only one camera can exist per model**. Also note gz-sim behaviour that shaped the file:

- gz-sim lumps fixed-joint chains onto the parent moving link and stamps sensor messages with
  `frame_id` `<model>/<lumped_link>/<sensor>` (here `ur30_cell/wrist_3_link/gemini_335_Lg_rgbd`). A
  zero-offset link with that name is added to the URDF so TF/RViz can resolve it.
- Gazebo cameras look along the **sensor's +X axis**, so the sensor is attached to `camera_color_frame`
  (+X forward), not `camera_color_optical_frame` (+Z forward), which would look sideways.
- `<gazebo><material>Gazebo/...</material>` tags in the xacro are classic-Gazebo syntax that gz-sim does not
  honour properly.

### Launch flow

`ur30_cell_bringup/launch/ur30_cell.launch.py` is self-contained (it does not include
`ur_sim_control.launch.py`). It expands `ur30_cell.xacro` with xacro args (`name:=ur30_cell`, `ur_type`,
`tf_prefix`, `simulation_controllers`, `safety_*`), then starts:

- `robot_state_publisher` (sim time),
- Gazebo via `ros_gz_sim`'s `gz_sim.launch.py` with the world file, and `create` to spawn the robot from the
  URDF string,
- a `/clock` bridge and a camera bridge (`config/camera_bridge.yaml`: color image and camera_info; depth and
  points are in `config/camera_depth_bridge.yaml`, only with `bridge_depth:=true`),
- `joint_state_broadcaster`, then RViz once the broadcaster spawner exits, and the initial joint controller
  (`joint_trajectory_controller`) as active or stopped depending on `activate_joint_controller`.

It appends the parent of `orbbec_description`'s share dir to `GZ_SIM_RESOURCE_PATH` so Gazebo can resolve the
`package://` mesh URIs.

Controllers are declared in `ur_simulation_gz/ur_simulation_gz/config/ur_controllers.yaml`, but the launch
spawns only `joint_state_broadcaster` and the one initial controller. Anything else (for example
`forward_velocity_controller`) must be spawned by whoever needs it.

If the launch file is changed to include Servo/other stacks, both must expand the xacro with identical
`ur_type`/`tf_prefix`/`name`, or the models silently diverge.

### Camera topics

`/camera/color/image_raw` and `/camera/color/camera_info` always; `/camera/depth/image_raw` and
`/camera/depth/points` only with `bridge_depth:=true`. Depth and points at 30 Hz are expensive (tens of
percent of a CPU and camera frame stalls on modest machines), and the IBVS pipeline does not use them.

### Vision stack (ur30_ibvs)

`ur30_cell_bringup/launch/ur30_ibvs_sim.launch.py` includes the cell launch and `ur30_cell_servo.launch.py`, spawns
`forward_velocity_controller` inactive, and starts `ibvs_node`. The node switches between
`joint_trajectory_controller` (survey, reset) and `forward_velocity_controller` (Servo twists) via
`/controller_manager/switch_controller`. Twists go to `/servo_node/delta_twist_cmds`; Servo drops commands
older than 0.1 s, so the node publishes at 50 Hz and sends a zero twist when no new image arrives.
Servo ignores twists below roughly 0.5-1 mm/s, which limits the residual lateral error to about 1 mm.
Pass `use_sim_time` consistently: true for every node in Gazebo, false on the real robot (no `/clock`).

On the real robot (`ur30_real_bringup`) the driver starts `scaled_joint_trajectory_controller` and loads
`forward_velocity_controller` inactive itself; do not spawn controllers there. The site file must set
`jtc_action`/`position_controller` to the scaled controller and `robot_program_topic` to
`/io_and_status_controller/robot_program_running` (latched, so the node subscribes transient-local).
Safety parameters of the node: `site_configured` (false refuses to move), `auto_start` (default false;
the sim launch sets it), `survey_max_joint_speed` and `survey_max_joint_distance` (survey move duration
scales with the largest joint move; larger moves FAIL).
