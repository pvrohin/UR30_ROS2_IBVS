# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace overview

This is a ROS 2 (Jazzy) `colcon` workspace for a simulated UR30 workcell: UR30 on a pedestal, a table,
a fixtured automotive door panel, and an eye-in-hand Orbbec Gemini 335Lg camera, run in Gazebo.
See `README.md` for setup and run instructions.

Version control: the workspace root (`/home/pvrohin/ur_ws_ah`) is unversioned, but `src/` is itself the
git repo (`https://github.com/pvrohin/UR30_ROS2_IBVS`). Run git commands from `src/`.

Five directories live under `src/`:

- **`ur30_cell_description/`** (ROS package `ur30_cell_description`, ours) — xacro for the cell:
  `pedestal.xacro`, `table.xacro`, `door_panel.xacro`, `camera_mount.xacro`, assembled by `ur30_cell.xacro`.
- **`ur30_cell_bringup/`** (ROS package `ur30_cell_bringup`, ours) — `launch/ur30_cell.launch.py`,
  `worlds/ur30_cell.world.sdf`, `config/camera_bridge.yaml`, `rviz/ur30_cell.rviz`.
- **`ur_description_gz/`** (ROS package name **`ur_description`**) — vendored
  `UniversalRobots/Universal_Robots_ROS2_Description`, branch `jazzy`, pinned at `3924298`.
- **`ur_simulation_gz/`** (ROS package `ur_simulation_gz` lives one level down, in
  `ur_simulation_gz/ur_simulation_gz/`) — vendored `UniversalRobots/Universal_Robots_ROS2_GZ_Simulation`,
  branch `ros2`, pinned at `5f0161a`.
- **`orbbec_description/`** (ROS package in `orbbec_description/orbbec_description/`) — sparse checkout of
  `orbbec/OrbbecSDK_ROS2` branch `v2-main` (only the description package), pinned at `8e7cad2`.

The three vendored directories are **gitlinks with no `.gitmodules`**: a fresh clone leaves them empty and
`git submodule update` does nothing. They must be cloned by hand (commands in `README.md`). Each one is its
own git repo, so commits inside them are separate from commits in `src/`. Do not edit vendored code unless
asked; put cell-specific changes in `ur30_cell_*`.

`build/`, `install/`, and `log/` are colcon-generated output — never hand-edit files there.

The `custom_ur30_visp` branch adds `ur30_ibvs` and `ur30_cell_moveit_config` (ViSP IBVS with MoveIt Servo)
and fixes that are not on `main`. Its `CLAUDE.md`/README may differ; check which branch you are on.

## Common commands

Build (from workspace root, `/home/pvrohin/ur_ws_ah`):
```bash
colcon build --symlink-install --packages-up-to ur30_cell_bringup --allow-overriding ur_description
source install/setup.bash
```

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

Tests exist only in the vendored packages; the cell packages have none. To run them, source the workspace, then:
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
- Gazebo cameras look along the **sensor's +X axis**. On `main` the sensor is attached to
  `camera_color_optical_frame` (+Z forward), so the simulated camera looks sideways. This is fixed on
  `custom_ur30_visp` (sensor on `camera_color_frame`). Keep this in mind before trusting `main`'s images.
- `<gazebo><material>Gazebo/...</material>` tags in the xacro are classic-Gazebo syntax that gz-sim does not
  honour properly.

### Launch flow

`ur30_cell_bringup/launch/ur30_cell.launch.py` is self-contained (it does not include
`ur_sim_control.launch.py`). It expands `ur30_cell.xacro` with xacro args (`name:=ur30_cell`, `ur_type`,
`tf_prefix`, `simulation_controllers`, `safety_*`), then starts:

- `robot_state_publisher` (sim time),
- Gazebo via `ros_gz_sim`'s `gz_sim.launch.py` with the world file, and `create` to spawn the robot from the
  URDF string,
- a `/clock` bridge and a camera bridge (`config/camera_bridge.yaml`: color image, depth image, points,
  camera_info),
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

`/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/depth/points`, `/camera/color/camera_info`.
Depth and points at 30 Hz are expensive (tens of percent of a CPU and camera frame stalls on modest
machines); `custom_ur30_visp` makes them an optional separate bridge.
