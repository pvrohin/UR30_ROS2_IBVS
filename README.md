# UR30 workcell simulation (ROS 2 Jazzy + Gazebo)

A simulated robot workcell for developing vision-guided manipulation, with the
aim of eventually running the same code on a real UR30 with an Orbbec Gemini
335Lg camera on a Jetson Orin AGX.

The cell contains:

- a **UR30** arm on a cylindrical pedestal at the world origin,
- a **table** (1.2 x 1.2 m, top at z = 0.75 m) in front of the robot,
- an **automotive door panel** (0.9 x 0.55 x 0.03 m, with armrest, speaker grille
  and four corner clamps) fixtured on the table,
- an **Orbbec Gemini 335Lg** mounted eye-in-hand on the tool flange, streaming
  into Gazebo and RViz.

The panel and table are simple primitives. No external meshes are needed except
the UR and Orbbec ones.

This is the `main` branch: the cell only. The visual servoing work (ViSP IBVS)
is on a separate branch, see [Visual servoing branch](#visual-servoing-branch).

## Repository layout

| Path | What it is |
|---|---|
| `ur30_cell_description/` | The cell's xacro: pedestal, table, door panel, camera mount, and `ur30_cell.xacro` that assembles them. |
| `ur30_cell_bringup/` | Launch file, RViz config, Gazebo world and camera bridge config. |
| `ur_description_gz/` | Vendored `Universal_Robots_ROS2_Description` (ROS package name `ur_description`). |
| `ur_simulation_gz/` | Vendored `Universal_Robots_ROS2_GZ_Simulation`. |
| `orbbec_description/` | Vendored `orbbec_description` from `OrbbecSDK_ROS2` (URDF and meshes only). |

The three vendored directories are recorded in this repository as git links
(gitlinks) **without a `.gitmodules` file**, so `git clone` leaves them empty and
`git submodule update --init` does nothing. You have to fetch them yourself, see
the next section.

## Setup

Requirements: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (the Jazzy default).

### 1. Clone into a workspace

```bash
mkdir -p ~/ur_ws_ah && cd ~/ur_ws_ah
git clone https://github.com/pvrohin/UR30_ROS2_IBVS.git src
cd src
```

### 2. Fetch the three vendored repositories

These are the exact revisions the cell was built and tested against.

```bash
# UR robot description (branch jazzy)
git clone --branch jazzy https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git ur_description_gz
git -C ur_description_gz checkout 3924298

# UR Gazebo simulation (branch ros2)
git clone --branch ros2 https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git ur_simulation_gz
git -C ur_simulation_gz checkout 5f0161a

# Orbbec description only (sparse checkout, the full repo is large)
git clone --filter=blob:none --no-checkout --branch v2-main https://github.com/orbbec/OrbbecSDK_ROS2.git orbbec_description
git -C orbbec_description sparse-checkout init --cone
git -C orbbec_description sparse-checkout set orbbec_description
git -C orbbec_description checkout 8e7cad2
```

Check: `ls orbbec_description/orbbec_description` should show `launch`, `meshes`,
`package.xml` and so on. `git status` in `src/` should stay clean.

### 3. Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2
cd ~/ur_ws_ah
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # picks up anything missing
```

### 4. Build

```bash
cd ~/ur_ws_ah
colcon build --symlink-install --packages-up-to ur30_cell_bringup --allow-overriding ur_description
source install/setup.bash
```

`--allow-overriding ur_description` is needed because a system-installed
`ros-jazzy-ur-description` may exist next to the vendored copy, which has the
same package name. The workspace copy must win.

## Run

```bash
ros2 launch ur30_cell_bringup ur30_cell.launch.py
```

This starts Gazebo with the cell, `robot_state_publisher`, the ros2_control
controllers (`joint_state_broadcaster`, then `joint_trajectory_controller`), the
camera bridge and RViz. RViz shows the robot model, TF and the camera images.

## Geometry (world frame)

| Item | Pose |
|---|---|
| Pedestal | height 0.5 m, radius 0.15 m, at the origin |
| Robot base | (0, 0, 0.5) |
| Table top centre | (0.9, 0, 0.75) |
| Door panel centre | (0.8, 0, 0.79), top face at z = 0.805 |

## Known problems on this branch

- **The simulated camera looks sideways.** The Gazebo sensor is attached to
  `camera_color_optical_frame`, but Gazebo cameras look along the sensor's +X
  axis, so the image shows the floor and horizon instead of the table. Fixed on
  `custom_ur30_visp` (attach the sensor to `camera_color_frame`).
- Depth and point cloud are bridged at 30 Hz. This costs a lot of CPU and makes
  the camera stall on modest machines. The branch below makes it optional.
- The vendored repositories are not set up as real submodules (see above).
- `CLAUDE.md` still describes an earlier layout (`ur_description.gz`, an
  unversioned workspace root). Treat it as out of date.

## Visual servoing branch

Image-based visual servoing (IBVS) with [ViSP](https://github.com/lagadic/visp)
lives on the branch `custom_ur30_visp`, not on `main`. It adds:

- `ur30_ibvs`: a C++ node that surveys the table, detects the door panel,
  approaches it, then tracks one long edge with ViSP (`vpMeLine`, `vpServo`) at
  0.2 m standoff. It has unit tests and a debug image topic.
- `ur30_cell_moveit_config`: MoveIt Servo configuration, used to send twists to
  the arm.
- the camera fix and the optional depth bridge mentioned above.

```bash
git checkout custom_ur30_visp
# ...repeat the vendored-repo setup if not done yet, install ros-jazzy-visp and ros-jazzy-moveit-servo
colcon build --symlink-install --allow-overriding ur_description
ros2 launch ur30_ibvs ur30_ibvs.launch.py
```

Status: verified end to end in headless Gazebo (the panel is found at its true
pose and the camera settles within about 1 mm of the target), and the failure
paths were tested. The Gazebo and RViz GUIs and real hardware have **not** been
tested.

## License

The repository contains a `LICENSE` file with the GPLv3 text, but the packages
`ur30_cell_description` and `ur30_cell_bringup` declare `BSD-3-Clause` in their
`package.xml`. This mismatch is unresolved; decide on one before distributing.
The vendored repositories keep their own licenses (Universal Robots: BSD-3-Clause,
OrbbecSDK_ROS2: Apache-2.0).
