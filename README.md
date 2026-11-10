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

On top of the cell sits the visual servoing work: ViSP image-based visual servoing (IBVS)
that finds the panel, follows its edge, and can dispense a lap around its boundary while the
part is nudged at random. See [Visual servoing](#visual-servoing).

## Repository layout

| Path | What it is |
|---|---|
| `ur30_cell_description/` | The cell's xacro: pedestal, table, door panel, camera mount. `ur30_cell_common.xacro` holds what simulation and the real robot share; `ur30_cell.xacro` is the Gazebo model and `ur30_cell_real.urdf.xacro` the real one. |
| `ur30_cell_bringup/` | Simulation: launch files (cell, IBVS demo), RViz config, Gazebo world, camera bridge config, and the Gazebo-only test runner and part shifter. |
| `ur30_cell_moveit_config/` | MoveIt Servo setup (SRDF, kinematics, joint limits, Servo parameters, launch) used to send Cartesian twists to the arm. No `move_group`. |
| `ur30_ibvs/` | The IBVS package: panel detector, edge tracker, controller, state machine, dispense lap and tests. No simulator dependency. Has its own [README](ur30_ibvs/README.md). |
| `ur30_real_bringup/` | The real cell: UR driver, MoveIt Servo and Orbbec camera launch files, and configuration templates. |
| `docker/`, `.github/workflows/` | The robot image, the Jetson's compose files and host scripts, CI and release. See [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md). |
| `ur30.repos` | Source dependencies of the simulation, fetched with `vcs` (next section). |

## Setup

Requirements: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (the Jazzy default).

### 1. Clone into a workspace

```bash
mkdir -p ~/ur_ws_ah && cd ~/ur_ws_ah
git clone https://github.com/pvrohin/UR30_ROS2_IBVS.git src
cd src
```

### 2. Fetch the simulation's source dependencies

The UR description and Gazebo simulation, at the revisions the cell was tested against
(`ur30.repos`). They are ignored by git. `orbbec_description` comes from apt in the next step.

```bash
sudo apt install python3-vcstool
vcs import . < ur30.repos
```

### 3. Install dependencies

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-broadcaster ros-jazzy-joint-trajectory-controller \
  ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2 \
  ros-jazzy-moveit-servo ros-jazzy-visp ros-jazzy-orbbec-description
cd ~/ur_ws_ah
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # picks up anything missing
```

### 4. Build

```bash
cd ~/ur_ws_ah
colcon build --symlink-install --allow-overriding ur_description \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Build in Release: without it the vision code is compiled unoptimised.
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

To run only the cell without the vision packages, build with `--packages-up-to ur30_cell_bringup`.

## Geometry (world frame)

| Item | Pose |
|---|---|
| Pedestal | height 0.5 m, radius 0.15 m, at the origin |
| Robot base | (0, 0, 0.5) |
| Table top centre | (0.9, 0, 0.75) |
| Door panel centre | (0.8, 0, 0.79), top face at z = 0.805 |

## Known problems

- Depth and point cloud are not bridged by default (they cost a lot of CPU at 30 Hz and the
  IBVS pipeline does not use them). Pass `bridge_depth:=true` to `ur30_cell.launch.py` to enable them.
- Only verified in headless Gazebo. The Gazebo and RViz GUIs were only partly exercised.
- Real hardware has **not** been tested. The real-robot launch files have been run with
  ros2_control's mock hardware only; see [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) for what is
  still open before IBVS may move a real arm.

## Visual servoing

![IBVS demo: the UR30 surveys the table, approaches the door panel and servos on its edge](docs/IBVS.gif)

[ViSP](https://github.com/lagadic/visp) IBVS lives in two packages:

- `ur30_ibvs`: a C++ node that surveys the table, detects the door panel, approaches it, then
  tracks one long edge with ViSP (`vpMeLine`, `vpServo`) at 0.2 m standoff. With
  `dispense_enabled` it goes on to trace the whole panel boundary (the "dispense lap"), turning
  each corner by rotating the camera image, and a `part_shifter` node moves the part by up to
  2 cm / 2 deg at random during the lap to show lateral tracking. It has unit tests and a debug
  image topic. Details, parameters and results are in [`ur30_ibvs/README.md`](ur30_ibvs/README.md).
- `ur30_cell_moveit_config`: the MoveIt Servo configuration used to turn twists into joint
  velocities for `forward_velocity_controller`.

```bash
# hold 0.2 m above the +y edge of the panel
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py

# dispense lap with random part nudges
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true

# clean lap, no nudges
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true shifts:=false

# headless
ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true gazebo_gui:=false launch_rviz:=false
```

Tests: `colcon test --packages-select ur30_ibvs ur30_cell_bringup` (104 tests).

## Real robot

The same IBVS node runs on the real UR30 and Gemini 335Lg from a container on a Jetson AGX
Orin. Setup, deployment, and what must be done before the arm may move under IBVS:
[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).

Status: in headless Gazebo the panel is found at its true pose, the camera settles within about
1 mm of the target when holding, and the dispense lap closes with a settled boundary error of
about 3-7 mm under nudges (see `ur30_ibvs/README.md`).

## License

The repository contains a `LICENSE` file with the GPLv3 text, but the packages
`ur30_cell_description` and `ur30_cell_bringup` declare `BSD-3-Clause` in their
`package.xml`. This mismatch is unresolved; decide on one before distributing.
The vendored repositories keep their own licenses (Universal Robots: BSD-3-Clause,
OrbbecSDK_ROS2: Apache-2.0).
