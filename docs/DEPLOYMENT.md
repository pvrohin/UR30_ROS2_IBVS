# Deploying to the real cell: Jetson AGX Orin, Gemini 335Lg, UR30

How to set up the robot computer, the camera and the arm, how code gets from the laptop to the
Jetson, and what is still missing before IBVS may move the real arm.

**Status:** the hardware side of the repo has been checked on the laptop with ros2_control's
mock hardware, never on a real Jetson, camera or arm. The safety gaps in
[5.6](#56-safety-gaps-to-close-before-the-arm-moves-under-ibvs) are not all closed yet.

**Assumptions:** an AGX Orin *developer kit*, and a UR30 on PolyScope 5.15 or newer (the UR
driver needs at least 5.9.4 on e-Series controllers; check the version on the pendant).

## Decisions

| Question | Decision | Why |
|---|---|---|
| Jetson OS | **JetPack 7.2.1** (Jetson Linux 39.2.1, Ubuntu 24.04) | The same Ubuntu as the laptop, and the release ROS 2 Jazzy targets. JetPack 7.2 is the first JetPack 7 release for the Orin family. |
| How software is installed | **In containers.** The Jetson host gets only JetPack, Docker, the camera's udev rule, network and tuning. | Reproducible, and rolling back is changing one tag. A broken experiment cannot damage the host. |
| Container base | **`ros:jazzy-ros-base`** (arm64). Not Isaac ROS, not `jetson-containers`. | The IBVS pipeline runs on the CPU (ViSP, OpenCV). Those projects exist for GPU and ML stacks. Add `runtime: nvidia` only if the GPU is ever needed. |
| Where code is built | CI (or the laptop). **Never compiled by hand on the Jetson.** | The Jetson holds a pinned image and this robot's calibration, nothing else. |
| Laptop | **Keep the native Jazzy install for simulation**; Docker for checking the robot image. | Gazebo and Isaac Sim GUIs are easier natively. |
| Camera cable | USB-C on the bench. **Decide USB or GMSL2 before routing a cable along the arm.** | A long, flexing USB3 run is a common cause of dropouts. GMSL2 is what the 335Lg is built for, but needs a deserializer board and driver setup on the Jetson. |

Every package the image needs exists for arm64 in the ROS 2 Jazzy apt repository, including
`visp` 3.7.0, `moveit-servo` 2.12.4, `ur-robot-driver` 3.9.0, `ur-calibration`,
`orbbec-camera` 2.9.3, `image-proc` and `foxglove-bridge` (checked 2026-11-02).
`easy_handeye2` is **not** packaged: use it from source on the laptop.

---

## 1. Connecting the camera, and what to install where

### Docker in 60 seconds

- **Dockerfile:** the recipe. Here: [`docker/Dockerfile`](../docker/Dockerfile).
- **Image:** the built result, immutable: a small Ubuntu with ROS and this repo's code in it.
- **Container:** a running instance of an image. Changes made inside are lost when it is
  removed, by design.
- **Registry (GHCR):** where images are stored. CI pushes to it; the Jetson pulls from it.
- **Bind mount:** a host folder made visible inside a container. Calibration, logs and
  recordings live there because they must outlive any container.
- **Compose:** one YAML file saying which containers run and how. Here:
  [`docker/compose.yaml`](../docker/compose.yaml).

A container shares the host's Linux kernel. So **kernel-level things stay on the host**
(udev rules, network, CPU power settings, drivers) and **user-space things go in the image**
(ROS, ViSP, this code).

| On the Jetson host, set up once | In the image, versioned | Bind-mounted, per robot |
|---|---|---|
| JetPack, Docker, Orbbec udev rule, static IP, chrony, power mode, SSH keys | ROS Jazzy, `ur_robot_driver`, `moveit_servo`, `orbbec_camera`, ViSP, this repo's packages | `/etc/ur30/`: calibration and site parameters. `/var/lib/ur30/`: recordings and logs |

**Do not install on the host:** ROS, OrbbecSDK, ViSP, OpenCV development packages.

### Test the camera on the laptop first

This tells a camera or cable problem apart from a Jetson problem.

```bash
sudo apt install ros-jazzy-orbbec-camera
sudo cp /opt/ros/jazzy/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
# Plug the camera straight into a USB3 port (no hub), then:
lsusb -t                                          # the camera's line must say 5000M, not 480M
ros2 launch orbbec_camera gemini_330_series.launch.py \
  color_width:=1280 color_height:=720 color_fps:=30 \
  enable_depth:=false enable_point_cloud:=false
ros2 topic hz /camera/color/image_raw             # about 30 Hz
ros2 topic echo --once /camera/color/camera_info  # note K (intrinsics) and D (distortion)
```

- 480M means a USB2 link: try another cable or port. Many USB-C cables are USB2 only.
- Do not run ROS nodes with `sudo` to get around a missing udev rule.
- Update the camera firmware to **1.8.10** (the version Orbbec recommends for the 335Lg on
  `v2-main`) with the Orbbec Viewer.
- The default `camera_name` gives `/camera/color/image_raw` and `/camera/color/camera_info`:
  the topics `ur30_ibvs.yaml` expects. No remapping.
- `ur30_real_bringup/launch/camera.launch.py` starts the camera the way the robot will:
  `ros2 launch ur30_real_bringup camera.launch.py`.

### Flash the Jetson

1. Check what is installed (monitor and keyboard): `cat /etc/nv_tegra_release`. Reflash
   unless it reports R39.
2. Flash JetPack 7.2.1 with **NVIDIA SDK Manager** from the laptop: the kit in recovery mode,
   connected through its flashing USB-C port (the developer kit user guide shows which).
   NVIDIA also publishes an ISO for developer kits; check it covers AGX Orin before using it.
3. Install to NVMe if the kit has one, and keep Docker's data there: images and recordings
   fill eMMC quickly.

### Connect the camera to the Jetson

A USB3 host port, directly; repeat the `lsusb -t` check. The udev rule is installed on the
host by [`docker/host/seed_site_config.sh`](../docker/host/seed_site_config.sh), which copies
it out of the image, so the rule always matches the driver version (section 3).

---

## 2. How code gets to the robot

```
laptop: edit, test in Gazebo ──> git push (branch) ──> pull request
                                                         │
                         GitHub Actions (ci.yml): robot image + all tests in clean environments
                                                         │
                                       merge to main, then: git tag v0.2.0 && git push origin v0.2.0
                                                         │
                     GitHub Actions (release.yml): arm64 image ──> ghcr.io/pvrohin/ur30_ibvs:v0.2.0
                                                         │
Jetson: IMAGE_TAG=v0.2.0 in .env ──> docker compose pull && docker compose up -d camera robot
Roll back: IMAGE_TAG=v0.1.0 ──> docker compose up -d camera robot
```

- **CI** checks every change automatically. **CD** here means the image is built and published
  automatically. **Deploying stays a manual step**: a robot arm never starts new code without
  a person there.
- Images are tagged with a version and a commit (`sha-…`), never only `latest`: it must always
  be possible to say exactly what is on the robot.
- The release builds on GitHub's native arm64 runner, the same architecture as the Jetson.
- While iterating on the bench, skip the registry and build on the Jetson from the laptop:
  `docker context create jetson --docker host=ssh://ur30-jetson`, then
  `docker --context jetson compose -f compose.yaml -f compose.dev.yaml up --build`.
  The sources stay on the laptop.

### The files

| File | What it does |
|---|---|
| [`docker/Dockerfile`](../docker/Dockerfile) | Stages `base`, `build-deps`, `build`, `test`, `runtime`. Only the four hardware packages; no Gazebo. Release build. |
| [`.dockerignore`](../.dockerignore) | Keeps everything else (simulation, `.git`) out of the build. |
| [`docker/compose.yaml`](../docker/compose.yaml) | The Jetson: `camera`, `robot` (driver, Servo) and `ibvs` containers from one image. |
| [`docker/compose.dev.yaml`](../docker/compose.dev.yaml) | Laptop: builds the image locally and uses mock hardware instead of the arm. |
| [`docker/.env.example`](../docker/.env.example) | Per-machine settings: image tag, robot IP, domain ID, calibration file. |
| [`docker/host/setup_jetson.sh`](../docker/host/setup_jetson.sh) | One-time host setup (section 3). |
| [`docker/host/seed_site_config.sh`](../docker/host/seed_site_config.sh) | Copies the udev rule and the `/etc/ur30` templates out of the image. |
| [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) | Pull requests: the robot image's tests, and a full build with simulation from a fresh clone. |
| [`.github/workflows/release.yml`](../.github/workflows/release.yml) | Version tags: test, then publish the arm64 image. |

Why three containers: separate logs, and the IBVS node can be restarted without touching the
driver. `camera` and `robot` restart on their own; `ibvs` never does. The node also waits for
an explicit `ros2 service call /ibvs_state_machine/start std_srvs/srv/Trigger`, and refuses
while `site_configured` is false.

### Checking the robot image on the laptop

Your user needs access to Docker first: `sudo usermod -aG docker $USER`, then log out and in.

```bash
cd src
docker build -f docker/Dockerfile --target test .     # builds and runs the unit tests
cd docker && cp .env.example .env                     # then set IMAGE_TAG=dev in .env
docker compose -f compose.yaml -f compose.dev.yaml up --build robot ibvs
```

Without a camera the `ibvs` container stays in `WAIT_READY`, "waiting for: camera_info". With
one, it goes on to `IDLE` and refuses `start`: the shipped template has `site_configured: false`.

To look at the containers with ROS tools on the **same** computer, use UDP in that terminal and
restart the `ros2` command-line daemon once:
```bash
export ROS_DOMAIN_ID=30 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 daemon stop
ros2 control list_controllers    # scaled_joint_trajectory_controller active, forward_velocity_controller inactive
```
The containers run as root and talk to each other through shared memory, which a normal user's
processes cannot open; without the setting nothing is found. Computers elsewhere on the network
(the laptop looking at the Jetson) always use UDP and need nothing.

### Working habits

- A branch and a pull request for every change, even working alone. Do not commit to `main`.
- A new version tag for each deployment; a line in a `CHANGELOG.md` saying what changed.
- Build in Release (`--cmake-args -DCMAKE_BUILD_TYPE=Release`) on the laptop too. Without it
  the vision code is compiled unoptimised and timing measurements mean nothing.

---

## 3. Getting the Jetson ready

**Everything is done from the laptop over SSH.** After flashing, the Jetson needs no monitor.

### Network

The developer kit has one RJ45 port, and the Jetson, the arm and the laptop must all reach
each other. Use a **small unmanaged gigabit switch on a private robot subnet**, and the
Jetson's Wi-Fi for internet (pulling images).

| Device | Address | Link |
|---|---|---|
| UR30 | 192.168.56.101 | Ethernet |
| Jetson | 192.168.56.2 | Ethernet, static, no gateway |
| Laptop | 192.168.56.3 | Ethernet, static; Wi-Fi stays for internet |

Keep the robot network off the office LAN, and never run robot control traffic over Wi-Fi.

### Host setup

```bash
# On the laptop
ssh-keygen -t ed25519                       # only if you have no key yet
ssh-copy-id <user>@<jetson-ip>
# ~/.ssh/config:
#   Host ur30-jetson
#     HostName 192.168.56.2
#     User <user>
scp -r src/docker ur30-jetson:~/ur30-docker
ssh ur30-jetson

# On the Jetson
ip -br link                                 # find the Ethernet interface name
sudo ~/ur30-docker/host/setup_jetson.sh <interface> 192.168.56.2/24
# log out and back in, then:
cd ~/ur30-docker && cp .env.example .env    # edit IMAGE_TAG and ROBOT_IP
docker compose pull
./host/seed_site_config.sh ghcr.io/pvrohin/ur30_ibvs:<tag>
```

`setup_jetson.sh` checks for JetPack 7, then sets the hostname, the static address, Docker
with log rotation, chrony, holds the NVIDIA board packages against automatic upgrades,
switches off the desktop and creates `/etc/ur30` and `/var/lib/ur30`. It prints the power
modes; choose the maximum with `sudo nvpmodel -m <id>`, and `sudo jetson_clocks` for steady
timing.

### Real-time

The UR client library documents a real-time kernel setup; it reduces control jitter. Start
with the stock kernel and measure: watch the driver log for overrun warnings. The `robot`
container already has the permissions for real-time priorities. Check NVIDIA's documentation
for the JetPack 7.2.1 real-time kernel before installing it.

### First container test

```bash
docker compose up -d camera
# exec skips the entrypoint, so go through it to get ROS on the PATH
docker compose exec camera /entrypoint.sh ros2 topic hz /camera/color/image_raw
```

### Watching from the laptop

Do not stream raw images over Wi-Fi. Either run `foxglove-bridge` on the Jetson and connect
Foxglove from the laptop, or put the laptop on the robot network with the same
`ROS_DOMAIN_ID` and run RViz. `~/debug_image` is only drawn while something subscribes, so
leave it off when measuring timing.

---

## 4. What to do before the final URDF exists

### A. Laptop, no hardware

1. **Baseline.** Build in Release, `colcon test` (104 tests), and a headless lap:
   `ros2 launch ur30_cell_bringup ur30_ibvs_sim.launch.py demo:=true shifts:=false gazebo_gui:=false launch_rviz:=false`.
2. **Mock hardware.** The real-robot launch file with a simulated arm, no robot needed:
   `ros2 launch ur30_real_bringup robot.launch.py robot_ip:=127.0.0.1 use_mock_hardware:=true`.
3. **URSim: the most useful step without a robot.** UR's own controller simulator, already
   installed with the driver, running in Docker:
   ```bash
   ros2 run ur_client_library start_ursim.sh -m ur30
   # the pendant opens in a browser: http://192.168.56.101:6080/vnc.html
   ros2 launch ur30_real_bringup robot.launch.py robot_ip:=192.168.56.101
   ```
   Practise the real procedure: install the External Control URCap, create the program,
   press play. Then check the switch from `scaled_joint_trajectory_controller` to
   `forward_velocity_controller` and Servo twists against the real driver.
4. **Failure drills in URSim:** stop the program, trigger a protective stop, kill
   `servo_node`, kill `ibvs_node`. Write down what happens; these answer
   [5.6](#56-safety-gaps-to-close-before-the-arm-moves-under-ibvs) items 3 to 5.

### B. Laptop and camera, on a tripod

5. **Intrinsics.** Compare the real `camera_info` with the simulation, which assumes a 90°
   horizontal field of view at 1280x720; the survey pose was chosen for that. Check how much
   of the panel fills the image from 0.6 m.
6. **Distortion.** The code ignores it (`panel_detector.hpp` uses only `K`). A straight edge
   bends near the border of a real lens. Look at `D`, and try `image_proc` rectification;
   pointing `image_topic` at the rectified topic needs no code change.
7. **Recordings.** A target like the real part, recorded at the survey height (0.6 m) and the
   servo height (0.2 m), under different lighting (overhead, window light, a hand's shadow,
   shiny surfaces):
   ```bash
   ros2 bag record -s mcap /camera/color/image_raw /camera/color/camera_info
   ```
8. **Offline perception tests: the largest unknown.** `PanelDetector` and `EdgeTracker` have no
   ROS dependency and can run directly on recorded frames. They were tuned in a clean
   simulation:
   - a **dark rectangle on a bright table** (`dark_threshold: 150`, measured on rendered
     colours);
   - `min_rectangularity: 0.85` and the 0.9 x 0.55 m size assume a rectangular outline. **A
     real door panel is not one.**
   - Reflections and clamps can break both the threshold and the edge tracker.

   Keep a few real frames as test fixtures in `ur30_ibvs/test/`, so later tuning cannot
   silently break cases that used to work.
9. **End-to-end smoke test:** real camera, URSim and IBVS together. The loop is not closed, so
   nothing converges, but topics, frames, timing and launch files are proven together.

### C. Jetson

10. Flash and set up (section 3).
11. Camera container: 30 Hz for an hour without dropouts.
12. **Time budget.** The node logs `image processing: N frames, mean, p99, max` every 10 s.
    Image processing shares a thread with the 50 Hz twist timer, and Servo drops twists older
    than 0.1 s: if p99 approaches 100 ms, give the timer its own callback group with a
    multi-threaded executor. Watch `tegrastats` for CPU saturation and throttling.
13. Soak test: IBVS on the Jetson, URSim on the laptop, over the switch, for an hour.

### D. Preparing for the URDF and Isaac Sim

14. **Do not export the UR30 from SolidWorks.** Keep the `ur_robot` xacro macro and its
    kinematics; export only the bracket, pedestal, table and applicator as meshes, in metres,
    with sensible origins, and add them to
    [`ur30_cell_common.xacro`](../ur30_cell_description/urdf/ur30_cell_common.xacro).
15. From the same CAD model, read the transforms `tool0` to `camera_link` and `tool0` to the
    applicator tip. The first replaces the placeholder in
    [`camera_mount_nominal.yaml`](../ur30_cell_description/config/camera_mount_nominal.yaml).
16. **Isaac Sim is another adapter, like Gazebo**: the same topic names and `/clock`, the same
    joint command interfaces. Then `ur30_ibvs` does not change. It stays on the laptop's GPU.
17. Hand-eye calibration: a printed ChArUco board on a rigid plate, and `easy_handeye2` from
    source on the laptop (or OpenCV's `calibrateHandEye`).

---

## 5. The repo for the real robot

**One IBVS core, swappable adapters.** `ur30_ibvs` does not know whether it runs in Gazebo,
Isaac Sim or the real cell: it sees topics, TF and parameters.

```
              ur30_ibvs (node and core library): no simulator or hardware dependencies
                                         │
              topics / TF / controllers: the same interface everywhere
             ╱                           │                           ╲
  ur30_cell_bringup              Isaac Sim adapter              ur30_real_bringup
  (Gazebo, sim time)               (future)                    (UR driver, Orbbec, wall time)
```

### 5.1 Dependencies

- The vendored repositories are no longer tracked. `ur_description` and `orbbec_description`
  come from apt through `rosdep`; the simulation's `ur_description_gz` and `ur_simulation_gz`
  are fetched at their tested revisions: `cd src && vcs import . < ur30.repos`.
- `ur30_ibvs` declares no simulator packages, so `rosdep` installs no Gazebo on the Jetson.
- Open: the GPLv3 `LICENSE` file and the BSD-3-Clause `package.xml` files disagree. Decide
  before distributing.

### 5.2 Robot description

| File | Contents |
|---|---|
| `ur30_cell_common.xacro` | World, pedestal, UR30, table, camera link tree. **The SolidWorks parts go here.** |
| `ur30_cell.xacro` | Simulation: common parts, door panel, Gazebo plugin and sensor. |
| `ur30_cell_real.urdf.xacro` | Real robot: common parts and the UR driver's `ros2_control`. Takes `kinematics_params` and `camera_mount_params`. |
| `camera_mount.xacro` | The mount only. The Gazebo sensor is in `camera_gz_sensor.xacro`. |

The real description has no door panel: the camera finds the part.

### 5.3 `ur30_real_bringup`

| File | |
|---|---|
| `launch/robot.launch.py` | The UR driver (`ur_control.launch.py`) and MoveIt Servo. The driver starts `scaled_joint_trajectory_controller` and loads `forward_velocity_controller` inactive by itself. |
| `launch/rsp.launch.py` | `robot_state_publisher` with the real description; the driver includes it. |
| `launch/camera.launch.py` | The 335Lg: color 1280x720 at 30 Hz, no depth, no point cloud, `publish_tf:=false` (the URDF already publishes the camera frames, and two publishers of one frame conflict). |
| `config/ur30_ibvs_real.yaml` | Template for `/etc/ur30/ibvs_site.yaml`. |
| `config/ur30_servo_real.yaml` | Servo with lower speed limits (0.05 m/s, 0.1 rad/s). |

### 5.4 Launch files and simulation time

- `use_sim_time` is a launch argument of the Servo launch and of `ibvs_node.launch.py`. On the
  real robot there is no `/clock`: with simulation time on, ROS time would never advance.
- The Gazebo demo moved to `ur30_cell_bringup/launch/ur30_ibvs_sim.launch.py`, together with
  `part_shifter.py` and `ibvs_test_runner.py`, which only work in Gazebo.

### 5.5 Site parameters

[`ur30_ibvs_real.yaml`](../ur30_real_bringup/config/ur30_ibvs_real.yaml) is the template: the
driver's controller names, the robot program topic, slow speeds, and the values marked
`MEASURE` (survey pose, part size, panel height, threshold). The node refuses to move the arm
until `site_configured: true`.

### 5.6 Safety gaps to close before the arm moves under IBVS

| # | Gap | State |
|---|---|---|
| 1 | The survey move had a fixed 6 s duration from wherever the arm was, with no planning or collision checking: a far start meant a fast sweep. | **Closed.** Duration scales with the largest joint move (`survey_max_joint_speed`), and moves beyond `survey_max_joint_distance` are refused. |
| 2 | Servo collision checking is off: the SRDF only covers the arm. | Open. Needs a cell SRDF with the final URDF's bracket and applicator. |
| 3 | The node did not watch the robot. After a protective stop it carried on. | **Closed.** With `robot_program_topic` set, it fails when the External Control program stops, and refuses to start while it is not running. Tested with mock hardware and a stand-in topic (the mock has no robot program); still to be confirmed against URSim. |
| 4 | `forward_velocity_controller` holds its last command. Measured with mock hardware: Servo publishes nothing while idle, 100 Hz while twists arrive, and **one single zero** 0.1 s after the last twist, then nothing. So stopping depends on that one message: if `servo_node` dies first, or the message is lost, the last nonzero velocity stays. | Open. Drill in URSim (4.4), then add a watchdog that deactivates the controller when commands stop. The driver's controller stopper covers the program-stopped case. |
| 5 | Servo smoothing is off: twist steps go straight to the arm. | Open. Retest the acceleration limiter in URSim and on the arm, or rate-limit in the node. |
| 6 | Lens distortion is ignored, and TF is looked up at the latest time rather than the image's. | Open; negligible at low speed. Check as speeds rise. |

### 5.7 First power-on with the arm

Each step passes before the next one starts. **A person stays at the E-stop for every motion
test**, and the cell's risk assessment comes first, as your site requires.

1. Pendant: payload and TCP including bracket, camera and applicator. **Reduced safety
   limits**: speed, force, and a safety plane above the table. Network: 192.168.56.101.
2. Install the External Control URCap; Host IP is the **Jetson** (192.168.56.2). Create the
   program. The driver uses ports 50001 to 50004 on the Jetson.
3. Kinematics calibration, then set `KINEMATICS_FILE=/etc/ur30/kinematics.yaml` in `.env`:
   ```bash
   docker compose run --rm robot ros2 launch ur_calibration calibration_correction.launch.py \
     robot_ip:=192.168.56.101 target_filename:=/var/lib/ur30/kinematics.yaml
   sudo cp /var/lib/ur30/kinematics.yaml /etc/ur30/   # /etc/ur30 is read-only in containers
   ```
4. Driver only, arm in free space: `/joint_states` matches the pendant. Pendant speed slider
   low.
5. Servo: small twists by hand. **Check that +X really is +X of `tool0`.**
6. Hand-eye calibration into `/etc/ur30/camera_mount.yaml`; check by projecting a known point.
7. IBVS `SEARCH` only: the camera looks at the part, the arm stays still. Watch `~/debug_image`.
8. `APPROACH` slowly with extra standoff, then `SERVO` at 0.2 m, then `DISPENSE` at 0.01 m/s,
   raising speeds gradually.

### Keeping it maintainable

- Do not edit vendored code; changes go in this repo's packages.
- Commit what is true everywhere. This robot's values and calibration live in `/etc/ur30/`,
  backed up separately.
- A test with every bug fix; recorded frames with every perception change.
- Keep this file, `README.md` and `CLAUDE.md` in step with the code.

## Sources

- [NVIDIA JetPack downloads](https://developer.nvidia.com/embedded/jetpack/downloads) and the [Jetson Linux archive](https://docs.nvidia.com/jetson/archives/)
- [OrbbecSDK_ROS2, v2-main](https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main)
- [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and [Universal_Robots_Client_Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library)
- [Orbbec Viewer in UR's AI Accelerator documentation](https://docs.universal-robots.com/AI_Accelerator_Documentation/html/orbbec_viewer.html): a 335Lg and AGX Orin pairing documented by UR
