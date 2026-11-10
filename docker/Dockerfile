# syntax=docker/dockerfile:1
#
# Image for the real UR30 cell (UR driver, MoveIt Servo, Orbbec camera, IBVS node).
# Simulation packages are deliberately left out: the robot computer never needs Gazebo.
#
# Build context: the repository root (src/). From there:
#   docker build -f docker/Dockerfile --target test .       # build and run the unit tests
#   docker build -f docker/Dockerfile --target runtime -t ur30_ibvs:dev .
#
# Stages:
#   base       ROS 2 Jazzy plus an up-to-date rosdep database
#   build-deps everything needed to build and test, installed from the package.xml files only,
#              so this layer is rebuilt only when dependencies change, not when code does
#   build      colcon build in Release
#   test       colcon test (used by CI; nothing ships from it)
#   runtime    run dependencies plus the install/ tree: no compilers, no sources

# The packages that make up the image. Keep this list and the COPY lines below in step.
# Declared before the first FROM so that every stage can use it (each says ARG PACKAGES).
ARG PACKAGES="ur30_cell_description ur30_cell_moveit_config ur30_ibvs ur30_real_bringup"

# Pin by digest (ros:jazzy-ros-base@sha256:...) once a build has been verified on the robot.
FROM ros:jazzy-ros-base AS base
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws
RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

FROM base AS build-deps
ARG PACKAGES
COPY ur30_cell_description/package.xml src/ur30_cell_description/package.xml
COPY ur30_cell_moveit_config/package.xml src/ur30_cell_moveit_config/package.xml
COPY ur30_ibvs/package.xml src/ur30_ibvs/package.xml
COPY ur30_real_bringup/package.xml src/ur30_real_bringup/package.xml
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

FROM build-deps AS build
ARG PACKAGES
COPY ur30_cell_description src/ur30_cell_description
COPY ur30_cell_moveit_config src/ur30_cell_moveit_config
COPY ur30_ibvs src/ur30_ibvs
COPY ur30_real_bringup src/ur30_real_bringup
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select ${PACKAGES} \
       --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM build AS test
ARG PACKAGES
RUN source /opt/ros/jazzy/setup.bash \
    && colcon test --packages-select ${PACKAGES} \
    && colcon test-result --verbose

FROM base AS runtime
COPY --from=build-deps /ws/src /ws/src
RUN apt-get update \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy --dependency-types exec \
    && rm -rf /var/lib/apt/lists/* /ws/src
COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
