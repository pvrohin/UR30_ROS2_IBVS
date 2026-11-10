#!/bin/bash
# Make ROS and this workspace available, then run the container's command.
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
exec "$@"
