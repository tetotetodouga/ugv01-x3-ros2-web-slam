#!/usr/bin/env bash
set -eo pipefail
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${WORKSPACE_SETUP:-$HOME/ros2_ws/install/setup.bash}"
exec ros2 launch ugv01_room_explore web_navigation.launch.py "$@"
