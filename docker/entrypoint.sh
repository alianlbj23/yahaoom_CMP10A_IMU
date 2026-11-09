#!/bin/bash
# Source ROS 2 Jazzy and the wit_ros2_imu workspace, then run the given command.
set -e

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [ -f "${WS:-/ws}/install/setup.bash" ]; then
    # shellcheck disable=SC1091
    source "${WS:-/ws}/install/setup.bash"
fi

exec "$@"
