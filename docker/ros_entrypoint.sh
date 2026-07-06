#!/bin/bash
set -e

######### ROS #########
if [ -d /opt/ros/$ROS_DISTRO ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
    source ${OVERLAY_WS}/install/local_setup.bash
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

exec "$@"
