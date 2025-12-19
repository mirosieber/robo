#!/bin/bash
# Clean snap environment variables that interfere with RViz
unset SNAP
unset SNAP_NAME
unset SNAP_REVISION
unset SNAP_INSTANCE_NAME
unset SNAP_INSTANCE_KEY
unset SNAP_LIBRARY_PATH

# Remove snap paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /home/user/robo/install/setup.bash

# Run RViz
rviz2 -d /home/user/robo/install/robo_control/share/robo_control/rviz/robo.rviz
