#!/bin/bash

set -e

echo "Sourcing ROS 2 workspace..."
source /opt/ros/humble/setup.bash

echo "Building ROS 2 workspace in $(pwd)"
cd /app/src/ros2_ws
rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash

echo "Starting transformer_node in the background..."
ros2 run pointcloud_transformer transformer_node &

echo "Starting digital_twin_controller.py..."
/isaac-sim/python.sh src/digital_twin_controller.py