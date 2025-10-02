#!/bin/bash

set -e

# --- Launch the C++ node using the system ROS 2 environment ---
echo "--- Launching ROS 2 pointcloud_transformer node ---"
(
  unset LD_LIBRARY_PATH PYTHONPATH
  source /opt/ros/humble/setup.bash
  source /app/install/setup.bash
  ros2 run pointcloud_transformer transformer_node
) &


# --- Launch Isaac Sim using its own internal ROS 2 environment ---
echo "--- Launching Isaac Sim digital twin ---"
(
  unset LD_LIBRARY_PATH PYTHONPATH
  ROS_DISTRO=humble \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib \
  PYTHONPATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/rclpy \
  /isaac-sim/python.sh /app/src/digital_twin_controller.py
)

wait