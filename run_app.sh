#!/bin/bash

set -e

# --- Launch the C++ node using the External Environment (Workflow A) ---
echo "--- Launching ROS 2 pointcloud_transformer node ---"
(
  unset LD_LIBRARY_PATH
  source /opt/ros/humble/setup.bash
  source /app/src/install/setup.bash
  ros2 run pointcloud_transformer transformer_node
) &

# --- Launch Isaac Sim using the Internal Environment (Workflow B) ---
echo "--- Launching Isaac Sim digital twin ---"
/isaac-sim/python.sh /app/src/digital_twin_controller.py