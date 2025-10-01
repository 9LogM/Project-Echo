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
ros_node_pid=$!

# --- Launch Isaac Sim using the Internal Environment (Workflow B) ---
echo "--- Launching Isaac Sim digital twin ---"
/isaac-sim/python.sh /app/src/digital_twin_controller.py &
isaac_pid=$!

# --- Process Management ---
echo "Isaac Sim (PID: $isaac_pid) and ROS Node (PID: $ros_node_pid) are running."
echo "Press Ctrl+C to stop."

cleanup() {
    echo "--- Shutting down processes ---"
    kill -SIGTERM $ros_node_pid
    kill -SIGTERM $isaac_pid
    wait $ros_node_pid
    wait $isaac_pid
    echo "--- Shutdown complete ---"
}

trap cleanup SIGINT SIGTERM
wait -n
cleanup