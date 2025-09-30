#!/bin/bash

set -e

ROS_BASE_WS_PATH="/app/IsaacSim-ros_workspaces/build_ws/humble/humble_ws"

if [ ! -f "${ROS_BASE_WS_PATH}/install/setup.bash" ]; then
    echo "--- Python 3.11 ROS 2 base not found. Building now... ---"
    echo "--- This will take a long time on the first run. ---"
    cd /app/IsaacSim-ros_workspaces
    ./build_ros.sh -d humble -v 22.04
else
    echo "--- Found existing Python 3.11 ROS 2 base. Skipping build. ---"
fi

echo "--- Sourcing Python 3.11 ROS 2 base environment ---"
source ${ROS_BASE_WS_PATH}/install/setup.bash

echo "Building ROS 2 workspace in $(pwd)"
cd /app/src/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash

cleanup() {
    echo "--- Shutting down background processes ---"
    kill $(jobs -p)
    echo "--- Shutdown complete ---"
}
trap cleanup SIGINT SIGTERM

echo "Starting transformer_node in the background..."
ros2 run pointcloud_transformer transformer_node &

echo "Starting digital_twin_controller.py..."
/isaac-sim/python.sh src/digital_twin_controller.py

echo "--- Startup complete. ---"