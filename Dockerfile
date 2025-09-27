FROM nvcr.io/nvidia/isaac-sim:5.0.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ISAAC_SIM_ROOT=/isaac-sim
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
ENV PYTHONPATH=${ISAAC_SIM_ROOT}/exts/isaacsim.ros2.bridge/humble/rclpy:${PYTHONPATH}

WORKDIR /app