FROM nvcr.io/nvidia/isaac-sim:5.0.0

ENV DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib

WORKDIR /app