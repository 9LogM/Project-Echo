FROM nvcr.io/nvidia/isaac-sim:5.0.0

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    gnupg \
    lsb-release \
    ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-dev-tools \
    libyaml-cpp-dev \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY ./src ./src

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

COPY run_app.sh .
RUN chmod +x ./run_app.sh