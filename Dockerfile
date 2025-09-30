FROM nvcr.io/nvidia/isaac-sim:5.0.0

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    gnupg \
    lsb-release \
    ca-certificates \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

 RUN install -m 0755 -d /etc/apt/keyrings \
 && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
 && chmod a+r /etc/apt/keyrings/docker.gpg \
 && echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
    tee /etc/apt/sources.list.d/docker.list > /dev/null

RUN apt-get update && apt-get install -y \
    git \
    docker-ce-cli \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    libyaml-cpp-dev \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

WORKDIR /app

RUN git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git

COPY run_app.sh .
RUN chmod +x run_app.sh