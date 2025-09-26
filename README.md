# Project Echo

Welcome to **Project Echo**, an open-source, real-time digital twin in NVIDIA Omniverse. This project aims to create a simulated environment that can be controlled by a webcam and features a PyTorch-powered autonomous agent.


## Prerequisites

Before you begin, ensure you have the following installed on your system:

1.  **NVIDIA Driver:** A compatible driver for your GPU.
2.  **[Docker Engine](https://docs.docker.com/engine/install/)**
3.  **[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)**
4.  **[Isaac Sim WebRTC Streaming Client](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html#isaac-sim-latest-release)**


## Getting Started

This project is fully containerized, making setup quick and reproducible.

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/9LogM/project-echo.git
    cd project-echo
    ```

2.  **Build and run the container:**

    ```bash
    docker compose up --build
    ```

3. **Open an interactive shell inside the container:**
    ```bash
    docker exec -it echo-container bash
    ```

4. **Inside the container, run the script:**
    ```bash
    /isaac-sim/python.sh source/hello_world.py
    ```

5.  **View the Simulation:**
    * Launch the **Isaac Sim WebRTC Streaming Client** that you downloaded.
    * Connect to `127.0.0.1`. You should now see the live simulation.


## Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are greatly appreciated.