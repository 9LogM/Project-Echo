cmake_minimum_required(VERSION 3.8)
project(pointcloud_transformer)

# Find all necessary packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)

# Add executable.
add_executable(
    transformer_node src/transformer_node.cpp
)

# Use ament_target_dependencies to handle all ROS2 dependencies.
ament_target_dependencies(
    transformer_node
    "rclcpp"
    "sensor_msgs"
    "std_msgs"
)

# Link any non-ament/system libraries.
target_link_libraries(transformer_node
    ${YAML_CPP_LIBRARIES}
    spdlog::spdlog # Added: Explicitly link spdlog
)

# Install the executable.
install(
    TARGETS transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

# Register the package with ament.
ament_package()