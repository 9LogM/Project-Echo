cmake_minimum_required(VERSION 3.8)
project(pointcloud_transformer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(
    transformer_node src/transformer_node.cpp
)

target_link_libraries(transformer_node yaml-cpp)

ament_target_dependencies(
    transformer_node
    "rclcpp"
    "sensor_msgs"
)

install(
    TARGETS transformer_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()