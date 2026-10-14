cmake_minimum_required(VERSION 3.8)
project(falco_px4_telemetry)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(falco_telemetry_node src/falco_telemetry_node.cpp)
target_compile_features(falco_telemetry_node PUBLIC cxx_std_17)
ament_target_dependencies(falco_telemetry_node rclcpp px4_msgs)

install(TARGETS falco_telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
