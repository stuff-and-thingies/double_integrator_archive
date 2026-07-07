cmake_minimum_required(VERSION 3.8)
project(command_gate)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomy_msgs REQUIRED)

add_executable(command_gate_node src/command_gate_node.cpp)
ament_target_dependencies(command_gate_node rclcpp autonomy_msgs)

if(BUILD_TESTING)

endif()

install(TARGETS command_gate_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
