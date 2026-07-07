cmake_minimum_required(VERSION 3.8)
project(mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autonomy_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node rclcpp autonomy_msgs nav_msgs)

if(BUILD_TESTING)

endif()

install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
