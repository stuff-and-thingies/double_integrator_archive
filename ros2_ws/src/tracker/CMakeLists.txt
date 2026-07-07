cmake_minimum_required(VERSION 3.8)
project(tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonomy_core REQUIRED)
find_package(autonomy_msgs REQUIRED)

add_executable(tracker_node src/tracker_node.cpp)
ament_target_dependencies(tracker_node rclcpp nav_msgs geometry_msgs
                          autonomy_core autonomy_msgs)

if(BUILD_TESTING)
  # Block the linters before finding ament_lint_auto
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Load the auto-lint framework
  find_package(ament_lint_auto REQUIRED)

  # Discover remaining test hooks safely
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
