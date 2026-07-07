cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autonomy_core REQUIRED)

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs
                          autonomy_core)

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

install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
