cmake_minimum_required(VERSION 3.8)

# Name of this ROS 2 package. This should match the <name> field in package.xml.
project(autonomy_core)

# Use C++17 explicitly instead of relying on compiler defaults.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable useful compiler warnings for GCC/Clang. These help catch mistakes while
# learning C++.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Load the ROS 2 CMake build system. Provides ament_package(),
# ament_export_targets(), etc.
find_package(ament_cmake REQUIRED)

# ------------------------------------------------------------------------------
# Pure C++ library
# ------------------------------------------------------------------------------

# Build the reusable pure-C++ library.
#
# Header-only files such as vector2.hpp, vector3.hpp, path.hpp, etc. do not need
# to be listed here because they are not compiled separately.
#
# Only .cpp files that contain compiled implementations go here.
add_library(
  autonomy_core
  src/double_integrator.cpp
  src/grid_map.cpp
  src/grid_neighbors.cpp
  src/dijkstra_planner.cpp
  src/astar_planner.cpp
  src/obstacle_inflation.cpp
  src/line_of_sight.cpp
  src/path_smoother.cpp
  src/path_conversion.cpp
  src/path_to_trajectory.cpp
  src/trajectory_sampler.cpp
  src/trajectory_tracker.cpp)

# Tell CMake where this library's public headers are.
#
# PUBLIC means: - autonomy_core itself can use these include paths. - anything
# that links against autonomy_core also gets these include paths.
#
# BUILD_INTERFACE is used while building directly from ros2_ws/src.
# INSTALL_INTERFACE is used after the package is copied into ros2_ws/install.
target_include_directories(
  autonomy_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

# ------------------------------------------------------------------------------
# Install/export rules
# ------------------------------------------------------------------------------

# Install public headers into the workspace install directory.
#
# Example source header:
# ros2_ws/src/autonomy_core/include/autonomy_sim/double_integrator.hpp
#
# After colcon build, it becomes available from the install space so downstream
# packages can include: #include "autonomy_sim/double_integrator.hpp"
install(DIRECTORY include/ DESTINATION include)

# Install the compiled library.
#
# ARCHIVE: static library output, e.g. libautonomy_core.a LIBRARY: shared
# library output, e.g. libautonomy_core.so RUNTIME: executable/runtime output
# location, mainly relevant on some platforms
#
# EXPORT creates CMake metadata for this target so other packages can import it.
install(
  TARGETS autonomy_core
  EXPORT export_autonomy_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Export the public include directory for downstream packages.
ament_export_include_directories(include)

# Export the library target for downstream packages.
#
# This allows a later package, such as fake_drone_sim, to depend on
# autonomy_core and link against this library.
ament_export_targets(export_autonomy_core HAS_LIBRARY_TARGET)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(BUILD_TESTING)
  function(add_autonomy_test test_name test_source)
    add_executable(${test_name} ${test_source} tests/test_utils.cpp)

    target_include_directories(${test_name}
                               PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)

    target_link_libraries(${test_name} autonomy_core)

    add_test(NAME ${test_name} COMMAND ${test_name})
  endfunction()

  add_autonomy_test(vector2_tests tests/vector2_tests.cpp)
  add_autonomy_test(vector3_tests tests/vector3_tests.cpp)
  add_autonomy_test(time_tests tests/time_tests.cpp)
  add_autonomy_test(path_tests tests/path_tests.cpp)
  add_autonomy_test(trajectory_tests tests/trajectory_tests.cpp)
  add_autonomy_test(double_integrator_tests tests/double_integrator_tests.cpp)
  add_autonomy_test(grid_map_tests tests/grid_map_tests.cpp)
  add_autonomy_test(grid_neighbors_tests tests/grid_neighbors_tests.cpp)
  add_autonomy_test(dijkstra_planner_tests tests/dijkstra_planner_tests.cpp)
  add_autonomy_test(astar_planner_tests tests/astar_planner_tests.cpp)
  add_autonomy_test(obstacle_inflation_tests tests/obstacle_inflation_tests.cpp)
  add_autonomy_test(planner_integration_tests
                    tests/planner_integration_tests.cpp)
  add_autonomy_test(line_of_sight_tests tests/line_of_sight_tests.cpp)
  add_autonomy_test(path_smoother_tests tests/path_smoother_tests.cpp)
  add_autonomy_test(path_conversion_tests tests/path_conversion_tests.cpp)
  add_autonomy_test(path_to_trajectory_tests tests/path_to_trajectory_tests.cpp)
  add_autonomy_test(trajectory_sampler_tests tests/trajectory_sampler_tests.cpp)
  add_autonomy_test(trajectory_tracker_tests tests/trajectory_tracker_tests.cpp)
endif()

# Finalize the ament package. Keep this near the end after targets, install
# rules, exports, and tests.
ament_package()
