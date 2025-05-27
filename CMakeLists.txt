cmake_minimum_required(VERSION 3.8)
project(spbot_planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)

# allow #include "spbot_planners/..." to resolve in-build
include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(rrt_star_planner SHARED
  src/rrt_star_planner.cpp
)
ament_target_dependencies(rrt_star_planner
  pluginlib nav2_core nav2_util nav2_costmap_2d
  rclcpp geometry_msgs nav_msgs tf2_ros
)

# install headers so downstream sees them (✱)
install(
  DIRECTORY include/
  DESTINATION include
)

# install your library (✱)
install(
  TARGETS rrt_star_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# export include dir for any downstream consumers
ament_export_include_directories(include)

# hook up your plugins.xml into nav2_core’s plugin pool (you already had this)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# but also actually install that XML into share/nav2_core (✱)
install(
  FILES plugins.xml
  DESTINATION share/nav2_core
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
