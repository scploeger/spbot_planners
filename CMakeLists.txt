cmake_minimum_required(VERSION 3.8)
project(spbot_planners)

# ── dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ── build the planner library ────────────────────────────────────────────
add_library(rrt_star_planner SHARED
  src/rrt_star_planner.cpp
)
ament_target_dependencies(rrt_star_planner
  pluginlib
  nav2_core
  nav2_util
  nav2_costmap_2d
  rclcpp_lifecycle
  rclcpp
  tf2_ros
  geometry_msgs
  nav_msgs
)

target_include_directories(rrt_star_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ── install: library + headers ───────────────────────────────────────────
install(TARGETS rrt_star_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install( DIRECTORY bt_xml/
  DESTINATION share/${PROJECT_NAME}/bt_xml ) 

# ── export plugin description *once* (this also installs the XML
#    into share/spbot_planners/ and adds the ament index entry) ───────────
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# ── export headers for downstream users ──────────────────────────────────
ament_export_include_directories(include)
ament_package()
