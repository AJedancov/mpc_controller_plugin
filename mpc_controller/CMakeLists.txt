cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  rclcpp
  nav2_common
  nav2_core
  nav2_util
  nav2_costmap_2d
  geometry_msgs
  nav_msgs
  nav2_msgs
  pluginlib
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(mpc_controller SHARED 
  ${PROJECT_SOURCE_DIR}/src/mpc_controller.cpp
)

ament_target_dependencies(mpc_controller PUBLIC ${dependencies})

pluginlib_export_plugin_description_file(
  nav2_core
  config/mpc_controller_descr.xml
)

install(TARGETS mpc_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
