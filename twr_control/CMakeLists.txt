cmake_minimum_required(VERSION 3.8)
project(twr_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  pluginlib
  nav2_core
)

include_directories(
  nav2_controllers/twr_g2g_controller/include
)

add_library(twr_g2g_controller SHARED
            nav2_controllers/twr_g2g_controller/src/twr_g2g_controller.cpp)
ament_target_dependencies(twr_g2g_controller ${dependencies})

pluginlib_export_plugin_description_file(
  nav2_core 
  nav2_controllers/twr_g2g_controller/config/twr_g2g_controller_descr.xml
)

install(TARGETS twr_g2g_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  nav2_bt
  nav2_controllers
  ros2_controllers
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
