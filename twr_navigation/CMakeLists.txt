cmake_minimum_required(VERSION 3.8)
project(twr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
  include
)

add_library(twr_g2g_controller SHARED
            src/nav2_controller/twr_g2g_controller/twr_g2g_controller.cpp)
ament_target_dependencies(twr_g2g_controller ${dependencies})

pluginlib_export_plugin_description_file(nav2_core config/nav2_controller/twr_g2g_controller_descr.xml)


install(TARGETS twr_g2g_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
