cmake_minimum_required(VERSION 3.8)
project(twr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  launch 
  map
  nav2_bt
  nav2_planners
  robot_localization
  slam_toolbox
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()