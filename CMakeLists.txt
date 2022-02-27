cmake_minimum_required(VERSION 2.8.3)
project(ecn_mobile_control)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
  tf2_ros
)

find_package(VISP REQUIRED)
set(CMAKE_CXX_STANDARD 17)

generate_dynamic_reconfigure_options(
  cfg/Gains.cfg
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)

add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES} ${VISP_LIBRARIES})
add_dependencies(control ecn_mobile_control_gencpp)
