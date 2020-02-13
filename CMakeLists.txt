cmake_minimum_required(VERSION 2.8.3)
project(ecn_mobile_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Gains.cfg
)



catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES mobro
#  CATKIN_DEPENDS nav_msgs sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable(gains_client src/gains_client.cpp)
target_link_libraries(gains_client ${catkin_LIBRARIES} )
