cmake_minimum_required(VERSION 3.5)
project(mobro)

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(log2plot REQUIRED)
find_package(VISP REQUIRED COMPONENTS core)
set(CMAKE_CXX_STANDARD 17)


add_executable(control src/control.cpp src/unicycle.cpp src/bike.cpp src/two_steering.cpp src/robot.cpp)
target_include_directories(control PRIVATE
                            include ${VISP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(control ${VISP_LIBRARIES})
ament_target_dependencies(control rclcpp sensor_msgs tf2_ros nav_msgs urdf)

install(TARGETS control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY urdf launch maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
