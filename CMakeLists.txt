cmake_minimum_required(VERSION 3.5)
project(mobro)

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)

find_package(ct_core REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_models REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(log2plot REQUIRED)
find_package(VISP REQUIRED COMPONENTS core)
set(CMAKE_CXX_STANDARD 17)


add_executable(control src/control.cpp src/unicycle.cpp src/bike.cpp src/two_steering.cpp src/robot.cpp)
target_include_directories(control PRIVATE include ${VISP_INCLUDE_DIRS}
 ${ct_core_INCLUDE_DIRS}
  ${ct_optcon_INCLUDE_DIRS}
  ${ct_models_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${crocoddyl_INCLUDE_DIRS})  
target_link_libraries(control ${VISP_LIBRARIES} ${ct_core_LIBRARIES} ${ct_optcon_LIBRARIES} ${ct_models_LIBRARIES} ${crocoddyl_LIBRARIES})
ament_target_dependencies(control rclcpp sensor_msgs tf2_ros nav_msgs urdf)

install(TARGETS control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
file(GLOB ecn_manip_python src/python/*.py) 
install(PROGRAMS ${ecn_manip_python} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf launch maps
  DESTINATION share/${PROJECT_NAME})

ament_package()

#add_executable(unicycle-optctrl src/unicycle-optctrl.cpp)
#target_include_directories(unicycle-optctrl PRIVATE ${VISP_INCLUDE_DIRS} ${LOG2PLOT_INCLUDE_DIRS})
#target_link_libraries(unicycle-optctrl ${crocoddyl_LIBRARIES} ${LOG2PLOT_LIBRARIES}  ${VISP_LIBRARIES})]]
