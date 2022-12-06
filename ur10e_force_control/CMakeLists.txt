cmake_minimum_required(VERSION 3.0.2)
project(ur10e_force_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(MRobotics_DIR "$ENV{HOME}/ModernRoboticsCpp") # To use the functions of the modern robotics


find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED thread system)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  ur10e_force_msgs
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)


catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    geometry_msgs
    roscpp
    ur10e_force_msgs
    sensor_msgs
    moveit_msgs
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
  DEPENDS 
    system_lib
)



include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${MRobotics_DIR}/include
)

add_library(${PROJECT_NAME}
  src/parameter.cpp
)
################################################################
add_executable(gravity_comp
  src/gravity_comp.cpp
)

add_dependencies(gravity_comp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gravity_comp
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${MRobotics_DIR}/_install/lib/libModernRoboticsCpp.so
)

# ################################################################
add_executable(compute_force_command
  src/compute_force_command.cpp
)
target_link_libraries(compute_force_command ${catkin_LIBRARIES})
add_dependencies(compute_force_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# ################################################################
add_executable(force_control
  src/force_control.cpp
)
add_dependencies(force_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(force_control
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${MRobotics_DIR}/_install/lib/libModernRoboticsCpp.so
)

