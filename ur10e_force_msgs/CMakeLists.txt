cmake_minimum_required(VERSION 3.0.2)
project(ur10e_force_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)


add_message_files(
  FILES
)

add_service_files(
  FILES
  ForceCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur10e_force_msgs
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
