cmake_minimum_required(VERSION 2.8.3)
project(software_challenge)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  roscpp
  rospy
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES waypoint.action
)

add_message_files(
  DIRECTORY msg
  FILES distance.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES software_challenge
  CATKIN_DEPENDS actionlib actionlib_msgs message_generation roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


add_executable(waypoint_server src/waypoint_server.cpp)
add_dependencies(
  waypoint_server
  ${software_challenge_EXPORTED_TARGETS}
)
target_link_libraries(
  waypoint_server
  ${catkin_LIBRARIES}
)

add_executable(assignment_node src/assignment_node.cpp)
target_link_libraries(assignment_node ${catkin_LIBRARIES})

add_executable(waypoint_client src/waypoint_client.cpp)
add_dependencies(
  waypoint_client
  ${software_challenge_EXPORTED_TARGETS}
)
target_link_libraries(
  waypoint_client
  ${catkin_LIBRARIES}
)


