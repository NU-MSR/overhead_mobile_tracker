cmake_minimum_required(VERSION 2.8.3)
project(overhead_mobile_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  tf
  vive_tracker_loc
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  SetBool.srv
  SetOdomOffset.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES nav_msgs
  )

catkin_package(
   CATKIN_DEPENDS geometry_msgs nav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
