cmake_minimum_required(VERSION 2.8.3)
project(overhead_mobile_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ar_sys
  ar_track_alvar
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  tf
  usb_cam
  uvc_camera
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
