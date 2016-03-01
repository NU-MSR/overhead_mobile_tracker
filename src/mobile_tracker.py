#!/usr/bin/env python
"""
Jarvis Schultz
February 2016

This is a simple node that uses ar_track_alvar to track the pose of a mobile
robot using an overhead camera system.

SUBSCRIPTIONS:
  + ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers) ~ pose of all markers detected by ar_track_alvar

PUBLISHERS:
  + meas_pose (nav_msgs/Odometry) ~ measured pose of the mobile robot
  + meas_path (nav_msgs/Path) ~ Path that the robot has traced out over time

SERVICES:
  + publish_bool (std_srvs/SetBool) ~ Control whether the pose of the robot and the path should be published or not
  + set_offset (overhead_mobile_tracker/OdomOffset) ~ Set an offset that is always added to tracker value before publishing
  + set_height (overhead_mobile_tracker/SetHeightOffset) ~ Set distance driving plane is from camera (in driving plane normal direction)

PARAMETERS:
  + frame_id (string) ~ What frame should measured odometry be reported in (default: "/odom_meas")
  + x0,y0,th0 (float) ~ These are used to define the odometry offset (default: all zero)
  + height (float) ~ Distance from camera to tag in z-direction
  + pubstate (bool) ~ Should publishing be enabled by default
"""
# ROS IMPORTS
import rospy
import tf
import tf.transformations as tr
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from overhead_mobile_tracker.srv import OdomOffset
from overhead_mobile_tracker.srv import OdomOffsetRequest
from overhead_mobile_tracker.srv import OdomOffsetReply

# NON-ROS IMPORTS
import numpy as np
from collections import deque
import copy

# LOCAL IMPORTS
import angle_utils
import conversions

# GLOBAL CONSTANTS
PATH_LEN = 30 # number of elements in published path


class MobileTracker( object ):
    def __init__(self):
        # first let's load all parameters:
        self.rospy.get



    
def main():
    rospy.init_node('mobile_robot_tracker', log_level=rospy.INFO)

    try:
        tracker = MobileTracker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()
