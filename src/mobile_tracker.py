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
  + camera_frame_id (string) ~ What frame is at the camera lens? (default: "/overhead_cam_frame")
  + x0,y0,th0 (float) ~ These are used to define the odometry offset (default: all zero)
  + height (float) ~ Distance from camera to tag in z-direction (default: 2.5)
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
import odom_conversions

# GLOBAL CONSTANTS
PATH_LEN = 30 # number of elements in published path


class MobileTracker( object ):
    def __init__(self):
        # first let's load all parameters:
        self.frame_id = rospy.get_param("~frame_id", "odom_meas")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "overhead_cam_frame")
        self.x0 = rospy.get_param("~x0", 0.0)
        self.y0 = rospy.get_param("~y0", 0.0)
        self.th0 = rospy.get_param("~th0", 0.0)
        self.height = rospy.get_param("~height", 2.5)
        self.pubstate = rospy.get_param("~pubstate", True)

        # now let's create publishers, listeners, and subscribers
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.meas_pub = rospy.Publisher("meas_pose", Odometry, queue_size=5)
        self.path_pub = rospy.Publisher("meas_path", Path, queue_size=1)
        
        



    
def main():
    rospy.init_node('mobile_robot_tracker', log_level=rospy.INFO)

    try:
        tracker = MobileTracker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()
