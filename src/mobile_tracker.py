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
  + publish_bool (overhead_mobile_tracker/SetBool) ~ Control whether the pose of the robot and the path should be published or not
  + set_offset (overhead_mobile_tracker/OdomOffset) ~ Set an offset that is always added to tracker value before publishing
  + set_height (overhead_mobile_tracker/SetHeightOffset) ~ Set distance driving plane is from camera (in driving plane normal direction)

PARAMETERS:
  + frame_id (string) ~ What frame should measured odometry be reported in (default: "/odom_meas")
  + camera_frame_id (string) ~ What frame is at the camera lens? (default: "/overhead_cam_frame")
  + base_frame_id (string) ~ Frame attached to the mobile robot (default: "/base_meas")
  + x0,y0,th0 (float) ~ These are used to define the odometry offset (default: all zero)
  + height (float) ~ Distance from camera to tag in z-direction (default: 2.5)
  + pubstate (bool) ~ Should publishing be enabled by default
  + marker_id (int) ~ Which marker should we be tracking (default: 12)
"""
# ROS IMPORTS
import rospy
import tf
import tf.transformations as tr
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from ar_track_alvar_msgs.msg import AlvarMarkers
from overhead_mobile_tracker.srv import SetBool
from overhead_mobile_tracker.srv import SetBoolRequest
from overhead_mobile_tracker.srv import SetBoolReply
from overhead_mobile_tracker.srv import SetOdomOffset
from overhead_mobile_tracker.srv import SetOdomOffsetRequest
from overhead_mobile_tracker.srv import SetOdomOffsetReply
from overhead_mobile_tracker.srv import SetHeightOffset
from overhead_mobile_tracker.srv import SetHeightOffsetRequest
from overhead_mobile_tracker.srv import SetHeightOffsetReply

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
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_meas")
        self.x0 = rospy.get_param("~x0", 0.0)
        self.y0 = rospy.get_param("~y0", 0.0)
        self.th0 = rospy.get_param("~th0", 0.0)
        self.height = rospy.get_param("~height", 2.5)
        self.pubstate = rospy.get_param("~pubstate", True)
        self.marker_id = rospy.get_param("~marker_id", 12)

        # setup other required vars:
        self.odom_offset = odom_conversions.numpy_to_odom(np.array([self.x0, self.y0, self.th0]),
                                                          self.frame_id)

        # now let's create publishers, listeners, and subscribers
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.meas_pub = rospy.Publisher("meas_pose", Odometry, queue_size=5)
        self.path_pub = rospy.Publisher("meas_path", Path, queue_size=1)
        self.alvar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.alvarcb)
        self.publish_serv = rospy.ServiceProxy("publish_bool", SetBool, self.pub_bool_srv_cb)
        self.height_serv = rospy.ServiceProxy("set_height", SetHeightOffset, self.height_srv_cb)
        self.offset_serv = rospy.ServiceProxy("set_offset", SetOdomOffset, self.offset_srv_cb)
        return

    def alvarcb(self, markers):
        rospy.logdebug("Detected markers!")
        # can we find the correct marker?
        for m in markers.markers:
            if m.id == self.marker_id:
                odom_meas = Odometry()
                odom_meas.header = m.pose.header
                odom.child_frame_id = self.base_frame_id
                odom.header.frame_id = self.frame_id
                # now we need to transform this pose measurement from the camera
                # frame into the frame that we are reporting measure odometry in
                pose_transformed = self.transform_pose(m.pose.pose)
                odom.pose.pose = pose_transformed
                # Now let's add our offsets:
                odom = odom_conversions(odom, self.offset)
                self.meas_pub.publish(odom)
        return


    def transform_pose(pose):
        """
        This function will send the transform from the camera frame
        (self.camera_frame_id) to the measurement odometry frame
        (self.frame_id). It will then use this transform to convert the pose arg
        from the camera frame to the odom frame. We will assume that the odom
        measurement frame is a distance of self.height from the camera frame,
        and that the odom frame will b
        """

        return


    def pub_bool_srv_cb(self, request):
        self.pubstate = request.data
        reply = SetBoolReply()
        reply.success = True
        reply.message = "Publishing camera measurements? ... %s"%self.pubstate
        return reply


    def height_srv_cb(self, request):
        self.height = request.dist
        return SetHeightOffsetReply(True)


    def offset_srv_cb(self, request):
        self.odom_offset = request.odom
        return SetOdomOffsetReply(True)

    
def main():
    rospy.init_node('mobile_robot_tracker', log_level=rospy.INFO)

    try:
        tracker = MobileTracker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()
