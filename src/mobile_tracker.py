#!/usr/bin/env python
"""
Jarvis Schultz
February 2016

This is a simple node that uses ar_track_alvar to track the pose of a mobile
robot using an overhead camera system.

SUBSCRIPTIONS:
  + tracker_pose (geometry_msgs/PoseStamped) ~ Pose of the tracker in the lighthouse frame

PUBLISHERS:
  + meas_pose (nav_msgs/Odometry) ~ measured pose of the mobile robot
  + meas_path (nav_msgs/Path) ~ Path that the robot has traced out over time

SERVICES:
  + publish_bool (overhead_mobile_tracker/SetBool) ~ Control whether the pose of the robot and the path should be published or not
  + set_offset (overhead_mobile_tracker/OdomOffset) ~ Set an offset that is always added to tracker value before publishing

PARAMETERS:
  + frame_id (string) ~ What frame should measured odometry be reported in (default: "/odom_meas")
  + camera_frame_id (string) ~ What frame is at the camera lens? (default: "/overhead_cam_frame")
  + base_frame_id (string) ~ Frame attached to the mobile robot (default: "/base_meas")
  + x0,y0,th0 (float) ~ These are used to define the odometry offset (default: all zero)
  + pubstate (bool) ~ Should publishing be enabled by default
"""
# ROS IMPORTS
import rospy
import tf
import tf.transformations as tr
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from overhead_mobile_tracker.srv import SetBool
from overhead_mobile_tracker.srv import SetBoolRequest
from overhead_mobile_tracker.srv import SetBoolResponse
from overhead_mobile_tracker.srv import SetOdomOffset
from overhead_mobile_tracker.srv import SetOdomOffsetRequest
from overhead_mobile_tracker.srv import SetOdomOffsetResponse


# NON-ROS IMPORTS
import numpy as np
from collections import deque
import copy

# LOCAL IMPORTS
import angle_utils
import odom_conversions

# GLOBAL CONSTANTS
PATH_LEN = 200 # number of elements in published path


class MobileTracker( object ):
    def __init__(self):
        # first let's load all parameters:
        self.frame_id = rospy.get_param("~frame_id", "odom_meas")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "overhead_cam_frame")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_meas")
        self.x0 = rospy.get_param("~x0", 0.0)
        self.y0 = rospy.get_param("~y0", 0.0)
        self.th0 = rospy.get_param("~th0", 0.0)
        self.pubstate = rospy.get_param("~pubstate", True)

        # setup other required vars:
        self.odom_offset = odom_conversions.numpy_to_odom(np.array([self.x0, self.y0, self.th0]),
                                                          self.frame_id)
        self.path_list = deque([], maxlen=PATH_LEN)

        # now let's create publishers, listeners, and subscribers
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.meas_pub = rospy.Publisher("meas_pose", Odometry, queue_size=5)
        self.path_pub = rospy.Publisher("meas_path", Path, queue_size=1)
        self.pose_sub = rospy.Subscriber("tracker_pose", PoseStamped, self.posecb)
        self.publish_serv = rospy.Service("publish_bool", SetBool, self.pub_bool_srv_cb)
        self.offset_serv = rospy.Service("set_offset", SetOdomOffset, self.offset_srv_cb)
        return


    def posecb(self, m):
        rospy.logdebug("Received pose!")
        odom_meas = Odometry()
        odom_meas.header.frame_id = self.frame_id
        odom_meas.child_frame_id = self.base_frame_id
        odom_meas.header.stamp = m.header.stamp
        # now we need to transform this pose measurement from the camera
        # frame into the frame that we are reporting measure odometry in
        pose_transformed = self.transform_pose(m)
        if pose_transformed is not None:
            odom_meas.pose.pose = pose_transformed.pose
            # Now let's add our offsets:
            odom_meas = odom_conversions.odom_add_offset(odom_meas, self.odom_offset)
            self.meas_pub.publish(odom_meas)
            self.send_transforms(odom_meas)
            self.publish_path(m)
        return


    def publish_path(self, ps):
        path = Path()
        path.header.frame_id = self.camera_frame_id
        path.header.stamp = ps.header.stamp
        self.path_list.append(ps)
        path.poses = self.path_list
        self.path_pub.publish(path)
        return


    def transform_pose(self, pose):
        try:
            new_pose = self.listener.transformPose(self.frame_id, pose)
            # now new_pose should be the pose of the tag in the odom_meas frame.
            # Let's ignore the height, and rotations not about the z-axis
            new_pose.pose.position.z = 0
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        return new_pose


    def send_transforms(self, odom):
        self.br.sendTransform(
            (odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z),
            (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w),
            rospy.Time.now(),
            self.base_frame_id,
            self.frame_id)
        return

    def pub_bool_srv_cb(self, request):
        self.pubstate = request.data
        reply = SetBoolResponse()
        reply.success = True
        reply.message = "Publishing camera measurements? ... %s"%self.pubstate
        return reply

    def offset_srv_cb(self, request):
        self.odom_offset = request.odom
        return SetOdomOffsetResponse(True)


def main():
    rospy.init_node('mobile_robot_tracker', log_level=rospy.INFO)
    rospy.loginfo("Starting tracking node...")

    try:
        tracker = MobileTracker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()
