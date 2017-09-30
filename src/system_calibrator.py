#!/usr/bin/env python
"""
Jarvis Schultz
February 2016

SUBSCRIPTIONS:
  + ar_pose_marker (ar_track_alvar_msgs/AlvarMarkers) ~ pose of all markers detected by ar_track_alvar

PARAMETERS:
  + frame_id (string) ~ What frame should measured odometry be reported in (default: "/odom_meas")
  + camera_frame_id (string) ~ What frame is at the camera lens? (default: "/overhead_cam_frame")
  + marker_id (int) ~ Which marker should we be tracking (default: 12)
  + count (int) ~ Number of samples to calibrate (default: 50)
"""
# ROS IMPORTS
import rospy
import rospkg
import tf
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


import numpy as np
import scipy.linalg
import os

# local imports
import angle_utils
import kbhit


FNAME = "odom_frame_broadcaster.launch"


class SystemCalibrator( object ):
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "odom_meas")
        self.camera_frame_id = rospy.get_param("~camera_frame_id", "tracker")
        self.count = rospy.get_param("~count", 300)

        # local vars:
        self.calibrate_flag = False
        self.calibrated = False
        self.calibrate_count = 0
        self.kb = kbhit.KBHit()
        self.trans_arr = np.zeros((self.count, 3))
        self.quat_arr = np.zeros((self.count, 4))
        self.trans_ave = np.zeros(3)
        self.quat_ave = np.zeros(3)
        
        # now create subscribers, timers, and publishers
        self.br = tf.TransformBroadcaster()
        self.kb_timer = rospy.Timer(rospy.Duration(0.1), self.keycb)
        rospy.on_shutdown(self.kb.set_normal_term)
        # self.alvar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.alvarcb)
        self.pose_sub = rospy.Subscriber("tracker_pose", PoseStamped, self.alvarcb)
        
        return

    
    def keycb(self, tdat):
        # check if there was a key pressed, and if so, check it's value and
        # toggle flag:
        if self.kb.kbhit():
            c = self.kb.getch()
            if ord(c) == 27 or c == 'q':
                rospy.signal_shutdown("Escape pressed!")
            else:
                print c
            if c == 'c':
                rospy.loginfo("You pressed 'c'.... calibrating!")
                self.calibrate_count = 0
                self.calibrate_flag = True
                self.calibrated = False
                self.trans_arr = np.zeros((self.count, 3))
                self.quat_arr = np.zeros((self.count, 4))
                self.trans_ave = np.zeros(3)
                self.quat_ave = np.zeros(3)
            elif c == 'w':
                if self.calibrated:
                    rospy.loginfo("Writing launch file")
                    self.write_launch_file()
                    rospy.loginfo("Done writing launch file")
                else:
                    rospy.logwarn("Not calibrated!")
                    rospy.loginfo("Press 'c' to calibrate")
            elif c == 'h':
                print "Press 'c' to calibrate"
                print "Once successfully calibrated, press 'w' to write to a file"
            self.kb.flush()
            # if calibrated, send transform:
        if self.calibrated:
            self.send_tranform()
        return

    def write_launch_file(self):
        r = rospkg.RosPack()
        path = r.get_path("overhead_mobile_tracker")
        fname = os.path.join(path,"launch",FNAME)
        with open(fname, 'w') as f:
            f.write("<launch>\n")
            f.write('    <node pkg="tf" type="static_transform_publisher" name="odom_frame_broadcaster"\n')
            f.write('     args="')
            for num in self.trans_ave:
                f.write('{0:3.4f} '.format(num))
            for num in self.quat_ave:
                f.write('{0:3.4f} '.format(num))
            f.write('{0:s} {1:s} 100" />\n'.format(self.camera_frame_id, self.frame_id))
            f.write('</launch>')
        return
    
    def send_tranform(self):
        self.br.sendTransform(self.trans_ave,
                         self.quat_ave,
                         rospy.Time.now(),
                         self.frame_id,
                         self.camera_frame_id)
        return


    def alvarcb(self, markers):
        if not self.calibrate_flag:
            return
        p = markers.pose
        self.trans_arr[self.calibrate_count,:] = np.array([p.position.x, p.position.y, p.position.z])
        self.quat_arr[self.calibrate_count,:] = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        self.calibrate_count += 1
        if self.calibrate_count%10 == 0:
            rospy.loginfo("calibrating...")
        if self.calibrate_count >= self.count-1:
            rospy.loginfo("Calibration Complete!")
            # we are done with calibration!
            self.calculate_averages()
            self.calibrated = True
            self.calibrate_flag = False
        return

    def calculate_averages(self):
        self.trans_ave = np.mean(self.trans_arr, axis=0)
        Qadj = np.dot(self.quat_arr.T, self.quat_arr)
        (eig, evec) = scipy.linalg.eigh(Qadj, eigvals=(3,3))
        self.quat_ave = evec.ravel()
        return


def main():
    rospy.init_node('system_calibrator', log_level=rospy.INFO)
    rospy.loginfo("Calibration node started")
    rospy.loginfo("Press 'c' to begin calibration")
    try:
        calibrator = SystemCalibrator()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()
