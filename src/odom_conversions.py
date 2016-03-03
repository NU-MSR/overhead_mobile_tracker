import copy
import numpy as np
import tf.transformations as tr
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def odom_to_numpy(odom):
    """
    Convert a nav_msgs/Odometry message into a 3 dimensional Numpy array
    [x,y,theta].
    
    We assume rotation is around body-fixed z-axis.
    """
    arr = np.zeros(3)
    arr[0] = odom.pose.pose.position.x
    arr[1] = odom.pose.pose.position.y
    quat = odom.pose.pose.orientation
    quat_arr = np.array([quat.x, quat.y, quat.z, quat.w])
    arr[2] = tr.euler_from_quaternion(quat_arr, 'sxyz')[2]
    return arr


def numpy_to_odom(arr, frame_id=None):
    """
    Takes in a numpy array [x,y,theta] and a frame ID and converts it to an
    Odometry message. 

    WARN: May require a timestamp.
    """
    odom = Odometry()
    if frame_id is not None: odom.header.frame_id = frame_id
    odom.pose.pose.position.x = arr[0]
    odom.pose.pose.position.y = arr[1]
    odom.pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(arr[2], 0, 0, 'szyx'))
    return odom


def odom_add_offset(odom, offset):
    """
    Takes in two odometry messages and returns a new odometry message that has
    the two added together.

    WARN: Both messages should be in the same frame!
    """
    new_odom = copy.deepcopy(odom)
    new_odom.pose.pose.position.x += offset.pose.pose.position.x
    new_odom.pose.pose.position.y += offset.pose.pose.position.y
    quat = new_odom.pose.pose.orientation
    quat_offset = offset.pose.pose.orientation
    quat_arr = np.array([quat.x, quat.y, quat.z, quat.w])
    offset_arr = np.array([quat_offset.x, quat_offset.y, quat_offset.z, quat_offset.w])
    theta = tr.euler_from_quaternion(quat_arr, 'sxyz')[2]
    theta_offset = tr.euler_from_quaternion(offset_arr, 'sxyz')[2]
    new_odom.pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(theta+theta_offset, 0, 0, 'szyx'))
    return new_odom


