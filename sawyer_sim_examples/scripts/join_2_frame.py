#!/usr/bin/env python  
import roslib
roslib.load_manifest('sawyer_sim_examples')

import argparse
import struct
import sys
import copy

from math import pi
import rospy
import tf
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

if __name__ == '__main__':
    rospy.init_node('test2_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, (pi), 0))
    print orientation

    while not rospy.is_shutdown():
        br.sendTransform((0.75, 0.0, 1.0),
                         (0.0, 1.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "camera_link",
                         "world")
        rate.sleep()
