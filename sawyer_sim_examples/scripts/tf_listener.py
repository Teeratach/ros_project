#!/usr/bin/env python  
import roslib
roslib.load_manifest('sawyer_sim_examples')

import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('Test_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():


        try:            
            (trans,rot) = listener.lookupTransform('/world', '/object_2', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print 'x = %s' %trans[0]
        print 'y = %s' %trans[1]
        print 'z = %s' %trans[2]


        rate.sleep()
