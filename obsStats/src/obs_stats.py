#!/usr/bin/env python  
import roslib
roslib.load_manifest('obsStats')
import rospy
import math
import numpy as np
import tf
import matplotlib.pyplot as plt

if __name__ == '__main__':
    rospy.init_node('obs_stats')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    listener.waitForTransform('/torso_lift_link',\
                                 '/l_gripper_tool_frame',\
                                 rospy.Time(0),rospy.Duration(1.0))
 
    nObs = 400
    staticStats = np.empty([nObs,7])
    for i in range(len(staticStats)):
        if not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/torso_lift_link',\
                                                       '/l_gripper_tool_frame',\
                                                       rospy.Time(0))
                print "Trans:",trans
                print "Rot:",rot
                staticStats[i] = np.append(np.array(trans),np.array(rot))

            except (tf.LookupException, tf.ConnectivityException, \
                        tf.ExtrapolationException):
                print 'fail'
                continue


            rate.sleep()

    print np.mean(staticStats,axis=0)
    print np.var(staticStats,axis=0)

    plt.scatter(staticStats[:,0],staticStats[:,1])
    plt.axis('equal')
    plt.show()

