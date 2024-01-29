#!/usr/bin/env python

# republish odom generated by the simulation planar plugin. Set covariance matrix to zero


import rospy


from nav_msgs.msg import Odometry

import numpy as np


def odom_filt(odom):

   odom_nocov = Odometry()

   odom_nocov.header = odom.header
   odom_nocov.child_frame_id = odom.child_frame_id

   odom_nocov.pose.pose = odom.pose.pose
   odom_nocov.twist.twist = odom.twist.twist

   odom_repub.publish(odom_nocov)



if __name__ == '__main__':
    rospy.init_node('odom_repub')
    
    rospy.Subscriber('/odom_sim', Odometry, odom_filt)

    odom_repub = rospy.Publisher("/odom", Odometry, queue_size=10)

    rospy.spin()