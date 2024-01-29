#!/usr/bin/env python

# subscribes to cmd vel and output cmd vel to the IDEA (infinite accle) move_planar gazebo plugin.
# Implements acceleration (and velocity) constraints


import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import numpy as np



# Parameters

cmd_vel_sim_topic = '/cmd_vel_sim'
odom_sim_topic = '/odom_sim'
cmd_vel_real_topic = '/cmd_vel'

cntrl_rate = 100 #Hz simulated controller rate


# Acceleration and velocity constraints (see pbl_robodog_sysid)

# measured, prevents the simulated robot to accel or brake faster than these values
MAX_ACC_X = 0.49
MAX_ACC_Y = 0.231 
MAX_ACC_THETA = 3.742

# arbitrary limits, prints warning if overshoot
MAX_VX = 2.0
MIN_VX = -1.0
MAX_VY = 1.0 
MAX_VTHETA = 3


class CmdVelController:

   def __init__(self):
      

      rospy.Subscriber(cmd_vel_real_topic, Twist, self.vel_control_cb)

      self.cmd_vel_pub = rospy.Publisher(cmd_vel_sim_topic, Twist, queue_size=10)

      #keeps track of speeds
      self.vx = 0
      self.vy = 0
      self.vtheta = 0

      self.vx_ref = 0
      self.vy_ref = 0
      self.vtheta_ref = 0


   def mainLoop(self):
        
      rate = rospy.Rate(cntrl_rate)
      dt = 1.0/cntrl_rate
      counter = 0

      cmd_vel_sim = Twist()

      while not rospy.is_shutdown():
         

         # bring actual vels closer to reference vels according to accels
         dvx = np.clip(self.vx_ref-self.vx,-MAX_ACC_X*dt,MAX_ACC_X*dt)
         dvy = np.clip(self.vy_ref-self.vy,-MAX_ACC_Y*dt,MAX_ACC_Y*dt)
         
         dvtheta = np.clip(self.vtheta_ref-self.vtheta,-MAX_ACC_THETA*dt,MAX_ACC_THETA*dt)

         # update vels
         self.vx = self.vx + dvx
         self.vy = self.vy + dvy
         self.vtheta = self.vtheta + dvtheta

         # check and correct for max vel violations
         if self.vx > MAX_VX:
            rospy.logwarn("Exceeded max vx!")
            self.vx = MAX_VX
         elif self.vx < MIN_VX:
            rospy.logwarn("Exceeded min vx!")
            self.vx = MIN_VX
         if abs(self.vy) > MAX_VY:
            rospy.logwarn("Exceeded max/min vy!")
            self.vy = np.clip(self.vy,-MAX_VY,MAX_VY)
         if abs(self.vtheta) > MAX_VTHETA:
            rospy.logwarn("Exceeded max/min vtheta!")
            self.vtheta = np.clip(self.vtheta,-MAX_VTHETA,MAX_VTHETA)


         #publish cmd_vel to gazebo (ideal holonomic controller, planar_move)
         cmd_vel_sim.linear.x = self.vx
         cmd_vel_sim.linear.y = self.vy
         cmd_vel_sim.angular.z = self.vtheta
         self.cmd_vel_pub.publish(cmd_vel_sim)


         # if gazebo world is reset during the rate.sleep() execution, will get ROSTimeMovedBackwardsException
         # so do exception handling
         try:
            rate.sleep()
         except rospy.ROSTimeMovedBackwardsException:
            rospy.logwarn("ROS Time Backwards! Just ignore the exception!")




   def vel_control_cb(self,cmd_vel):

      # set target vels
      self.vx_ref = cmd_vel.linear.x
      self.vy_ref = cmd_vel.linear.y
      self.vtheta_ref = cmd_vel.angular.z


if __name__ == '__main__':
    rospy.init_node('cmd_vel_controller')
    
    CmdVelController().mainLoop()