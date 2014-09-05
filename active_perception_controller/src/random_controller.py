#! /usr/bin/env python
import random

import rospy
from geometry_msgs.msg import PoseArray, Twist
from nav_msgs.msg import Odometry


class RandomController():
    def __init__(self):
        self.particles_sub = rospy.Subscriber("particlecloud",
                                              PoseArray,
                                              self.particles_cb)
        self.odom_sub = rospy.Subscriber("odom",
                                         Odometry,
                                         self.odom_cb)
        self.cmd_pub = rospy.Publisher("cmd_vel",
                                       Twist,
                                       queue_size=1)
        cmd = Twist()
        cmd.linear.x = 0.1
        for t in range(1, 10):
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        random.seed()

    def particles_cb(self, msg):
        """
        Magic code goes here <--------
        """
        cmd = Twist()
        cmd.linear.x = random.random()-0.5
        cmd.angular.z = random.random()-0.5
        self.cmd_pub.publish(cmd)

    def odom_cb(self, msg):
        """
        Magic code goes here <--------
        """
        cmd = Twist()
        cmd.linear.x = random.random()-0.5
        cmd.angular.z = random.random()-0.5
        self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('controller')
    r = RandomController()

    rospy.spin()
