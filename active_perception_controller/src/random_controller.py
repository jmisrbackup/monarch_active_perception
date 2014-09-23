#! /usr/bin/env python
import random
from math import *
import rospy
from geometry_msgs.msg import PoseArray, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pdb

class RandomController():
    def __init__(self):
        self._particles_sub = rospy.Subscriber("particlecloud",
                                               PoseArray,
                                               self.particles_cb)
        self._odom_sub = rospy.Subscriber("odom",
                                          Odometry,
                                          self.odom_cb)
        self._pose_sub = rospy.Subscriber("amcl_pose",
                                          PoseWithCovarianceStamped,
                                          self.pose_cb)
        self._cmd_pub = rospy.Publisher("cmd_vel",
                                        Twist,
                                        queue_size=1)
        self._particles_pub = rospy.Publisher("predicted_particles",
                                              PoseArray,
                                              queue_size=1)
        random.seed()
        self._odom_model = []
        for i in range(1, 6):
            self._odom_model.append(rospy.get_param("odom_alpha"+str(i), 0.0))
        self._particle_set = []
        self._heading = None
        self.hack = False

    def particles_cb(self, msg):
        """
        Magic code goes here <--------
        """
        pdb.set_trace()
        if not self.hack:
            self._particle_set = msg.poses
            self.hack = True

    def pose_cb(self, msg):
        self._heading = 2*asin(msg.pose.pose.orientation.z)

    def odom_cb(self, msg):
        """
        Magic code goes here <--------
        """
        if self._heading is None:
            return

        delta = msg.pose.pose
        trans = hypot(delta.position.x, delta.position.y)
        course = atan2(delta.position.y, delta.position.x) + self._heading
        cs_course = cos(course)
        sn_course = sin(course)

        rot = 2*asin(delta.orientation.z)

        #NOTE: this might not be completely correct in terms of parameters
        # but I'm sticking with the current amcl implementation
        # (which is apparently being fixed)
        # In particular, I think it should use hypot instead of sqrt

        trans_std = sqrt(self._odom_model[2]*(trans**2) +
                         self._odom_model[0]*(rot**2))
        rot_std = sqrt(self._odom_model[3]*(rot**2) +
                       self._odom_model[1]*(trans**2))
        strafe_std = sqrt(self._odom_model[4]*(trans**2) +
                          self._odom_model[0]*(rot**2))

        for p in self._particle_set:
            heading = 2*asin(p.orientation.z)
            trans_sample = random.gauss(trans, trans_std)
            rot_sample = random.gauss(rot, rot_std)
            strafe_sample = random.gauss(0.0, strafe_std)
            if trans_sample > 0.1:
                pass
            x = (trans_sample * cs_course +
                 strafe_sample * sn_course)
            y = (trans_sample * sn_course -
                 strafe_sample * cs_course)
            print '(x,y)', (x, y)
            p.position.x += (trans_sample * cs_course +
                             strafe_sample * sn_course)
            p.position.y += (trans_sample * sn_course -
                             strafe_sample * cs_course)
            heading += rot_sample
            p.orientation.w = cos(heading / 2.0)
            p.orientation.z = sin(heading / 2.0)

        parray = PoseArray()
        parray.poses = self._particle_set
        parray.header.frame_id = '/map'
        self._particles_pub.publish(parray)
        #cmd = Twist()
        #cmd.linear.x = random.random()-0.5
        #cmd.angular.z = random.random()-0.5
        #self.cmd_pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('controller')
    r = RandomController()

    rospy.spin()
