#!/usr/bin/env python

from math import *
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from nav_msgs.srv import GetMap
import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg._PointCloud import PointCloud
from sklearn.neighbors import NearestNeighbors
import time
from geometry_msgs.msg import Point32
import pdb

class MotionPlanner():
    def __init__(self):
        self._robot_pose_sub = rospy.Subscriber("amcl_pose",
                                                PoseWithCovarianceStamped,
                                                self.robot_pose_cb,
                                                queue_size=1)
        # TODO: change the particles topic name
        self._robot_pose_sub = rospy.Subscriber("particle_cloud",
                                                PointCloud,
                                                self.target_particle_cloud_cb,
                                                queue_size=1)
        self._robot_pose = Pose()
        
#         self._test_pub = rospy.Publisher("test",
#                                          PointCloud,
#                                          queue_size=1)
        
        getmap = rospy.ServiceProxy('static_map', GetMap)
        
        srv_available = False
        while not srv_available:
            try:
                rospy.wait_for_service('static_map',2.0)
                srv_available = True
            except rospy.exceptions.ROSException as e:
                rospy.logwarn(e.message)
        self._navmap = getmap().map
        width = self._navmap.info.width
        height = self._navmap.info.height
        self._freecells = [i for i in range(0,len(self._navmap.data)) 
                           if self._navmap.data[i] == 0]
        self._rrt_eta = rospy.get_param("rrt_eta", 0.5) # Notation from Karaman & Frazolli, 2011
    
    def robot_pose_cb(self, msg):
        self._robot_pose = msg.pose.pose

    def target_particle_cloud_cb(self, msg):
        c = [[msg.points[i].x, msg.points[i].y]
             for i in range(0,len(msg.points))]
        targets = np.asarray(c)
        self.plan(targets)

    def plan(self, targets):
        self.rrt(targets)

    def rrt(self,targets):
        probot = np.array([self._robot_pose.position.x,self._robot_pose.position.y])
        V = [probot]
        E = {}
        nbrs = NearestNeighbors(n_neighbors=1)
        
        #px = X[idx].flatten()
        #magic goes here
        for n in range(1,100):
            """
            Basic RRT Algorithm
            """
            prand = self.sample_free_uniform()
            nbrs.fit(V)
            (dist, idx) = nbrs.kneighbors(prand)
            idx = idx.flatten(1)[0]
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(V[idx], prand)
            if self.segment_safe(V[idx],pnew) is True:
                if E.has_key(idx):
                    E[idx].append(len(V))
                else:
                    E[idx] = [len(V)]
                V.append(pnew)
        pdb.set_trace()
   
    def steer(self, org, dst):
        alpha = atan2(dst[1]-org[1],
                      dst[0]-org[0])
        new = org + self._rrt_eta*np.array([cos(alpha),sin(alpha)])
        return new
                
    def segment_safe(self, org, dst):
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y
        res = self._navmap.info.resolution
        h = self._navmap.info.height
        
        org_idx = np.array([int((org[0] - xo)/res),int((org[1] + yo)/res + h)])
        dst_idx = np.array([int((dst[0] - xo)/res),int((dst[1] + yo)/res + h)])
        return self.greedy_cardinal_search(org_idx, dst_idx)
                
    def greedy_cardinal_search(self, org_idx, dst_idx):
        if np.all(org_idx == dst_idx):
            return True

        #TODO: at least 6 of these directions can be ommitted given prior knowledge        
        dir = np.mat('1 0; -1 0; 0 1; 0 -1; 1 1; -1 -1; 1 -1; -1 1')
        
        print org_idx
        
        next = org_idx + dir
        norms = np.linalg.norm(next - dst_idx, 2, 1)
        best_idx = np.asarray(next[np.argmin(norms),:]).flatten(1)

        best_linear_idx = best_idx[1]*self._navmap.info.width+best_idx[0]
        if self._navmap.data[best_linear_idx] == 0:
            return self.greedy_cardinal_search(best_idx, dst_idx)
        else:
            return False
        
    def sample_free_uniform(self):
        r = np.random.uniform(0,len(self._freecells),1)
        idx = self._freecells[int(r)]
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y
        res = self._navmap.info.resolution
        w = self._navmap.info.width
        h = self._navmap.info.height
        x = (idx%w)*res+xo
        y = ((idx-x)/h)*res+yo
        return np.array([x,y])
        
        
if __name__=='__main__':
    rospy.init_node('entropy_motion_planner')
    m = MotionPlanner()
    
    rospy.spin()