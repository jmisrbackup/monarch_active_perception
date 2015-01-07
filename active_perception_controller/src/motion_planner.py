#!/usr/bin/env python

from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import *
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
import pdb
import rospy
import roslib
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg._PointCloud import PointCloud
from sklearn.neighbors import NearestNeighbors

import time
import threading
import numpy as np
import scipy as sp
import scipy.ndimage
from active_perception_controller import ap_utility

from StringIO import StringIO

class MotionPlanner():
    def __init__(self):        
        self._lock = threading.Lock()
        self._lock.acquire()
        
        self._robot_pose_sub = rospy.Subscriber("amcl_pose",
                                                PoseWithCovarianceStamped,
                                                self.robot_pose_cb,
                                                queue_size=1)
        # TODO: change the particles topic name
        self._target_particles_sub = rospy.Subscriber("person_particle_cloud",
                                                      PointCloud,
                                                      self.target_particle_cloud_cb,
                                                      queue_size=1)
        self._robot_pose = PoseWithCovariance()
        
        self._test_pub = rospy.Publisher("rrt",
                                         Path,
                                         queue_size=1,
                                         latch = True)
        
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
        self._freecells = [i for i in xrange(0,len(self._navmap.data)) 
                           if self._navmap.data[i] == 0]
        
        self._rrt_eta = rospy.get_param("rrt_eta", 1.0) # Notation from Karaman & Frazolli, 2011
        self._rrt_lim = rospy.get_param("rrt_lim", 100)
        
        self._planned = False # This is just for testing purposes. Delete me!
        mapdata = np.asarray(self._navmap.data, dtype=np.int8).reshape(height, width)
        logical = np.flipud(mapdata == 0)
        self._distmap = sp.ndimage.distance_transform_edt(logical)
        
        pkgpath = roslib.packages.get_pkg_dir('active_perception_controller')
        self.utility_function = ap_utility.Utility(str(pkgpath) + "/config/sensormodel.png",
                                                   0.1034)
        self._lock.release()
        
    def robot_pose_cb(self, msg):
        self._robot_pose = msg.pose

    def target_particle_cloud_cb(self, msg):
        self._lock.acquire()   
        buf = StringIO()
        msg.serialize(buf)
        self.utility_function.setPersonParticles(buf.getvalue())
        
        self.plan()
        self._lock.release()

    def plan(self):        
        if not self._planned:
            self.rrtstar()
            self._planned = True
        
    def rrt(self):
        """
        Basic RRT Algorithm
        """
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])
        V = [probot]
        E = {}
        nbrs = NearestNeighbors(n_neighbors=1)
        nbrs.fit(V)
        t1 = time.time()
        while len(V) < self._rrt_lim:
            prand = self.sample_free_uniform()
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
                nbrs.fit(V)
        print 'total time: ', time.time()-t1
        self.publish_rrt(V,E) 

    def rrtstar(self):
        """
        RRT* Algorithm
        """
        vol_freecells = len(self._freecells)*self._navmap.info.resolution**2
        gamma_rrg = 2*sqrt(1.5*vol_freecells/pi)
        probot = np.array([self._robot_pose.pose.position.x,self._robot_pose.pose.position.y])
        V = [probot]
        E = {}
        parents = {}
        C = [0.0]
        nbrs = NearestNeighbors(n_neighbors=1)
        nbrs.fit(V)
        cmin = 0
        t1 = time.time()
        
        #u = ap_utility.computeInfoGain(targets, self._robot_pose, 0)
        
        while len(V) < self._rrt_lim:
            t2 = time.time()
            """
            Sampling new point
            """
            prand = self.sample_free_uniform()      
            (dist, idx) = nbrs.kneighbors(prand)
            pnearest_idx = idx.flatten(1)[0]
            pnearest = V[pnearest_idx]
            """
            Turning new point into reachable point
            """
            if dist < self._rrt_eta:
                pnew = prand
            else:
                pnew = self.steer(pnearest, prand)
            """
            Checking if segment is valid and updating graph
            """
            if self.segment_safe(V[pnearest_idx],pnew) is True:
                r = np.min([gamma_rrg*sqrt(log(len(V))/float(len(V))),self._rrt_eta])
                Pnear_idx = nbrs.radius_neighbors(pnew, r, return_distance = False)
                Pnear_idx = Pnear_idx[0]
                pmin_idx = pnearest_idx
                i = self.utility_function.computeInfoGain(pnew[0], pnew[1])
                cmin = C[pnearest_idx] + np.linalg.norm(pnearest-pnew)
                for p_idx in Pnear_idx:
                    c = C[p_idx] + np.linalg.norm(V[p_idx]-pnew)
                    if (self.segment_safe(V[p_idx],pnew) is True and 
                        c < cmin):
                        cmin = c
                        pmin_idx = p_idx
                
                if E.has_key(pmin_idx):
                    E[pmin_idx].add(len(V))
                else:
                    E[pmin_idx] = set([len(V)])
                pnew_idx = len(V)
                V.append(pnew)
                C.append(cmin)
                parents[pnew_idx] = pmin_idx
                """
                Re-wire the tree
                """
                for p_idx in Pnear_idx:
                    if (self.segment_safe(V[p_idx],pnew) is True and 
                        cmin + np.linalg.norm(V[p_idx]-pnew) < C[p_idx]):
                        E[parents[p_idx]].remove(p_idx)
                        parents[p_idx] = pnew_idx
                        if E.has_key(pnew_idx):
                            E[pnew_idx].add(p_idx)
                        else:
                            E[pnew_idx] = set([p_idx])              
                nbrs.fit(V)
            print 'iteration done. time: ', time.time()-t2
        print 'total time: ', time.time()-t1
        self.publish_rrt(V,E) 

    def publish_rrt(self, V,E):
        pt = Path()
        pt.header.frame_id = '/map'
        path = []
        vis = set()
        self.gen_path(0, 0, V, E, path, vis)
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = '/map'
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pt.poses.append(pose)
        self._test_pub.publish(pt)
        
    def gen_path(self, ix, p_ix, V, E, path, vis ):
        path.append(V[ix])
        vis.add(ix)
        if E.has_key(ix):
            for c in E[ix]:
                if c not in vis:
                    self.gen_path(c, ix, V, E, path, vis )
        path.append(V[p_ix])
   
    def steer(self, org, dst):
        alpha = atan2(dst[1]-org[1],
                      dst[0]-org[0])
        new = org + self._rrt_eta*np.array([cos(alpha),sin(alpha)])
        return new
                
    def segment_safe(self, org, dst):
        xo = self._navmap.info.origin.position.x
        yo = self._navmap.info.origin.position.y
        res = self._navmap.info.resolution

        org_idx = np.array([int((org[0] - xo)/res),int(-(org[1] + yo)/res)])
        dst_idx = np.array([int((dst[0] - xo)/res),int(-(dst[1] + yo)/res)])
        # t1 = time.time()
        v1 = self.distance_transform_search(org_idx, dst_idx)
        # t2 = time.time()
        # v2 = self.greedy_cardinal_search(org_idx, dst_idx)
        # t3 = time.time()
        # v3 = self.directed_search(org_idx, dst_idx)
        # t4 = time.time()
        # print '----------------------------------------'
        # print 'dist transform: ',v1,' t: ',t2-t1
        # print 'greedy search: ',v2,' t: ',t3-t2
        # print 'directed search: ',v3,' t: ',t4-t3
        return v1
    
    def distance_transform_search(self, org_idx, dst_idx):
        alpha = atan2(dst_idx[1]-org_idx[1],
                      dst_idx[0]-org_idx[0])
        ca = cos(alpha)
        sa = sin(alpha)
        ridx = org_idx
        while not np.all(ridx == dst_idx):
            dist = self._distmap[int(ridx[1]),
                                 int(ridx[0])]
            if dist == 0:
                return False
            elif np.linalg.norm(ridx - dst_idx) < dist:
                return True
            else:
                ridx = ridx + np.array([ca, sa])*dist
        return True
    
    def directed_search(self, org_idx, dst_idx):
        alpha = atan2(dst_idx[1]-org_idx[1],
                      dst_idx[0]-org_idx[0])
        ca = cos(alpha)
        sa = sin(alpha)   
        idx = org_idx
        ridx = idx
        while not np.all(ridx == dst_idx):
            idx = idx + np.array([ca, sa])
            ridx = np.floor(idx)
            linear_idx = int(((self._navmap.info.height-ridx[1]-1)*self._navmap.info.width
                              + ridx[0]))
            if self._navmap.data[linear_idx] != 0:
                return False
        return True
    
    def greedy_cardinal_search(self, org_idx, dst_idx, prev_idx = None): 
        if np.all(org_idx == dst_idx):
            return True
      
        dir = np.mat('1 0; -1 0; 0 1; 0 -1; 1 1; -1 -1; 1 -1; -1 1')
         
        next = org_idx + dir
        norms = np.linalg.norm(next - dst_idx, 2, 1)
        best_idx = np.asarray(next[np.argmin(norms),:]).flatten(1)
        best_linear_idx = ((self._navmap.info.height-best_idx[1]-1)*self._navmap.info.width
                           + best_idx[0])
        if self._navmap.data[best_linear_idx] == 0:
            return self.greedy_cardinal_search(best_idx, dst_idx, org_idx)
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