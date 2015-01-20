#!/usr/bin/env python

import sys
import os.path
import time
import math
import random
import cPickle as pickle

import numpy as np
import scipy.linalg
import scipy.signal

import roslib
SCOUT_NAVIGATION_DIR = roslib.packages.get_pkg_dir('scout_navigation')
MAPS_DIR = roslib.packages.get_pkg_dir('maps')

import rospy
import rosbag
import tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append( SCOUT_NAVIGATION_DIR )
import planner



DEFAULT_MAP = os.path.join(MAPS_DIR, "EPFL-v02cr_nav.yaml")
MAP_FRAME = 'map'
DEFAULT_CLEARANCE = 0.5
N_PARTICLES = 500
FACTOR = 3

# sensor model
ANGLE_DELTA = 0.003 # hacked for Hokuyo (approx angle_increment/2)
# observation model
OCC_LOGPROB  =  5.0
FREE_LOGPROB = -1.0



def fix_angle(a):
    pi = math.pi
    return (a+pi)%(2*pi) - pi


class PathParticleFilter:
    # random sample injection
    RANDOM_FRAC = 0 # 0.01

    points = None  # [[x,y], ...]
    logprob = None  # [index]
    
    def __init__(self, path, factor):
        """path is a list of pairs (x,y); factor w.r.t. path length"""
        self.path = path
        n = len(path)
        p = np.array(path)
        d = np.sqrt(np.sum((p[1:]-p[:-1])**2, axis=1))
        A = np.vstack([np.zeros(n-1),
                       np.tri(n-1)])
        cum = np.dot(A, d)
        plen  = cum[-1]
        total = int(math.ceil(factor*plen))
        # samples in total path len units
        l = plen*np.random.uniform(size=total)
        # samples segment indices
        i = (cum[:,None] <= l[None,:]).sum(axis=0) - 1
        # and offsets
        o = l - cum[i]
        self.points = p[i] + (p[i+1]-p[i])*(o/d[i])[:,None]
        self.logprob = np.zeros(total)
        #print "path: length=%s, took %s samples"%(n,total)

    def apply_importance(self, importance):
        """importance is an array of log prob increments"""
        self.logprob += importance

    def resample(self):
        """low-variance resampling implementation"""
        if len(self.logprob)>0:
            # get normalized probabilities
            n = int(math.ceil( self.RANDOM_FRAC*len(self.logprob) ))
            m = len(self.logprob) - n
            w = np.exp(self.logprob)
            w /= w.sum()
            # re-sample m samples
            si  = scipy.linalg.toeplitz(w, np.zeros_like(w)).sum(axis=1)
            rj  = (np.random.random() + np.arange(m)) / m
            dij = si[:,None] - rj[None,:]
            dij[dij<0] = np.inf
            k = dij.argmin(axis=0)
            resampled = self.points[k]
            if n>0:
                # generate n new samples
                total = self.cells.shape[1]
                samples = [ tuple(self.cells[:,random.randint(0,total-1)]) for i in xrange(n) ]
                new = np.array(samples, dtype=int)
                self.points = np.vstack((resampled, new))
            else:
                self.points = resampled
            self.logprob = np.zeros(len(self.points))

    # TODO:
    def diffusion(self):
        raise Exception, "Not yet implemented"

class MapParticleModel:
    filters = None
    paths = cells = None
    
    def __init__(self, mapfile, clearance=DEFAULT_CLEARANCE):
        self.pln = planner.FastMarching()
        self.pln.load_map(mapfile)
        self.pln.setup()
        maxd = clearance/self.pln.scale
        self.feasible = self.pln.distmap > maxd
        candidates = self.feasible.nonzero()
        imin = candidates[0].min()
        imax = candidates[0].max()
        jmin = candidates[1].min()
        jmax = candidates[1].max()
        (xmin, ymax) = self.pln.i2p((imin, jmin))
        (xmax, ymin) = self.pln.i2p((imax, jmax))
        self.bbox = (xmin, xmax, ymin, ymax)

    def sample_points(self, total):
        """generate positions in map free space"""
        points  = []
        (xmin, xmax, ymin, ymax) = self.bbox
        while len(points)<total:
            x = random.uniform(xmin, xmax)
            y = random.uniform(ymin, ymax)
            (i, j) = self.pln.p2i((x, y))
            if self.feasible[i,j]:
                points.append((x, y))
        return points

    # def gen_paths_full(self, total):
    #     """generate paths in a fully connected graph"""
    #     points = self.sample_points(total)
    #     self.paths = []
    #     for (i,g) in enumerate(points[:-1]):
    #         print "-- Solving for particle %d of %d"%(i+1,len(points))
    #         t1 = time.time()
    #         self.pln.solve(g)
    #         for o in points[i+1:]:
    #             p = np.array( self.pln.get_path(o[0], o[1], step=self.pln.scale) )
    #             self.paths.append(p)
    #         t2 = time.time()
    #         print "   took %f ms"%(1000*(t2-t1))

    def gen_paths_pair(self, total):
        """generate paths between pairs of samples"""
        points = self.sample_points(total)
        self.paths = []
        for k in xrange(0, len(points)-1, 2):
            (o, g) = points[k:k+2]
            if self.pln.solve(g) and self.pln.validate(o):
                p = np.array( self.pln.get_path(o[0], o[1], step=self.pln.scale) )
                self.paths.append(p)
            else:
                print "*** discarding pair", o, g

    def sample_paths(self, factor):
        """create factor*L particle for each path and create a cell->path dict"""
        assert self.paths is not None, "no paths were generated"
        self.filters = [ PathParticleFilter(p, factor) for p in self.paths ]

    def compute_importance(self, scan, pose):
        assert self.filters is not None, "no filters were generated yet"
        ## prepare data
        (position, quaternion) = pose
        z = np.array(scan.ranges)
        a = np.arange(scan.angle_min, scan.angle_min+scan.angle_increment*len(z), scan.angle_increment)
        assert len(z)==len(a), "Range and angle arrays do not match"
        # discard invalid ranges
        valid = np.isfinite(z)
        zv = z[valid]
        valid[valid] = (zv>scan.range_min) & (zv<scan.range_max)
        assert valid.any(), "No valid scan line found"
        zi, ai = z[valid], a[valid]
        # obtain laser pointcloud in device coordinates
        xl = zi*np.cos(ai)
        yl = zi*np.sin(ai)
        print "laser points:", len(xl), "=", len(yl)
        # transform pointcloud according to tf
        (xr, yr, zr) = position
        T = tf.transformations.quaternion_matrix(quaternion)
        R = T[0:3,0:3]
        pl = np.vstack([xl, yl, np.zeros_like(zi)])
        pb = np.dot(R, pl)
        xi = pb[0,:] + xr
        yi = pb[1,:] + yr
        ti = np.arctan2(pb[1,:], pb[0,:])
        # compute hit radius
        ri = zi * math.sin(ANGLE_DELTA)
        # compute bounding box
        xmin = (xi-ri).min()
        xmax = (xi+ri).max()
        ymin = (yi-ri).min()
        ymax = (yi+ri).max()
        print "limits:", xmin, xmax, ymin, ymax
        # gather particles -- array of [x, y, filter_index, point_index]
        particles = np.vstack( [ np.hstack( [ pf.points,
                                              i*np.ones(len(pf.points))[:,None],
                                              np.arange(len(pf.points))[:,None] ] )
                                 for (i,pf) in enumerate(self.filters) ] )
        print "all particles:", particles.shape
        # select particles within bounding box
        within = reduce(np.logical_and, [ particles[:,0]>=xmin,
                                          particles[:,0]<=xmax,
                                          particles[:,1]>=ymin,
                                          particles[:,1]<=ymax ] )
        particles_within = particles[within]
        pj = particles_within[:,0]
        qj = particles_within[:,1]
        print "particles in bbox:", particles_within.shape
        # select particles found occupied
        occ_hits = ( (xi[:,None]-pj[None,:])**2 + (yi[:,None]-qj[None,:])**2 <= ri[:,None]**2 ).any(axis=0)
        print "occ_hits:", occ_hits.shape, occ_hits.sum()
        # select particles found free
        bj  = np.arctan2(qj-yr, pj-xr)
        dj2 = (pj-xr)**2 + (qj-yr)**2
        free_hits = np.logical_and( np.abs(fix_angle(bj[None,:]-ti[:,None])) <= ANGLE_DELTA,
                                    dj2[None,:] < zi[:,None]**2 ).any(axis=0)
        print "free_hits:", free_hits.shape, free_hits.sum()
        # part of particles_within table which fall within free and occupied zones
        parts_free = particles_within[free_hits]
        parts_occ  = particles_within[occ_hits]
        for (k,pf) in enumerate(self.filters):
            importance = np.zeros(len(pf.points))
            if len(parts_free)>0:
                # update particles in free zone
                free_mask = (parts_free[:,2] == k)
                free_idx  = parts_free[free_mask,3].astype(int)
                importance[free_idx] = FREE_LOGPROB
            if len(parts_occ)>0:
                # update particles in occupied zone
                occ_mask = (parts_occ[:,2] == k)
                occ_idx  = parts_occ[occ_mask,3].astype(int)
                importance[occ_idx] = OCC_LOGPROB
            #print k, importance
            pf.apply_importance(importance)
        
    def resample(self):
        assert self.filters is not None, "no filters were generated yet"
        for p in self.filters:
            p.resample()




        
def test1():
    """simple particle generation"""
    mpm = MapParticleModel(DEFAULT_MAP)
    particles = mpm.sample_points(20)
    for p in particles:
        print p


        
def test2(state=None):
    """path filters generation"""
    if state is None:
        mpm = MapParticleModel(DEFAULT_MAP)
        mpm.gen_paths_pair(N_PARTICLES)
        with open("path_filters.state", 'w') as fh:
            pickle.dump(mpm, fh)
        print "Saved state to path_filters.state"
    else:
        with open(state) as fh:
            mpm = pickle.load(fh)
        #
        l, r, b, t = mpm.pln.x0, mpm.pln.x0+mpm.pln.scale*(mpm.pln.W-1), mpm.pln.y0, mpm.pln.y0+mpm.pln.scale*(mpm.pln.H-1)
        plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r, extent=(l,r,b,t))
        for p in mpm.paths:
            plt.scatter(p[:,0], p[:,1], marker='.', linewidths=0)
        #
        plt.show()


def test3(state="path_filters.state"):
    """sample paths; arguments: [state]"""
    # 1. load paths
    with open(state) as fh:
        print "Loading filter state from", state
        mpm = pickle.load(fh)
    # 2. initial path samples
    print "Initial sampling"
    mpm.sample_paths(FACTOR)
    #
    l, r, b, t = mpm.pln.x0, mpm.pln.x0+mpm.pln.scale*(mpm.pln.W-1), mpm.pln.y0, mpm.pln.y0+mpm.pln.scale*(mpm.pln.H-1)
    plt.subplot(121)
    plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r, extent=(l,r,b,t))
    for p in mpm.paths:
        plt.scatter(p[:,0], p[:,1], marker='.', linewidths=0)
    plt.subplot(122)
    plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r, extent=(l,r,b,t))
    for f in mpm.filters:
        plt.scatter(f.points[:,0], f.points[:,1], marker='.', linewidths=0)
    plt.show()


def test4(state="path_filters.state", scan="scan.state"):
    """one cycle of path particle filter; arguments: [state [scan]]"""
    # 1. load paths
    with open(state) as fh:
        print "Loading filter state from", state
        mpm = pickle.load(fh)
    # 2. initial path samples
    print "Initial sampling"
    mpm.sample_paths(FACTOR)
    # 3. load scan
    with open(scan) as fh:
        data = pickle.load(fh)
        scan = data['scan']
        pose = data['pose']
    # 4. compute importance
    mpm.compute_importance(scan, pose)
    #
    ## prepare data
    (position, quaternion) = pose
    z = np.array(scan.ranges)
    a = np.arange(scan.angle_min, scan.angle_min+scan.angle_increment*len(z), scan.angle_increment)
    assert len(z)==len(a), "Range and angle arrays do not match"
    # discard invalid ranges
    valid = np.isfinite(z)
    zv = z[valid]
    valid[valid] = (zv>scan.range_min) & (zv<scan.range_max)
    assert valid.any(), "No valid scan line found"
    zi, ai = z[valid], a[valid]
    # obtain laser pointcloud in device coordinates
    xl = zi*np.cos(ai)
    yl = zi*np.sin(ai)
    print "laser points:", len(xl), "=", len(yl)
    # transform pointcloud according to tf
    (xr, yr, zr) = position
    T = tf.transformations.quaternion_matrix(quaternion)
    R = T[0:3,0:3]
    pl = np.vstack([xl, yl, np.zeros_like(zi)])
    pb = np.dot(R, pl)
    xi = pb[0,:] + xr
    yi = pb[1,:] + yr
    # gather particles -- array of [x, y, logprob]

    # -- DEAD CODE BELOW --
    # if False:
    #     for i in xrange(len(xi)):
    #         coords = np.array([[xr, yr],
    #                            [xr + zi[i]*math.cos(ti[i]-ANGLE_DELTA), 
    #                             yr + zi[i]*math.sin(ti[i]-ANGLE_DELTA)],
    #                            [xr + zi[i]*math.cos(ti[i]+ANGLE_DELTA), 
    #                             yr + zi[i]*math.sin(ti[i]+ANGLE_DELTA)]])
    #         p = plt.Polygon(coords, color='g', lw=0)
    #         plt.gca().add_artist(p)
    #         c = plt.Circle((xi[i],yi[i]), ri[i], color='r', lw=0)
    #         plt.gca().add_artist(c)
    # if False:
    #     for j in xrange(len(pj)):
    #         l = plt.Line2D([xr, xr+math.sqrt(dj2[j])*math.cos(bj[j])],
    #                        [yr, yr+math.sqrt(dj2[j])*math.sin(bj[j])])
    #         plt.gca().add_artist(l)
    # if False:
    #     plt.scatter([xr], [yr], c='k')
    #     plt.scatter(xi, yi, c='m', marker='.', linewidth=0)
    #     colors = np.empty_like(free_hits, dtype=str)
    #     colors[:] = 'b'
    #     colors[free_hits] = 'g'
    #     colors[occ_hits]  = 'r'
    #     plt.scatter(pj, qj, c=colors, marker='.', linewidth=0)
    #     plt.show()
    # --//--

    pln = mpm.pln
    l, r, b, t = pln.x0, pln.x0+pln.scale*(pln.W-1), pln.y0, pln.y0+pln.scale*(pln.H-1)
    #
    plt.subplot(121)
    particles = np.vstack( [ np.hstack( [ pf.points, pf.logprob[:,None] ] )
                             for pf in mpm.filters ] )
    plt.imshow(pln.occgrid, cmap=cm.gray_r, extent=(l,r,b,t))
    plt.scatter([xr], [yr], c='k')
    plt.scatter(xi, yi, c='m', marker='.', linewidth=0)
    plt.scatter(particles[:,0], particles[:,1], c=particles[:,2], marker='.', linewidth=0)
    plt.gca().axes.set_aspect('equal')

    # 5. resample
    mpm.resample()
    #

    plt.subplot(122)
    particles = np.vstack( [ np.hstack( [ pf.points, pf.logprob[:,None] ] )
                             for pf in mpm.filters ] )
    plt.imshow(pln.occgrid, cmap=cm.gray_r, extent=(l,r,b,t))
    plt.scatter([xr], [yr], c='k')
    plt.scatter(xi, yi, c='m', marker='.', linewidth=0)
    plt.scatter(particles[:,0], particles[:,1], c=particles[:,2], marker='.', linewidth=0)
    plt.gca().axes.set_aspect('equal')

    plt.show()


    # plt.subplot(133)
    # img = np.zeros(mpm.pln.occgrid.shape)
    # for p in mpm.filters:
    #     img[p.points[:,0], p.points[:,1]] = 1
    # plt.imshow(img, cmap=cm.gray_r)
    #

    
    # 6. diffuse samples


    #
    # plt.subplot(221)
    # plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r)
    #
    # plt.subplot(222)
    # img = np.zeros(mpm.pln.occgrid.shape)
    # (im, jm) = mpm.find_swept_cells(scan, pose)
    # img[im,jm] = 1
    # plt.imshow(img, cmap=cm.gray_r)
    #
    #
    plt.show()

   

def test5(state="scan.state"):
    """extract a scan.state from the first scan found"""
    scan = [None]
    pose = [None]
    def handler(msg):
        if scan[0] is None or pose[0] is None:
            try:
                frame = msg.header.frame_id
                pose[0] = tfl.lookupTransform(MAP_FRAME, frame, rospy.Time())
                scan[0] = msg
                print "got scan from %s to %s"%(MAP_FRAME, frame)
                rospy.signal_shutdown("got scan")
            except tf.Exception:
                print "can't transform from %s to %s, still trying..."%(MAP_FRAME, frame)
    rospy.init_node('test5', anonymous=True, argv=sys.argv)
    tfl = tf.TransformListener()
    sub = rospy.Subscriber("scan", LaserScan, handler)
    print "Waiting for scans"
    rospy.spin()
    if scan[0] is not None and pose[0] is not None:
        with open(state, 'w') as fh:
            state = dict(scan=scan[0], pose=pose[0])
            pickle.dump(state, fh)





def test6(state="path_filters.state"):
    """closed loop experiment"""
    scans = {}
    #
    def lidar_handler(msg):
        frame = msg.header.frame_id
        try:
            pose = tfl.lookupTransform(MAP_FRAME, frame, rospy.Time())
        except tf.Exception:
            return
        scans[frame] = (pose, msg)
    #
    def publish_particles():
        points = []
        logprob = []
        #n = 0
        for pp in mpm.filters:
            (xx, yy) = mpm.pln.np_i2p((pp.points[:,0], pp.points[:,1]))
            points.extend( [Point32(xx[i],yy[i],0) for i in xrange(len(pp.points))] ) # if pp.logprob[i]>=0] )
            logprob.extend(pp.logprob)
            #n += (pp.logprob<0).sum()
        #print "hidding", n, "filters"
        cloud = PointCloud()
        cloud.header.frame_id = MAP_FRAME
        cloud.points = points
        cloud.channels = [ ChannelFloat32(name='logprob', values=logprob) ]
        pub.publish(cloud)
    #
    with open(state) as fh:
        print "Loading filter state from", state
        mpm = pickle.load(fh)
    #
    print "Initial sampling"
    mpm.sample_paths(FACTOR)
    print len(mpm.filters), "filters, total of",
    print sum([len(p.points) for p in mpm.filters]), "particles"
    #
    rospy.init_node('test6', argv=sys.argv)
    tfl = tf.TransformListener()
    sub = rospy.Subscriber("scan", LaserScan, lidar_handler)
    pub = rospy.Publisher("particles", PointCloud, queue_size=1)
    rate = rospy.timer.Rate(1)
    print "Node ready -- starting loop"
    publish_particles()
    while not rospy.is_shutdown():
        for (frame,(pose,scan)) in scans.iteritems():
            print "-- got scan from", frame
            #
            t1 = time.time()
            mpm.compute_importance(scan, pose)
            t2 = time.time()
            #
            print "[compute_importance took %f ms]"%(1000*(t2-t1))
            t1 = time.time()
            mpm.resample()
            t2 = time.time()
            #
            print "[resample took %f ms]"%(1000*(t2-t1))
            publish_particles()
        rate.sleep()



    
def main(argv):
    if len(argv)>1:
        func = globals()[argv[1]]
        args = argv[2:]
        func(*args)
    else:
        print "Missing argument!\nUsage: %s function args"%(argv[0])
        print "Available functions:"
        g = globals()
        l = [f for f in g if f.startswith('test')]
        l.sort()
        for f in l:
            print "  %s -- %s"%(f, g[f].__doc__)


if __name__=='__main__':
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm

    main(rospy.myargv(sys.argv))

# EOF
