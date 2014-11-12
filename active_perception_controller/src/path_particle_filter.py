#!/usr/bin/env python

print "+ sys"
import sys
import os.path
import time
import math
import random
import cPickle as pickle

print "+ scipy"
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
import matplotlib.cm as cm

print "+ dirs"
if False: #os.path.exists('/Users/yoda'):
    # Hack it for speed
    SCOUT_NAVIGATION_DIR = "/Users/yoda/monarch/trunk/rosbuild_ws/scout_navigation"
    MAPS_DIR = "/Users/yoda/monarch/trunk/rosbuild_ws/maps"
else:
    import roslib
    SCOUT_NAVIGATION_DIR = roslib.packages.get_pkg_dir('scout_navigation')
    MAPS_DIR = roslib.packages.get_pkg_dir('maps')

print "+ ros"
import rospy
import rosbag
import tf
from sensor_msgs.msg import *
from geometry_msgs.msg import *

print "+ planner"
sys.path.append( SCOUT_NAVIGATION_DIR )
import planner



DEFAULT_MAP = os.path.join(MAPS_DIR, "EPFL-v02cr_nav.yaml")
MAP_FRAME = 'map'
DEFAULT_CLEARANCE = 0.5
N_PARTICLES = 200
FACTOR = 0.1




def fix_angle(a):
    pi = math.pi
    return (a+pi)%(2*pi) - pi


class PathParticleFilter:
    points = None  # [index,coordinate]
    logprob = None  # [index]
    
    def __init__(self, cells, factor):
        n = cells.shape[1]
        total = int(math.ceil(factor*n))
        samples = [tuple(cells[:,random.randint(0,n-1)]) for i in xrange(total)]
        self.points = np.array(samples, dtype=int)
        self.logprob = np.zeros(total)
        print "path: length=%s, took %s samples"%(n,total)

    def apply_importance(self, func):
        """func maps an array of [index,coordinate] points to importances of [index]"""
        self.logprob += func(self.points)

    def resample(self):
        # get normalized probabilities
        m = len(self.logprob)
        w = np.exp(self.logprob)
        w /= w.sum()
        # determine indices of new samples
        si  = scipy.linalg.toeplitz(w, np.zeros_like(w)).sum(axis=1)
        rj  = (np.random.random() + np.arange(m)) / m
        dij = si[:,None] - rj[None,:]
        dij[dij<0] = np.inf
        k   = dij.argmin(axis=0)
        # replace particles and update weights
        self.points = self.points[k]
        self.logprob = np.zeros(len(self.points))
    
class MapParticleModel:
    filters = None
    paths = cells = None
    
    def __init__(self, mapfile, clearance=DEFAULT_CLEARANCE):
        self.pln = planner.FastMarching()
        self.pln.load_map(mapfile)
        self.pln.setup()
        maxd = clearance/self.pln.scale
        feasible = self.pln.distmap > maxd
        self.candidates = feasible.nonzero()

    def sample_points(self, total):
        """generate positions in map free space"""
        points  = []
        for n in xrange(total):
            k = random.randint(0, len(self.candidates[0])-1)
            (i, j) = (self.candidates[0][k], self.candidates[1][k])
            (x, y) = self.pln.i2p((i, j))
            points.append((x, y))
        return points

    def gen_paths_full(self, total):
        """generate paths in a fully connected graph"""
        points = self.sample_points(total)
        self.count = np.zeros(self.pln.free.shape)
        self.paths = []
        self.cells = []
        for (i,g) in enumerate(points[:-1]):
            print "-- Solving for particle %d of %d"%(i+1,len(points))
            t1 = time.time()
            self.pln.solve(g)
            for o in points[i+1:]:
                p = np.array( self.pln.get_path(o[0], o[1], step=self.pln.scale) )
                self.paths.append(p)
                (ii, jj) = self.pln.np_p2i((p[:,0], p[:,1]))
                # prune repeated cells
                matches = np.logical_and(ii[1:]==ii[:-1],
                                         jj[1:]==jj[:-1])
                unique = np.ones_like(jj, dtype=bool)
                unique[:-1] = np.logical_not(matches)
                ii = ii[unique]
                jj = jj[unique]
                self.cells.append( np.array([ii, jj], dtype=int) )
                self.count[ii,jj] += 1
            t2 = time.time()
            print "   took %f ms"%(1000*(t2-t1))

    def gen_paths_pair(self, total):
        """generate paths between pairs of samples"""
        points = self.sample_points(total)
        self.count = np.zeros(self.pln.free.shape)
        self.paths = []
        self.cells = []
        for k in xrange(0, len(points)-1, 2):
            (o, g) = points[k:k+2]
            self.pln.solve(g)
            p = np.array( self.pln.get_path(o[0], o[1], step=self.pln.scale) )
            self.paths.append(p)
            (ii, jj) = self.pln.np_p2i((p[:,0], p[:,1]))
            # prune repeated cells
            matches = np.logical_and(ii[1:]==ii[:-1],
                                     jj[1:]==jj[:-1])
            unique = np.ones_like(jj, dtype=bool)
            unique[:-1] = np.logical_not(matches)
            ii = ii[unique]
            jj = jj[unique]
            self.cells.append( np.array([ii, jj], dtype=int) )
            self.count[ii,jj] += 1

    def sample_paths(self, factor):
        """create factor*L particle for each path and create a cell->path dict"""
        assert self.cells is not None, "no paths were generated"
        # self.sampled_paths = []
        # self.cell_dict = {}
        self.filters = [ PathParticleFilter(cs, factor) for cs in self.cells ]
        # for s in samples:
        #     if s in self.cell_dict:
        #         self.cell_dict[s].append(k)
        #     else:
        #         self.cell_dict[s] = [k]
        # print "total samples:", sum([len(s) for s in self.sampled_paths])
        # print "unique cells:", len(self.cell_dict)

    def find_swept_cells(self, scan, xr, yr, tr):
        s = self.pln.scale
        z = np.array(scan.ranges)
        a = tr + np.arange(scan.angle_min, scan.angle_min+scan.angle_increment*len(z), scan.angle_increment)
        a = fix_angle(a)
        assert len(z)==len(a), "Range and angle arrays do not match"
        valid = np.isfinite(z)
        zv = z[valid]
        valid[valid] = (zv>scan.range_min) & (zv<scan.range_max)
        assert valid.any(), "No valid scan line found"
        z, a = z[valid], a[valid]
        #
        t1 = time.time()
        # determine quadrants
        q = 3*np.ones_like(a, dtype=int)
        q[ (a>=-math.pi/4)   & (a<math.pi/4)   ] = 1
        q[ (a>=math.pi/4)    & (a<3*math.pi/4) ] = 2
        q[ (a>=-3*math.pi/4) & (a<-math.pi/4)  ] = 4
        #print "q:", q
        # determine sweep
        kmax = int(math.ceil(max( np.abs(z*np.cos(a)/s).max(), np.abs(z*np.sin(a)/s).max() )))
        k = np.arange(kmax+1)
        #print "k:", k[0], "-", k[-1]
        # sweep indices of hits
        kh = np.empty_like(a, dtype=int)
        kh[q==1] =  np.around(z[q==1]*np.cos(a[q==1])/s)
        kh[q==2] =  np.around(z[q==2]*np.sin(a[q==2])/s)
        kh[q==3] = -np.around(z[q==3]*np.cos(a[q==3])/s)
        kh[q==4] = -np.around(z[q==4]*np.sin(a[q==4])/s)
        #print "kh:", kh
        # obtain [i,k] matrix where 0=unknown, 1=free, 2=hit
        # where i is the angle index and k is the sweep index
        m = np.zeros((len(a), len(k)), dtype=int)
        m[ np.arange(len(k))[None,:] < kh[:,None] ] = 1
        m[ xrange(len(a)), kh ] = 2
        print "[sweeps: %f ms]"%(1000*(time.time()-t1))
        #
        t1 = time.time()
        # determine index offsets of sweep
        j = np.empty((len(a), len(k)), dtype=int)
        i = np.empty((len(a), len(k)), dtype=int)
        # Q I
        j[q==1,:] = k[None,:]
        i[q==1,:] = -np.around(np.tan(a[q==1,None])*k[None,:])
        # Q II
        j[q==2,:] = np.around((1/np.tan(a[q==2,None]))*k[None,:])
        i[q==2,:] = -k[None,:]
        # Q III
        j[q==3,:] = -k[None,:]
        i[q==3,:] =  np.around(np.tan(a[q==3,None])*k[None,:])
        # Q IV
        j[q==4,:] = -np.around((1/np.tan(a[q==4,None]))*k[None,:])
        i[q==4,:] =  k[None,:]
        print "[sweep in index coords: %f ms]"%(1000*(time.time()-t1))
        # determine swept map indices found free
        t1 = time.time()
        (ir, jr) = self.pln.p2i((xr,yr))
        im = ir + i[m==1]
        jm = jr + j[m==1]
        # check map limits
        valid = (im>=0) & (im<self.pln.H) & (jm>=0) & (jm<self.pln.W)
        im = im[valid]
        jm = jm[valid]
        # prune repeated cells
        matches = np.logical_and(im[1:]==im[:-1],
                                 jm[1:]==jm[:-1])
        unique = np.ones_like(jm, dtype=bool)
        unique[:-1] = np.logical_not(matches)
        print "prunning repeated cells:", len(im),
        im = im[unique]
        jm = jm[unique]
        print "->", len(im)
        return (im, jm)

    def compute_importance(self, scan, xr, yr, tr):
        assert self.filters is not None, "no filters were generated yet"
        # determine swept cells
        (im, jm) = self.find_swept_cells(scan, xr, yr, tr)
        # bounding box of swept cells
        imin, imax = im.min(), im.max()
        jmin, jmax = jm.min(), jm.max()
        ww  = jmax - jmin + 1
        wh  = imax - imin + 1
        win = np.zeros((wh, ww), dtype=bool)
        win[im-imin, jm-jmax] = True
        # determine particles hit
        def importance(points):
            """assume points in [index,coordinate]"""
            n  = len(points)
            ii = points[:,0] - imin
            jj = points[:,1] - jmin
            valid = (ii>=0) & (ii<wh) & (jj>=0) & (jj<ww)
            hits = np.zeros(n, dtype=bool)
            hits[valid] = win[ ii[valid], jj[valid] ]
            # HACK for testing
            res = np.zeros(n)
            res[hits] = -1
            #print "importance: %s hits in %s points"%(hits.sum(),n)
            return res
        for p in self.filters:
            p.apply_importance(importance)
        
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
    """sampled paths generation"""
    if state is None:
        mpm = MapParticleModel(DEFAULT_MAP)
        mpm.gen_paths_pair(N_PARTICLES)
        mpm.sample_paths(0.1)
        with open("test2.state", 'w') as fh:
            pickle.dump(mpm, fh)
        print "Saved state to test2.state"
    else:
        with open(state) as fh:
            mpm = pickle.load(fh)
        #
        plt.subplot(121)
        plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r)
        #plt.subplot(121)
        # pts = np.zeros(mpm.pln.free.shape)
        # for p in mpm.filters:
        #     (i,j) = mpm.pln.p2i(p)
        #     pts[i,j]=1
        # plt.imshow(pts, cmap=cm.gray_r)
        #p = np.array(mpm.particles)
        #plt.plot(p[:,0], p[:,1], '.')
        plt.subplot(122)
        plt.imshow(mpm.count)
        #
        plt.show()



def test3():
    """gridmap laser coverage testbed"""
    with open("scan.msg") as fh:
        scan = pickle.load(fh)
    #
    # t1 = time.time()
    # # determine world coordinates of sweep
    # x = np.empty((len(a), len(k)))
    # y = np.empty((len(a), len(k)))
    # # Q I
    # x[q==1,:] = s*k[None,:]
    # y[q==1,:] = s*np.tan(a[q==1,None])*k[None,:]
    # # Q II
    # x[q==2,:] = s*(1/np.tan(a[q==2,None]))*k[None,:]
    # y[q==2,:] = s*k[None,:]
    # # Q III
    # x[q==3,:] = -s*k[None,:]
    # y[q==3,:] = -s*np.tan(a[q==3,None])*k[None,:]
    # # Q IV
    # x[q==4,:] = -s*(1/np.tan(a[q==4,None]))*k[None,:]
    # y[q==4,:] = -s*k[None,:]
    # # obtain discretized hit point cloud
    # xh = x[xrange(len(a)),kh]
    # yh = y[xrange(len(a)),kh]
    # print "[sweep in world coords: %f ms]"%(1000*(time.time()-t1))
    mpm = MapParticleModel(DEFAULT_MAP)
    (ii, jj) = mpm.find_swept_cells(scan, 13.2222172269, 28.7991394082, 0.4549502786420685)
    # make a bitmap
    img = np.zeros_like(mpm.pln.occgrid)
    img[ii, jj] = 1
    #
    print "plotting..."
    # plt.subplot(121)
    # plt.plot(xh[q==1], yh[q==1], '.r')
    # plt.plot(xh[q==2], yh[q==2], '.g')
    # plt.plot(xh[q==3], yh[q==3], '.k')
    # plt.plot(xh[q==4], yh[q==4], '.b')
    # plt.gca().axes.set_aspect('equal')
    # plt.subplot(122)
    plt.imshow(img)
    plt.show()


def test4(state=None, scan="scan.state"):
    """one cycle of path particle filter; arguments: [state [scan]]"""
    # 1. generate or load paths
    if state is None:
        mpm = MapParticleModel(DEFAULT_MAP)
        mpm.gen_paths_pair(N_PARTICLES)
        with open("test4.state", 'w') as fh:
            pickle.dump(mpm, fh)
        print "Saved state to test4.state"
    else:
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
    mpm.compute_importance(scan, pose[0], pose[1], pose[2])
    #
    plt.subplot(131)
    img = np.zeros(mpm.pln.occgrid.shape)
    for p in mpm.filters:
        img[p.points[:,0], p.points[:,1]] = 1
    plt.imshow(img, cmap=cm.gray_r)
    #
    plt.subplot(132)
    img = -np.ones(mpm.pln.occgrid.shape)
    for p in mpm.filters:
        img[p.points[:,0], p.points[:,1]] = p.logprob
    plt.imshow(img)
    #
    # 5. resample
    mpm.resample()
    #
    plt.subplot(133)
    img = np.zeros(mpm.pln.occgrid.shape)
    for p in mpm.filters:
        img[p.points[:,0], p.points[:,1]] = 1
    plt.imshow(img, cmap=cm.gray_r)
    #

    
    # 6. diffuse samples


    #
    # plt.subplot(221)
    # plt.imshow(mpm.pln.occgrid, cmap=cm.gray_r)
    #
    # plt.subplot(222)
    # img = np.zeros(mpm.pln.occgrid.shape)
    # (im, jm) = mpm.find_swept_cells(scan, pose[0], pose[1], pose[2])
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
                (position, quaternion) = tfl.lookupTransform(MAP_FRAME, frame, rospy.Time())
                pose[0] = (position[0], position[1], 2*math.atan2(quaternion[2], quaternion[3]))
                scan[0] = msg
                print "got scan from %s to %s"%(MAP_FRAME, frame)
                rospy.signal_shutdown("got scan")
            except tf.Exception:
                print "can't transform from %s to %s, still trying..."%(MAP_FRAME, frame)
    rospy.init_node('test5', anonymous=True, argv=sys.argv)
    tfl = tf.TransformListener()
    sub = rospy.Subscriber("scan", LaserScan, handler)
    rospy.spin()
    if scan[0] is not None and pose[0] is not None:
        with open(state, 'w') as fh:
            state = dict(scan=scan[0], pose=pose[0])
            pickle.dump(state, fh)





def test6(state="test4.state"):
    """closed loop experiment"""
    scans = {}
    #
    def lidar_handler(msg):
        frame = msg.header.frame_id
        try:
            (position, quaternion) = tfl.lookupTransform(MAP_FRAME, frame, rospy.Time())
            pose = (position[0], position[1], 2*math.atan2(quaternion[2], quaternion[3]))
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
    pub = rospy.Publisher("particles", PointCloud)
    rate = rospy.timer.Rate(1)
    print "Node ready -- starting loop"
    publish_particles()
    while not rospy.is_shutdown():
        for (frame,(pose,scan)) in scans.iteritems():
            print "-- got scan at", frame, "=", pose
            t1 = time.time()
            mpm.compute_importance(scan, pose[0], pose[1], pose[2])
            t2 = time.time()
            print "[compute_importance took %f ms]"%(1000*(t2-t1))
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
    main(rospy.myargv(sys.argv))

# EOF
