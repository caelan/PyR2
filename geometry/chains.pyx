cimport geometry.hu as hu
import geometry.hu as hu
import geometry.transformations as transf
import copy
import math
import random
import numpy as np
cimport numpy as np
from cpython cimport bool
import xml.etree.ElementTree as ET
from collections import OrderedDict

from geometry.hu cimport Transform, angleDiff, fixAnglePlusMinusPi
from geometry.shapes cimport Shape
from geometry.geom import bboxUnion, bboxOrigin, vertsBBox
from geometry.gjk2 import confViolationsOS, OS_Array, printOSa

cdef double PI2 = 2*math.pi
cdef Transform Ident = Transform(np.eye(4))

cpdef linkVerts(link, rel=False, prdebug=False):
    cdef np.ndarray[np.float64_t, ndim=2] off, vmat
    cdef list verts = []
    # print 'link origin\n', link.origin().matrix
    for prim in link.toPrims():
        vmat = prim.basePrim.baseVerts
        if rel:
            # Attached objects are already relative to link frame
            off = prim.origin().matrix
            verts.append(np.ascontiguousarray(np.dot(off, vmat)[:3,:].T,
                                              dtype=np.double))
        else:
            off = link.origin().inverse().compose(prim.origin()).matrix
            verts.append(np.ascontiguousarray(np.dot(off, vmat)[:3,:].T,
                                              dtype=np.double))
    if prdebug:
        print 'linkVerts'
        print verts
    return verts

# Compiles a Multi-Chain, like PR2
# frames is a dictionary of {frameName : ChainFrame instance}
# framesList is a sequential list of frameNames
# allChains is OrderedDict {chainName : [frameName, frameName,...]}
# frameChain is dictionary {frameName : chainName}, the chain it belongs to
cpdef compileChainFramesOS(robot):
    allChains = OrderedDict()
    frameChain = {}
    frames = {'root' : ChainFrameOS(frame=Ident.matrix)}
    framesList = []
    qi = 0
    for chain in robot.chains.chainsInOrder:
        base = chain.baseFname
        assert len(chain.joints) == len(chain.links), 'Links do not match joints'
        chainFrames = []
        for joint, link in zip(chain.joints, chain.links):
            framesList.append(joint.name)
            chainFrames.append(joint.name)
            frameChain[joint.name] = chain.name # reverse index
            if isinstance(joint, Rigid):
                index = -1
            else:
                index = qi
                qi += 1
            if link and link.parts():
                frames[joint.name] = ChainFrameOS(base=base, joint=joint, qi=index,
                                                  link=[link],
                                                  linkVerts=[linkVerts(link)],
                                                  bbox=[maxBBox])
            else:
                frames[joint.name] = ChainFrameOS(base=base, joint=joint, qi=index)
            base = joint.name
        allChains[chain.name] = chainFrames
    return frames, framesList, allChains, frameChain

inf = float('inf')
maxBBox = np.array(((-inf,-inf,-inf),(inf,inf,inf)))


# A multi-chain object is basically a dictionary that maps part names
# into Chain instances.

cdef class MultiChain:
    def __init__(self, name, chains):
        self.type = 'MultiChain'
        self.chainsInOrder = sorted(chains, chainCmp)
        self.chainsByName = dict([(c.name, c) for c in chains])
        self.fnames = [j for chain in chains for j in chain.fnames]
        nonLocalBaseFnames = [chain.baseFname for chain in chains \
                              if not chain.baseFname in self.fnames]
        # There should only be one "free" base fname among the chains.
        assert len(nonLocalBaseFnames) == 1, 'Only one free base allowed'
        self.baseFname = nonLocalBaseFnames[0]
        self.name = name

    cpdef placement(self, base, conf, getShapes = True):
        """Returns a shape object given a dictionary of name:jointValues"""
        cfg = conf.copy()
        chainShapes = []
        frames = {self.baseFname : base}
        # pr = True if (getShapes and getShapes != True) else False
        for chain in self.chainsInOrder:
            if getShapes is True:
                chainShape = getShapes
            else:
                chainShape = chain.name in getShapes
            pl = chain.placement(frames[chain.baseFname], cfg[chain.name],
                                 getShapes=chainShape)
            if pl is None:
                return (None, frames)
            (sha, trs) = pl
            if sha:
                chainShapes.append(sha)
            for joint, tr in zip(chain.joints, trs):
                frames[joint.name] = tr
        return Shape(chainShapes, None, name = self.name) if getShapes else None, frames

    cpdef valid(self, conf):
        for chain in conf.conf:
            if not self.chainsByName[chain].valid(conf[chain]):
                return False
        return True

    def __str__(self):
        return 'MultiChain:%s'%self.name
    __repr__ = __str__

cpdef int chainCmp(c1, c2):
    if c1.baseFname in c2.fnames:       # c1 needs to be later: c1 > c2
        return 1
    elif c2.baseFname in c1.fnames:     # c1 needs to be earlier: c1 < c2
        return -1
    return 0

############################################################
# Chains
############################################################

# A Chain is a list of links, a list of joints and a specified base frame name.

# Note that a regular "movable object" has a single link and a single
# (general) joint, parameterized by a Transform.  Some examples:
# tr = prismatic joint; rev = revolute joint; gen = general joint; fix = fixed joint
# movable: <base> <gen> <link>
# permanent: <base> <fix> <link>
# planar: <base> <tr: x> <empty> <tr: y> <empty> <rev: theta> <link>
# XYZT (cup or pan): <base> <tr: x> <empty> <tr: y> <empty> <tr: z> <empty> <rev: theta> <link>
# door: <base> <fix> <empty> <rev: theta> <link>

cdef class Chain:
    def __init__(self, name, baseFname, joints, links):
        self.name = name
        self.baseFname = baseFname
        self.joints = joints
        self.links = links
        self.fnames = [j.name for j in joints]
        self.chainsInOrder = [self]
        self.chainsByName = {name : self}
        self.jointLimits = None
        self.movingJoints = [joint for joint in self.joints\
                             if not isinstance(joint, Rigid)]
    # jointValues is a list of individual joint values
    cpdef frameTransforms(self, base, jointValues):
        """Returns all the frames (for each joint) given jointValues."""
        j = 0
        frames = [base]                 # list of Transforms
        for joint in self.joints:
            if isinstance(joint, Rigid):
                frames.append(frames[-1].compose(joint.transform()))
            else:
                val = jointValues[j]; j += 1
                if joint.valid(val):
                    frames.append(frames[-1].compose(joint.transform(val)))
                else:
                    return None
        assert j == len(jointValues), 'Inconsistent joint values'    # we used them all
        return frames[1:]               # not the base

    cpdef limits(self):
        "Returns the limits for the relevant joints."
        if not self.jointLimits:
            self.jointLimits = [joint.limits for joint in self.movingJoints]
        return self.jointLimits

    cpdef randomValues(self):
        return [lo + random.random()*(hi-lo) for (lo, hi) in self.limits()]

    cpdef bool valid(self, list jointValues):
        cdef double jv
        cdef Joint joint
        cdef list moving = self.movingJoints
        assert len(jointValues) == len(moving), 'Inconsistent joints in valid'
        for i in range(len(jointValues)):
            joint = moving[i]
            jv = jointValues[i]
            if not joint.valid(jv):
               return False
        return True

    cpdef placement(self, base, jointValues, getShapes=True):
        """Returns shape object for chain at the given jointValues and a list of
        the frames for each link."""
        trs = self.frameTransforms(base, jointValues)
        if trs:
            if getShapes:
                parts = []
                for p, tr in zip(self.links, trs):
                    if p:
                        parts.append(p.applyLoc(tr))
                shape = Shape(parts,
                              None,   # origin
                              name = self.name)
                return shape, trs
            else:
                return None, trs
        else:
            print 'Placement failed for', self, jointValues

    cpdef forwardKin(self, base, jointValues):
        """Returns the targetValue for given jointValues or None if illegal."""
        tr = self.frameTransforms(base, jointValues)
        if tr: return tr[-1]

    # targetValue is a Transform
    cpdef targetPlacement(self, base, targetValue):
        """Returns shape object for chain that places last joint frame."""
        ik = self.inverseKin(base, targetValue)
        if ik is None: return None
        return self.placement(base, ik)

    cpdef stepAlongLine(self, jvf, jvi, stepSize):
        assert len(jvf) == len(jvi), 'Inconsistent joints in stepAlongLine'
        assert self.valid(jvf), 'Invalid joint value in stepAlongLine'
        indices = range(len(jvi))
        diffs = [self.movingJoints[i].diff(jvf[i], jvi[i]) for i in indices]
        length = sum([d**2 for d in diffs])**0.5
        if length == 0. or stepSize/length >= 1.: return jvf
        vals = [diffs[i]*(stepSize/length) + jvi[i] for i in indices]
        return vals

    cpdef interpolate(self, jvf, jvi, ratio, stepSize):
        assert len(jvf) == len(jvi), 'Inconsistent joints in interpolate'
        assert self.valid(jvf), 'Invalid joint value in interpolate'
        indices = range(len(jvi))
        diffs = [self.movingJoints[i].diff(jvf[i], jvi[i]) for i in indices]
        length = sum([d**2 for d in diffs])**0.5
        vals = [diffs[i]*ratio + jvi[i] for i in indices]
        return vals, length <= stepSize

    cpdef distAbs(self, jvf, jvi):
        assert len(jvf) == len(jvi), 'Inconsistent joints in distAbs'
        cdef double d = 0.0
        cdef double diff
        cdef int i
        for i in range(len(jvi)):
            diff = self.movingJoints[i].diff(jvf[i], jvi[i])
            if diff < 0: d += -diff
            else: d += diff
        return d

    cpdef distAbsLess(self, jvf, jvi, dmax):
        assert len(jvf) == len(jvi), 'Inconsistent joints in distAbsLess'
        cdef double d = 0.0
        cdef double diff
        cdef int i
        for i in range(len(jvi)):
            diff = self.movingJoints[i].diff(jvf[i], jvi[i])
            if diff < 0: d += -diff
            else: d += diff
            if d > dmax: return None
        return d


    cpdef dist(self, jvf, jvi):
        assert len(jvf) == len(jvi), 'Inconsistent joints in dist'
        diffs = [self.movingJoints[i].diff(jvf[i], jvi[i]) for i in xrange(len(jvi))]
        return (sum([d*d for d in diffs]))**0.5

    cpdef normalize(self, jvf, jvi):
        assert len(jvf) == len(jvi), 'Inconsistent joints in normalize'
        indices = range(len(jvi))
        diffs = [self.movingJoints[i].diff(jvf[i], jvi[i]) for i in indices]
        new = [jvi[i] + diffs[i] for i in indices]
        jvf = list(jvf)
        if not self.valid(new):
            # print 'Attempt to normalize failed - produced invalid new values'
            # print '    jvi', jvi
            # print '    jvf', jvf
            # print '    new', new
            return jvf
        else:
            return new

    cpdef inverseKin(self, base, targetValue):
        """Returns valid jointValues for given targetValue or None if illegal.
        There is no systematic process for inverseKin for arbitrary chains.
        This has to be provided by the subclass."""
        return None

    def __str__(self):
        return 'Chain:%s'%self.name
    __repr__ = __str__

cdef class Movable(Chain):
    def __init__(self, name, baseFname, shape):
        Chain.__init__(self, name, baseFname,
                       [General(name, None, None, None)],
                       [shape])

    cpdef inverseKin(self, base, tr):
        ibase = base.compose(self.joints[0].trans)
        return ibase.inverse().compose(tr)

cdef class Planar(Chain):
    def __init__(self, name, baseFname, shape, bbox):
        Chain.__init__(self, name, baseFname,
                       [Prismatic(name+'_x', Ident,
                                  (bbox[0,0], bbox[1,0]), (1.,0.,0.)),
                        Prismatic(name+'_y', Ident,
                                  (bbox[0,1], bbox[1,1]), (0.,1.,0.)),
                        Revolute(name+'_theta', Ident,
                                 (-math.pi, math.pi), (0.,0.,1.))],
                        [Shape([], None),
                         Shape([], None), shape])

    cpdef inverseKin(self, base, tr):
        ibase = base.compose(self.joints[0].trans)
        tr = ibase.inverse().compose(tr)
        # Is it a (nearly) planar transform? Rot about z, no z offset
        pose = tr.pose(fail=False)
        if pose and abs(tr.matrix[2,3]) < 0.001:
            params = [pose.x, pose.y, pose.theta]
            for (j, x) in zip(self.joints, params):
                if not j.valid(x): return None
            return params

cdef class XYZT(Chain):
    def __init__(self, name, baseFname, shape, bbox):
        Chain.__init__(self, name, baseFname,
                       [Prismatic(name+'_x', Ident,
                                  (bbox[0,0], bbox[1,0]), (1.,0.,0.)),
                        Prismatic(name+'_y', Ident,
                                  (bbox[0,1], bbox[1,1]), (0.,1.,0.)),
                        Prismatic(name+'_z', Ident,
                                  (bbox[0,2], bbox[1,2]), (0.,0.,1.)),
                        Revolute(name+'_theta', Ident,
                                 (-math.pi, math.pi), (0.,0.,1.))],
                        [Shape([], None),
                         Shape([], None),
                         Shape([], None), shape])

    cpdef inverseKin(self, base, tr):
        ibase = base.compose(self.joints[0].trans)
        tr = ibase.inverse().compose(tr)
        # Is it a pose transform? Rot about z
        pose = tr.pose(fail=False)
        if pose and abs(tr.matrix[2,3]) < 0.001:
            params = [pose.x, pose.y, pose.z, pose.theta]
            for (j, x) in zip(self.joints, params):
                if not j.valid(x): return None
            return params

cdef class Permanent(Chain):
    def __init__(self, name, baseFname, shape, locTr):
        Chain.__init__(self, name, baseFname,
                       [Rigid(name+'_loc', locTr, None, None)],
                       [shape])

    cpdef inverseKin(self, base, target):
        return []                       # no motion

cdef class RevoluteDoor(Chain):
    def __init__(self, name, baseFname, shape, locTr, angles):
        Chain.__init__(self, name, baseFname,
                       [Rigid(name, locTr, None, None),
                        Revolute(name+'_theta', Ident, angles, (0.,0.,1.))],
                       [Shape([], None), shape])

    cpdef inverseKin(self, base, tr):
        ibase = base.compose(self.joints[0].trans)
        tr = ibase.inverse().compose(tr)
        # Is it a pure rotation about z transform?
        pose = tr.pose(fail=False)
        if pose and all([abs(tr.matrix[i,3]) < 0.001 for i in range(3)]):
            if self.joints[1].valid(pose.theta):
                return [None, pose.theta]

# This is a weird one..
cdef class GripperChain(Chain):
    cpdef frameTransforms(self, base, jointValues):
        width = jointValues[-1]
        return Chain.frameTransforms(self, base, [0.5*width, width])
    cpdef limits(self):
        return Chain.limits(self)[1:]
    cpdef bool valid(self, list jointValues):
        width = jointValues[-1]
        return Chain.valid(self, [0.5*width, width])
    cpdef placement(self, base, jointValues, getShapes=True):
        width = jointValues[-1]
        return Chain.placement(self, base, [0.5*width, width], getShapes=getShapes)
    cpdef stepAlongLine(self, jvf, jvi, stepSize):
        assert len(jvf) == len(jvi) == 1, 'Inconsistent joints in stepAlongLine'
        assert self.valid(jvf), 'Invalid joint in stepAlongLine'
        diff = jvf[-1] - jvi[-1]
        length = diff*diff
        if length == 0. or stepSize/length >= 1.: return jvf
        return [diff*(stepSize/length) + jvi[-1]]
    cpdef interpolate(self, jvf, jvi, ratio, stepSize):
        assert len(jvf) == len(jvi) == 1, 'Inconsistent joints in interpolate'
        assert self.valid(jvf), 'Invalid joint in interpolate'
        diff = jvf[-1] - jvi[-1]
        length = (diff*diff)**0.5
        return [diff*ratio + jvi[-1]], length <= stepSize
    cpdef dist(self, jvf, jvi):
        assert len(jvf) == len(jvi) == 1, 'Inconsistent joints in dist'
        diff = jvf[-1] - jvi[-1]
        return math.sqrt(diff*diff)
    cpdef normalize(self, jvf, jvi):
        return jvf
    cpdef forwardKin(self, base, jointValues):
        width = jointValues[-1]
        return Chain.forwardKin(self, base, [0.5*width, width])
    cpdef targetPlacement(self, base, targetValue):
        raise Exception, 'Not implemented'
    cpdef inverseKin(self, base, targetValue):
        raise Exception, 'Not implemented'

############################################################
# Joints
############################################################

cdef class Joint:
    subClasses = {}
    def __init__(self, name, trans, limits, axis):
        self.name = name
        self.trans = trans
        self.limits = limits
        self.axis = axis
        self.normalized = None
        self.cache = {}

cdef class Prismatic(Joint):
    def __init__(self, name, trans, limits, axis):
        self.jtype = 'prismatic'
        Joint.__init__(self, name, trans, limits, axis)
    cpdef np.ndarray matrix(self, val):
        cdef double v = val
        cdef list vec = [q*v for q in self.axis]
        cdef np.ndarray tr = np.array([[1., 0., 0., vec[0]],
                                       [0., 1., 0., vec[1]],
                                       [0., 0., 1., vec[2]],
                                       [0., 0., 0., 1.]],
                                      dtype=np.float64)
        ans = np.dot(self.trans.matrix, tr)
        return ans
    cpdef Transform transform(self, val):
        return Transform(self.matrix(val))
    cpdef bool valid(self, double val):
        cdef double lo, hi
        (lo, hi) = self.limits
        if lo-0.0001 <= val <= hi+0.0001:
            return True
        return False
    cpdef diff(self, a, b):
        return a - b
    def __repr__(self):
        return 'Joint:(%s, %s)'%('Prismatic', self.name)
    __str__ = __repr__
Joint.subClasses['prismatic'] = Prismatic

cdef class Revolute(Joint):
    def __init__(self, name, trans, limits, axis):
        self.jtype = 'revolute'
        self.cache = {}
        Joint.__init__(self, name, trans, limits, axis)
    cpdef np.ndarray matrix (self, val):
        cdef double v = val
        cv = math.cos(v); sv = math.sin(v)
        if self.axis[2] == 1.0:
            rot = np.array([[cv, -sv, 0., 0.],
                            [sv,  cv, 0., 0.],
                            [0.,  0., 1., 0.],
                            [0.,  0., 0., 1.]],
                           dtype=np.float64)
        elif self.axis[1] == 1.0:
            rot = np.array([[cv,  0., sv, 0.],
                            [0.,  1., 0., 0.],
                            [-sv,  0., cv, 0.],
                            [0.,  0., 0., 1.]],
                           dtype=np.float64)
        elif self.axis[0] == 1.0:
            rot = np.array([[1.,  0., 0., 0.],
                            [0., cv, -sv, 0.],
                            [0., sv,  cv, 0.],
                            [0.,  0., 0., 1.]],
                           dtype=np.float64)
        else:
            rot = transf.rotation_matrix(val, self.axis)
        ans = np.dot(self.trans.matrix, rot)
        return ans

    cpdef Transform transform(self, val):
        return Transform(self.matrix(val))

    cpdef bool valid(self, double val):
        cdef double lo, hi, vw
        if val in self.cache: return self.cache[val]
        if not self.normalized:
            self.normalized = normalizedAngleLimits(self.limits)
        for (lo, hi) in self.normalized:
            if val > math.pi or val < -math.pi:
                vw = (val+math.pi)%PI2
                if vw < 0: vw += PI2
                vw = vw - math.pi
            else:
                vw = val
            # vw = fixAnglePlusMinusPi(val)
            # if abs(vw1 - vw) > 0.0001:
            #    print val, 'vw1', vw1, 'vw', vw
            if (lo-0.001 <= vw <= hi+0.001):
                self.cache[val] = True
                return True
        self.cache[val] = False
        return False
    cpdef diff(self, a, b):
        if self.limits[0] == -math.pi and self.limits[1] == math.pi:
            z = (a - b)%PI2
            if z > math.pi:
                return z - PI2
            else:
                return z
        else:
            return a - b
    def __repr__(self):
        return 'Joint:(%s, %s)'%('Revolute', self.name)
    __str__ = __repr__
Joint.subClasses['revolute'] = Revolute
Joint.subClasses['continuous'] = Revolute

cdef list normalizedAngleLimits(tuple limits):
    (lo, hi) = limits
    if lo >= -math.pi and hi <= math.pi:
        return [limits]
    elif lo < -math.pi and hi <= math.pi:
        return [(-math.pi, hi), (fixAnglePlusMinusPi(lo), math.pi)]
    elif lo >= -math.pi and hi > math.pi:
        return [(-math.pi, fixAnglePlusMinusPi(hi)), (lo, math.pi)]
    else:
        raise Exception, 'Bad angle range'

cdef class General(Joint):
    def __init__(self, name, trans, limits, axis):
        self.jtype = 'general'
        Joint.__init__(self, name, trans, limits, axis)
    cpdef np.ndarray matrix(self, val):
        return val.matrix
    cpdef Transform transform(self, val):
        return Transform(val.matrix)
    cpdef bool valid(self, val):
        return True
    cpdef diff(self, a, b):
        return a.inverse().compose(b)
    def __repr__(self):
        return 'Joint:(%s, %s)'%('General', self.name)
    __str__ = __repr__
Joint.subClasses['general'] = General

cdef class Rigid(Joint):
    def __init__(self, name, trans, limits, axis):
        self.jtype = 'fixed'
        Joint.__init__(self, name, trans, limits, axis)
    cpdef np.ndarray matrix(self, val=None):
        return self.trans.matrix
    cpdef Transform transform(self, val=None):
        return self.trans
    cpdef bool valid(self, val=None):
        return True
    cpdef diff(self, a, b):
        raise Exception, 'Rigid joint does not have diff'
    def __repr__(self):
        return 'Joint:(%s, %s)'%('Rigid', self.name)
    __str__ = __repr__
Joint.subClasses['fixed'] = Rigid

def getUrdfJoints(jointNames,
                  file = "pr2.urdf"):
    robotTree = ET.parse(file)
    joints = [j for j in robotTree.findall('joint') if not j.find('origin') is None]
    result = []
    for name in jointNames:
        for j in joints:
            if j.attrib['name'] == name:
                result.append(jointFromUrdf(j))
    return result

cdef vec(str):
    return [float(x) for x in str.split()]

def jointFromUrdf(joint):
    name = joint.attrib['name']
    jtype = joint.attrib['type']
    origin = joint.find('origin')
    trn = transf.translation_matrix(vec(origin.attrib['xyz']))
    rot = transf.euler_matrix(*vec(origin.attrib.get('rpy', "0 0 0")), axes='sxyz')
    safety = joint.find('safety_controller')
    limit = joint.find('limit')
    axis = joint.find('axis')
    if not axis is None:
        axis = vec(axis.attrib['xyz'])
    if jtype == 'continuous':
        limits = (-math.pi, math.pi)
    elif jtype == 'fixed':
        limits = None
    else:
        if safety is not None:
            assert safety.attrib['soft_lower_limit'] is not None, 'Invalid joint in URDF:'+str((name, jtype, safety))
            limits = (float(safety.attrib['soft_lower_limit']),
                      float(safety.attrib['soft_upper_limit']))
        elif limit is not None:
            assert limit.attrib['lower'] is not None, 'Invalid joint in URDF:'+str((name, jtype, limit))
            limits = (float(limit.attrib['lower']),
                      float(limit.attrib['upper']))
        else:
            assert None, 'No limits for joint in URDF:'+str((name, jtype))
    return Joint.subClasses[jtype](name, Transform(np.dot(trn, rot)), tuple(limits) if limits else None, tuple(axis) if axis else None)

###################
# Chain Frames
##################

# The python definition for a "link frame" (also for objects)
cdef class ChainFrameOS:                     # object and shadow
    def __init__(self,
                 str base=None,
                 joint=None,            # an objects.Joint
                 int qi=-1,
                 np.ndarray[np.float64_t, ndim=2] frame=None,
                 list link=None,
                 list linkVerts=None,
                 list bbox = None,
                 list permanent = None):
        self.joint = joint
        self.base = base
        self.qi = qi
        self.osaIndices = []             # list of indices in OS_array
        # The link frame
        self.frame = frame
        # The args below are lists, for object and shadow
        self.link = link
        self.linkVerts = linkVerts
        self.bbox = bbox
        self.permanent = permanent
        # Radius squared for the link(s), computed
        if linkVerts:
            self.radius = []
            for lv in linkVerts:
                if lv is None:
                    self.radius.append(None)
                else:
                    radSq = 0.0
                    for verts in lv:
                        for i in xrange(verts.shape[0]):
                            radSq = max(radSq,
                                        verts[i,0]*verts[i,0] + \
                                        verts[i,1]*verts[i,1] + \
                                        verts[i,2]*verts[i,2] )
                    rad = radSq**0.5
                    self.radius.append(rad)
        else:
            self.radius = None

    def __str__(self):
        if self.joint: name=self.joint.name
        else: name='base:%s'%self.base
        return 'ChainFrame(%s)'%name
