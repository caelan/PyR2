import math
import numpy as np
cimport numpy as np
import random
import operator
from cpython cimport bool
import geometry.transf as transf
cimport geometry.transf as transf

# Poses are 4D (x, y, z, th)

# Types for transform matrices, for points and point matrices
# Transform is a 4x4 matrix
# Point is a column vector, (4x1)
# Quaternion is a row vector, 1x4

#################################
# Transform
#################################

cdef class Transform:
    """
    Rotation and translation represented as 4 x 4 matrix

    Three ways to specify:
    - m is a 4 x 4 matrix
    - p is a displacement and q is a quaternion

    """
    def __init__(self,
                 np.ndarray[np.float64_t, ndim=2] m = None,
                 np.ndarray[np.float64_t, ndim=2] p = None,
                 np.ndarray[np.float64_t, ndim=1] q = None):
        assert (not m is None) or ((not q is None) and (not p is None)), 'Invalid args to Transform'
        self.eqDistEps = 0.001                 # for equality
        self.eqAngleEps = 0.01                 # for equality
        self.matrix = m
        self.matrixInv = None
        self.reprString = None
        if (not q is None) and (not p is None):
            self.q = Quat(q)
            self.pt = Point(p)
            if m is None:
                self.matrix = np.dot(transf.translation_matrix(p.T[0]),
                                     transf.quaternion_matrix(q))
        else:
            self.q = None
            self.pt = None
        self.matrix.flags.writeable = False # so we can hash it

    cpdef Transform inverse(self):
        """
        Returns transformation matrix that is the inverse of this one
        """
        if self.matrixInv is None:
            # self.matrixInv =  np.linalg.inv(self.matrix)
            self.matrixInv = np.eye(4)
            self.matrixInv[:3,:3] = self.matrix[:3,:3].T
            self.matrixInv[:3,3] = -np.dot(self.matrixInv[:3,:3], self.matrix[:3,3])
        return Transform(self.matrixInv)

    cpdef Transform invertCompose(self, Transform trans):
        """
        Returns transformation matrix that is the inverse of this one composed with trans
        """
        if self.matrixInv is None:
            self.matrixInv = np.eye(4)
            self.matrixInv[:3,:3] = self.matrix[:3,:3].T
            self.matrixInv[:3,3] = -np.dot(self.matrixInv[:3,:3], self.matrix[:3,3])
        return Transform(np.dot(self.matrixInv, trans.matrix))
    
    def __neg__(self):
        return self.inverse()

    cpdef Transform compose(self, Transform trans):
        """
        Returns composition of self and trans
        """
        if trans == None:
            raw_input('Whoa ... null matrix')
        cdef Transform tr = Transform(np.dot(self.matrix, trans.matrix))
        return tr

    def __mul__(self, other):
        return self.compose(other)

    cpdef Pose pose(self, double zthr = 0.01, bool fail = True):
        """
        Convert to Pose
        """
        cdef double theta
        if abs(1.0 - self.matrix[2,2]) < zthr:
            theta = math.atan2(self.matrix[1,0], self.matrix[0,0])
            return Pose(self.matrix[0,3], self.matrix[1,3], self.matrix[2,3], theta)
        elif fail:
            print self.matrix
            raise Exception, "Not a valid 2.5D Pose"
        else:
            return None

    cpdef Point point(self):
        if self.pt is None:
            self.pt = Point(np.vstack([self.matrix[:3, 3:4], np.array([1.], dtype=np.float64)]))
        return self.pt

    cpdef Quat quat(self):
        if self.q is None:
            self.q = Quat(transf.quaternion_from_matrix(self.matrix))
        return self.q

    cpdef Point applyToPoint(self, Point point):
        """
        Transform a point into a new point.
        """
        return Point(np.dot(self.matrix, point.matrix))

    cpdef bool near(self, Transform trans, double distEps, double angleEps):
        """
        Return True if point of transform is within distEps of self
        and the quaternion distance is with angleEps.
        """
        # Check the distance between the centers
        if not self.point().isNear(trans.point(), distEps):
            return False
        # Check the angle between the quaternions
        dot = np.dot(trans.quat().matrix, self.quat().matrix)
        dot = max(-1.0, min(1.0, dot))
        if 2*abs(math.acos(abs(dot))) > angleEps:
            return False
        return True
    
    cpdef bool withinDelta(self, Transform trans, tuple delta, pr=False):
        """
        Return True if [x,y,z] of transform is within delta[:3] of self
        and the quaternion distance is with delta[3]
        """
        for i in range(3):
            if abs(self.matrix[i,3] - trans.matrix[i,3]) > delta[i]:
                if pr:
                    print 'xyz'[i], abs(self.matrix[i,3] - trans.matrix[i,3]), '>', delta[i]
                return False
        # The angle between the quaternions
        q1 = trans.quat().matrix
        q2 = self.quat().matrix
        if all(q1 == q2):
            return True
        dot = max(-1.0, min(1.0, np.dot(q1, q2)))
        if (1-dot) <= 1.0e-8:
            return True
        if 2*abs(math.acos(abs(dot))) > delta[3]:
            if pr:
                print 'theta', 2*abs(math.acos(abs(dot))), '>', delta[3]
            return False
        return True

    cpdef double distance(self, Transform tr):
        """
        Return the distance between the x,y,z part of self and the x,y,z
        part of pose.
        """
        return self.point().distance(tr.point())

    def __call__(self, Point point):
        return self.applyToPoint(point)

    def __repr__(self):
        if not self.reprString:
            self.reprString = 'Transform('+str(self.matrix.tolist())+')'
        return self.reprString
    def __str__(self):
        return repr(self)
    def __copy__(self):
        return Transform(self.matrix.copy())
    def __richcmp__(self, other, int op):
        if not (other and isinstance(other, Transform)):
            return True if op == 3 else False
        ans = self.near(other, self.eqDistEps, self.eqAngleEps) if op == 2 else False
        return ans
    def __hash__(self):
        return hash(self.matrix.data)
    def __deepcopy__(self):
        return self.__copy__(self)
    def copy(self):
        return self.__copy__(self)

#################################
# Pose
#################################

cdef class Pose(Transform):             # 2.5D transform
    """
    Represent the x,y,z,theta pose of an object in 2.5D space
    """
    def __init__(self, double x, double y, double z, double theta):
        self.x = x
        """x coordinate"""
        self.y = y
        """y coordinate"""
        self.z = z
        """z coordinate"""
        # self.theta = fixAngle02Pi(theta)
        self.theta = theta
        """rotation in radians"""
        self.initTrans()

    cpdef initTrans(self):
        cdef double cosTh
        cdef double sinTh
        cosTh = math.cos(self.theta)
        sinTh = math.sin(self.theta)
        self.reprString = None
        Transform.__init__(self, np.array([[cosTh, -sinTh, 0.0, self.x],
                                           [sinTh, cosTh, 0.0, self.y],
                                           [0.0, 0.0, 1.0, self.z],
                                           [0, 0, 0, 1]], dtype=np.float64))

    cpdef setX(self, double x):
        raw_input('Modifying Pose... not a good idea')
        self.x = x
        self.initTrans()

    cpdef setY(self, double y):
        raw_input('Modifying Pose... not a good idea')
        self.y = y
        self.initTrans()

    cpdef setZ(self, double z):
        raw_input('Modifying Pose... not a good idea')
        self.z = z
        self.initTrans()

    cpdef setTheta(self, double theta):
        raw_input('Modifying Pose... not a good idea')
        self.theta = theta
        self.initTrans()

    cpdef Pose average(self, Pose other, double alpha):
        """
        Weighted average of this pose and other
        """
        return Pose(alpha * self.x + (1 - alpha) * other.x,
                    alpha * self.y + (1 - alpha) * other.y,
                    alpha * self.z + (1 - alpha) * other.z,
                    angleAverage(self.theta, other.theta, alpha))

    cpdef Point point(self):
        """
        Return just the x, y, z parts represented as a C{Point}
        """
        return Point(np.array([[self.x], [self.y], [self.z], [1.0]], dtype=np.float64))

    cpdef Pose diff(self, Pose pose):
        """
        Return a pose that is the difference between self and pose (in
        x, y, z, and theta)
        """
        return Pose(self.x-pose.x,
                    self.y-pose.y,
                    self.z-pose.z,
                    fixAnglePlusMinusPi(self.theta-pose.theta))

    cpdef double totalDist(self, Pose pose, double angleScale = 1):
        return self.distance(pose) + \
               abs(fixAnglePlusMinusPi(self.theta-pose.theta)) * angleScale

    cpdef Pose inversePose(self):
        """
        Return a transformation matrix that is the inverse of the
        transform associated with this pose.
        """
        return super(Pose, self).inverse().pose()

    cpdef tuple xyztTuple(self):
        """
        Representation of pose as a tuple of values
        """
        return (self.x, self.y, self.z, self.theta)


    cpdef tuple xytTuple(self):
        """
        Ignore z
        """
        return (self.x, self.y, self.theta)

    cpdef Pose corrupt(self, double e, double eAng = 0.0, bool noZ = False):
        """
        Corrupt with a uniformly distributed Pose.
        """
        eAng = eAng or e
        return Pose(np.random.uniform(-e, e),
                    np.random.uniform(-e, e),
                    0.0 if noZ else np.random.uniform(-e, e),
                    np.random.uniform(-eAng, eAng)).compose(self)

    cpdef Pose corruptGauss(self, double mu, tuple stdDev, bool noZ = False):
        """
        Corrupt with a Gaussian distributed Pose.
        """
        perturbation = Pose(random.gauss(mu, stdDev[0]),
                            random.gauss(mu, stdDev[1]),
                            0.0 if noZ else random.gauss(mu, stdDev[2]),
                            random.gauss(mu, stdDev[3]))
        return self.compose(perturbation).pose()

    def __copy__(self):
        return Pose(self.x, self.y, self.z, self.theta)
    def __deepcopy__(self):
        return Pose(self.x, self.y, self.z, self.theta)
    def __str__(self):
        return 'Pose[' + prettyString(self.x) + ', ' +\
               prettyString(self.y) + ', ' +\
               prettyString(self.z) + ', ' +\
               (prettyString(self.theta) \
                if self.theta <= 6.283 else prettyString(0.0))\
                + ']'
    def __repr__(self):
        if not self.reprString:
            self.reprString = 'Pose(' + \
                              repr(self.x) + ',' + \
                              repr(self.y) + ',' + \
                              repr(self.z) + ',' + \
                              repr(self.theta) + ')'
        return self.reprString

# 3 poses (x, y, theta)
class Pose3(Pose):
    def __init__(self, x, y, theta):
        Pose.__init__(self, x, y, 0, theta)

#################################
# Point
#################################

cdef class Point:
    """
    Represent a point with its x, y, z values
    """
    def __init__(self, np.ndarray[np.float64_t, ndim=2] p): # column matrix
        self.eqDistEps = 0.001                 # for equality
        self.matrix = p
        self.matrix.flags.writeable = False # so we can hash it

    cpdef bool isNear(self, Point point, double distEps):
        """
        Return true if the distance between self and point is less
        than distEps
        """
        return self.distance(point) < distEps

    cpdef double distance(self, Point point):
        """
        Euclidean distance between two points
        """
        return np.linalg.norm((self.matrix - point.matrix)[:3])

    cpdef double distanceXY(self, Point point):
        """
        Euclidean distance between two XY points
        """
        return np.linalg.norm((self.matrix - point.matrix)[:2])

    cpdef double distanceSq(self, Point point):
        """
        Euclidean distance (squared) between two points
        """
        cdef np.ndarray[np.float64_t, ndim=2] delta
        delta = (self.matrix - point.matrix)[:3]
        return np.dot(delta.T, delta)

    cpdef double distanceSqXY(self, Point point):
        """
        Euclidean distance (squared) between two XY points
        """
        cdef np.ndarray[np.float64_t, ndim=2] delta
        delta = (self.matrix - point.matrix)[:2]
        return np.dot(delta.T, delta)

    cpdef double magnitude(self):
        """
        Magnitude of this point, interpreted as a vector in 3-space
        """
        return np.linalg.norm(self.matrix[:3])

    cpdef tuple xyzTuple(self):
        """
        Return tuple of x, y, z values
        """
        return tuple(self.matrix.reshape((4,))[:3])

    cpdef Pose pose(self, double angle = 0.0): #Pose
        """
        Return a pose with the position of the point.
        """
        return Pose(self.matrix[0,0], self.matrix[1,0], self.matrix[2,0], angle)

    cpdef Point point(self):
        """
        Return a point, that is, self.
        """
        return self

    def __str__(self):
        w = self.matrix[3]
        if w == 1:
            return 'Point'+ prettyString(self.xyzTuple())
        if w == 0:
            return 'Delta'+ prettyString(self.xyzTuple())
        else:
            return 'PointW'+ prettyString(tuple(self.matrix))

    def __repr__(self):
        return 'hu.Point(np.array(' + str(self.matrix) + '))'

    cpdef double angleToXY(self, Point p):
        """
        Return angle in radians of vector from self to p (in the xy projection)
        """
        cdef np.ndarray[np.float64_t, ndim=2] delta
        delta = p.matrix - self.matrix
        return math.atan2(delta[1,0], delta[0,0])

    cpdef Point add(self, Point point):
        """
        Vector addition
        """
        cdef np.ndarray[np.float64_t, ndim=2] summ
        summ = self.matrix + point.matrix
        summ[3,0] = 1.0
        return Point(summ)
    def __add__(self, point):
        return self.add(point)

    cpdef Point sub(self, Point point):
        """
        Vector subtraction
        """
        cdef np.ndarray[np.float64_t, ndim=2] diff
        diff = self.matrix - point.matrix
        diff[3,0] = 1.0
        return Point(diff)
    def __sub__(self, point):
        return self.sub(point)
    cpdef Point scale(self, double s):
        """
        Vector scaling
        """
        cdef np.ndarray[np.float64_t, ndim=2] sc
        sc = self.matrix * s
        sc[3,0] = 1.0
        return Point(sc)
    def __mul__(self, s):
        return self.scale(s)
    cpdef double dot(self, Point p):
        """
        Dot product
        """
        return np.dot(self.matrix[:3].T, p.matrix[:3])

    def __richcmp__(self, other, int op):
        if not (other and isinstance(other, Point)):
            return True if op == 3 else False
        ans = self.isNear(other, self.eqDistEps) if op == 2 else False
        return ans
    def __hash__(self):
        return hash(self.matrix.data)
    def __copy__(self):
        cp = Point(self.matrix)
        cp.minAngle = self.minAngle
        cp.topAngle = self.topAngle
        return cp
    __deepcopy__ = __copy__
    copy = __copy__

#################################
# Quat
#################################

cdef class Quat:
    def __init__(self, np.ndarray[np.float64_t, ndim=1] quat):
        self.matrix = quat

######################################################################
# Miscellaneous utilities  - are they used?
######################################################################
    
cpdef list smash(list lists):
    return [item for sublist in lists for item in sublist]

cpdef bool within(double v1, double v2, double eps):
    """
    Return True if v1 is with eps of v2. All params are numbers
    """
    return abs(v1 - v2) < eps

cpdef bool nearAngle(double a1, double a2, double eps):
    """
    Return True if angle a1 is within epsilon of angle a2  Don't use
    within for this, because angles wrap around!
    """
    return abs(fixAnglePlusMinusPi(a1-a2)) < eps

cpdef bool nearlyEqual(double x, double y):
    """
    Like within, but with the tolerance built in
    """
    return abs(x-y)<.0001

cpdef double fixAnglePlusMinusPi(double a):
    """
    A is an angle in radians;  return an equivalent angle between plus
    and minus pi
    """
    cdef double pi2
    cdef int i
    pi2 = 2.0* math.pi
    i = 0
    while abs(a) > math.pi:
        if a > math.pi:
            a = a - pi2
        elif a < -math.pi:
            a = a + pi2
        i += 1
        if i > 100: 
            raw_input('fixAnglePlusMinusPi called with crazy value', a)
            break                # loop found
    return a

cpdef fixAngle02Pi(a):
    """
    A is an angle in radians;  return an equivalent angle between 0
    and 2 pi
    """
    cdef double pi2 = 2.0* math.pi
    cdef int i = 0
    while a < 0 or a > pi2:
        if a < 0:
            a = a + pi2
        elif a > pi2:
            a = a - pi2
        i += 1
        if i > 100: 
            raw_input('fixAngle02Pi called with crazy value', a)
            break                # loop found
    return a

cpdef argmax(list l, f):
    """
    @param l: C{List} of items
    @param f: C{Procedure} that maps an item into a numeric score
    @returns: the element of C{l} that has the highest score
    """
    cdef list vals = [f(x) for x in l]
    return l[vals.index(max(vals))]

cpdef tuple argmaxWithVal(list l, f):
    """
    @param l: C{List} of items
    @param f: C{Procedure} that maps an item into a numeric score
    @returns: the element of C{l} that has the highest score and the score
    """
    best = l[0]
    cdef:
        double bestScore = f(best)
        double xScore
    for x in l:
        xScore = f(x)
        if xScore > bestScore:
            best, bestScore = x, xScore
    return (best, bestScore)

cpdef tuple argmaxIndex(list l, f): #edit - f = lambda x: x
    """
    @param l: C{List} of items
    @param f: C{Procedure} that maps an item into a numeric score
    @returns: the index of C{l} that has the highest score
    """
    cdef:
        int i
        int best = 0
        double bestScore = f(l[best])
        double xScore
    for i from 0 <= i < len(l):
        xScore = f(l[i])
        if xScore > bestScore:
            best, bestScore = i, xScore
    return (best, bestScore)

cpdef tuple argmaxIndexWithTies(list l, f): #edit - f = lambda x: x
    """
    @param l: C{List} of items
    @param f: C{Procedure} that maps an item into a numeric score
    @returns: the index of C{l} that has the highest score
    """
    cdef:
        list best = []
        double bestScore = f(l[0])
        double xScore
        int i
    for i from 0 <= i < len(l):
        xScore = f(l[i])
        if xScore > bestScore:
            best, bestScore = [i], xScore
        elif xScore == bestScore:
            best, bestScore = best + [i], xScore
    return (best, bestScore)

cpdef double clip(double v, vMin, vMax):
    """
    @param v: number
    @param vMin: number (may be None, if no limit)
    @param vMax: number greater than C{vMin} (may be None, if no limit)
    @returns: If C{vMin <= v <= vMax}, then return C{v}; if C{v <
    vMin} return C{vMin}; else return C{vMax}
    """
    if vMin == None:
        if vMax == None:
            return v
        else:
            return min(v, vMax)
    else:
        if vMax == None:
            return max(v, vMin)
        else:
            return max(min(v, vMax), vMin)

cpdef int sign(double x):
    """
    Return 1, 0, or -1 depending on the sign of x
    """
    if x > 0.0:
        return 1
    elif x == 0.0:
        return 0
    else:
        return -1

cpdef str prettyString(struct):
    """
    Make nicer looking strings for printing, mostly by truncating
    floats
    """
    if type(struct) == list:
        return '[' + ', '.join([prettyString(item) for item in struct]) + ']'
    elif type(struct) == tuple:
        return '(' + ', '.join([prettyString(item) for item in struct]) + ')'
    elif type(struct) == dict:
        return '{' + ', '.join([str(item) + ':' +  prettyString(struct[item]) \
                                             for item in struct]) + '}'
    elif type(struct) == float or type(struct) == np.float64:
        struct = round(struct, 3)
        if struct == 0: struct = 0      #  catch stupid -0.0
        return "%5.3f" % struct
    else:
        return str(struct)

cdef class SymbolGenerator:
    """
    Generate new symbols guaranteed to be different from one another
    Optionally, supply a prefix for mnemonic purposes
    Call gensym("foo") to get a symbol like 'foo37'
    """
    def __init__(self): 
        self.counts = {}
    cpdef str gensym(self, str prefix = 'i'):
        count = self.counts.get(prefix, 0)
        self.counts[prefix] = count + 1
        return prefix + '_' + str(count)

gensym = SymbolGenerator().gensym
"""Call this function to get a new symbol"""

cpdef double logGaussian(double x, double mu, double sigma):
    """
    Log of the value of the gaussian distribution with mean mu and
    stdev sigma at value x
    """
    return -((x-mu)**2 / (2*sigma**2)) - math.log(sigma*math.sqrt(2*math.pi))

cpdef double gaussian(double x, double mu, double sigma):
    """
    Value of the gaussian distribution with mean mu and
    stdev sigma at value x
    """
    return math.exp(-((x-mu)**2 / (2*sigma**2))) /(sigma*math.sqrt(2*math.pi))

cpdef list lineIndices(tuple one, tuple two):
    (i0, j0) = one
    (i1, j1) = two
    """
    Takes two cells in the grid (each described by a pair of integer
    indices), and returns a list of the cells in the grid that are on the
    line segment between the cells.
    """
    cdef:
        list ans = [(i0,j0)]
        int di = i1 - i0
        int dj = j1 - j0
        double t = 0.5
        double m
    if abs(di) > abs(dj):               # slope < 1
        m = float(dj) / float(di)       # compute slope
        t += j0
        if di < 0: di = -1
        else: di = 1
        m *= di
        while (i0 != i1):
            i0 += di
            t += m
            ans.append((i0, int(t)))
    else:
        if dj != 0:                     # slope >= 1
            m = float(di) / float(dj)   # compute slope
            t += i0
            if dj < 0: dj = -1
            else: dj = 1
            m *= dj
            while j0 != j1:
                j0 += dj
                t += m
                ans.append((int(t), j0))
    return ans

cpdef double angleDiff(double x, double y):
    cdef:
        double twoPi = 2*math.pi
        double z = (x - y)%twoPi
    if z > math.pi:
        return z - twoPi
    else:
        return z

cpdef bool inRange(v, tuple r):
    return r[0] <= v <= r[1]

cpdef bool rangeOverlap(tuple r1, tuple r2):
    return r2[0] <= r1[1] and r1[0] <= r2[1]

cpdef tuple rangeIntersect(tuple r1, tuple r2):
    return (max(r1[0], r2[0]), min(r1[1], r2[1]))

cpdef double average(list stuff):
    return sum(stuff) * (1.0 / float(len(stuff)))

cpdef tuple tuplify(x):
    if type(x) in (tuple, list):
        return tuple([tuplify(y) for y in x])
    else:
        return x

cpdef list squash(list listOfLists):
    return reduce(operator.add, listOfLists)

# Average two angles
cpdef double angleAverage(double th1, double th2, double alpha):
    return math.atan2(alpha * math.sin(th1) + (1 - alpha) * math.sin(th2),
                      alpha * math.cos(th1) + (1 - alpha) * math.cos(th2))

cpdef list floatRange(double lo, double hi, double stepsize):
    """
    @returns: a list of numbers, starting with C{lo}, and increasing
    by C{stepsize} each time, until C{hi} is equaled or exceeded.

    C{lo} must be less than C{hi}; C{stepsize} must be greater than 0.
    """
    if stepsize == 0:
       print 'Stepsize is 0 in floatRange'
    cdef list result = []
    cdef double v = lo
    while v <= hi:
        result.append(v)
        v += stepsize
    return result

cpdef pop(x):
    if isinstance(x, list):
        if len(x) > 0:
            return x.pop(0)
        else:
            return None
    else:
        try:
            return x.next()
        except StopIteration:
            return None

cdef class Hash(object):
    def __init__(self):
        self.hashValue = None
        self.descValue = None
    def desc(self):
        raise NotImplementedError('Abstract class')
    def __str__(self):
        return self.__class__.__name__+str(self.desc())
    def __repr__(self):
        return self.__class__.__name__+str(self.desc())
    def __hash__(self):
        if self.hashValue is None:
            self.hashValue = self.desc().__hash__()
        return self.hashValue
    def __richcmp__(self, other, int op):
        if op == 2:
            return isinstance(other, Hash) and self.desc() == other.desc()
        elif op == 3:
            return not (isinstance(other, Hash) and self.desc() == other.desc())
        else:
            return False

# Copied from mmUtil.py
def shadowp(obj):
    if isinstance(obj, str):
        return obj[-7:] == '_shadow'
    else:
        return obj.name()[-7:] == '_shadow'

# represent the "removable" collisions wth obstacles and shadows.
# This has to be hashable so use tuples and frozensets
cdef class Violations(Hash):
    def __init__(self, obstacles = None, shadows = None,
                 heldObstacles=None, heldShadows=None):
        obst = obstacles[:] if obstacles else []
        sh = shadows[:] if shadows else []
        if not heldObstacles: heldObstacles = ([], [])
        if not heldShadows: heldShadows = ([], [])
        self.obstacles = frozenset(obst)
        # Collisions with only shadows, remove collisions with objects as well
        self.shadows = frozenset([o for o in sh if not o in self.obstacles])
        ao = self.obstacles.union(self.shadows)
        ho = ([],[])
        if heldObstacles:
            for h in (0,1):
                for o in heldObstacles[h]:
                    if o not in ao: ho[h].append(o)
        hs = ([],[])
        if heldShadows:
            for h in (0,1):
                for o in heldShadows[h]:
                    if o not in ao: hs[h].append(o)
        self.heldObstacles = tuple([frozenset(ho[h]) for h in (0,1)])
        # Collisions only with heldShadow, remove collisions with heldObject as well
        self.heldShadows = tuple([frozenset([o for o in hs[h] \
                                             if not o in self.heldObstacles[h]]) \
                                  for h in (0,1)])
        Hash.__init__(self)
    cpdef list allObstacles(self):
        obst = list(self.obstacles)
        for h in (0,1):
            for o in self.heldObstacles[h].union(self.heldShadows[h]):
                if not shadowp(o): obst.append(o)
        return obst
    cpdef list allShadows(self):
        shad = list(self.shadows)
        for h in (0,1):
            for o in self.heldObstacles[h].union(self.heldShadows[h]):
                if shadowp(o): shad.append(o)
        return shad
    def empty(self):
        return (not self.obstacles) and (not self.shadows) \
               and (not any(x for x in self.heldObstacles)) \
               and (not any(x for x in self.heldShadows))
    cpdef Violations combine(self, obstacles, shadows, heldObstacles=None, heldShadows=None):
        return self.update(Violations(obstacles, shadows, heldObstacles, heldShadows))
    cpdef Violations update(self, viol):
            return Violations(upd(self.obstacles, viol.obstacles),
                              upd(self.shadows, viol.shadows),
                              (upd(self.heldObstacles[0], viol.heldObstacles[0]),
                               upd(self.heldObstacles[1], viol.heldObstacles[1])),
                              (upd(self.heldShadows[0], viol.heldShadows[0]),
                               upd(self.heldShadows[1], viol.heldShadows[1])))
    cpdef double weight(self, weights=(1.0, 0.5, 1.0, 0.5)):
        return weights[0]*len(self.obstacles) + \
               weights[1]*len(self.shadows) + \
               weights[2]*sum([len(ho) for ho in self.heldObstacles]) +\
               weights[3]*sum([len(hs) for hs in self.heldShadows])
    cpdef bool LEQ(self, other):
        return self.weight() <= other.weight()
    cpdef tuple desc(self):
        return (self.obstacles, self.shadows, self.heldObstacles, self.heldShadows)
    cpdef tuple names(self):
        return (frozenset([x.name() for x in self.obstacles]),
                frozenset([x.name() for x in self.shadows]),
                tuple([frozenset([x.name() for x in ho]) for ho in self.heldObstacles]),
                tuple([frozenset([x.name() for x in hs]) for hs in self.heldShadows]))
    def __repr__(self):
        return 'Violations%s'%str(([x.name() for x in self.obstacles],
                                   [x.name() for x in self.shadows],
                                   [[x.name() for x in ho] for ho in self.heldObstacles],
                                   [[x.name() for x in hs] for hs in self.heldShadows]))
    def draw(self, window, color='magenta'):
        for x in (self.obstacles, self.shadows, self.heldObstacles, self.heldShadows):
            # noinspection PyUnresolvedReferences
            x.draw(window, color)
    def __str__(self):
        return self.__repr__()

cpdef list upd(curShapes, newShapes):
    curDict = dict([(o.name(), o) for o in curShapes])
    newDict = dict([(o.name(), o) for o in newShapes])
    curDict.update(newDict)
    return curDict.values()

