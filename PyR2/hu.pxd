from cpython cimport bool
import numpy as np
cimport numpy as np

cdef class Transform(object):
    cdef public double eqDistEps, eqAngleEps
    cdef public np.ndarray matrix
    cdef np.ndarray matrixInv
    cdef Quat q 
    cdef Point pt
    cdef str reprString

    cpdef Transform inverse(self)
    cpdef Transform invertCompose(self, Transform trans)
    cpdef Transform compose(self, Transform trans)
    cpdef Pose pose(self, double zthr = *, bool fail = *)
    cpdef Point point(self)
    cpdef Quat quat(self)
    cpdef Point applyToPoint(self, Point point)
    cpdef bool near(self, Transform trans, double distEps, double angleEps)
    cpdef bool withinDelta(self, Transform trans, tuple delta, pr=*)
    cpdef double distance(self, Transform tr)

cdef class Pose(Transform):
    cdef public double x,y,z,theta

    cpdef initTrans(self)
    cpdef setX(self, double x)
    cpdef setY(self, double y)
    cpdef setZ(self, double z)
    cpdef setTheta(self, double theta)
    cpdef Pose average(self, Pose other, double alpha)
    cpdef Point point(self)
    cpdef Pose diff(self, Pose pose)
    cpdef double totalDist(self, Pose pose, double angleScale = *)
    cpdef Pose inversePose(self)
    cpdef tuple xyztTuple(self)
    cpdef tuple xytTuple(self)
    cpdef Pose corrupt(self, double e, double eAng = *, bool noZ = *)
    cpdef Pose corruptGauss(self, double mu, tuple stdDev, bool noZ = *)

cdef class Point:
    cdef public double eqDistEps
    cdef public np.ndarray matrix

    cpdef bool isNear(self, Point point, double distEps)
    cpdef double distance(self, Point point)
    cpdef double distanceXY(self, Point point)
    cpdef double distanceSq(self, Point point)
    cpdef double distanceSqXY(self, Point point)
    cpdef double magnitude(self)
    cpdef tuple xyzTuple(self)
    cpdef Pose pose(self, double angle = *)
    cpdef Point point(self)
    cpdef double angleToXY(self, Point p)
    cpdef Point add(self, Point point)
    cpdef Point sub(self, Point point)
    cpdef Point scale(self, double s)
    cpdef double dot(self, Point p)

cdef class Quat:
    cdef public np.ndarray matrix

cdef class Hash(object):
    cdef public hashValue, descValue

cdef class Violations(Hash):
    cdef public frozenset obstacles, shadows
    cdef public tuple heldObstacles, heldShadows

    cpdef list allObstacles(self)
    cpdef list allShadows(self)
    cpdef Violations combine(self, obstacles, shadows, heldObstacles=*, heldShadows=*)
    cpdef Violations update(self, viol)
    cpdef double weight(self, weights=*)
    cpdef bool LEQ(self, other)
    cpdef tuple desc(self)
    cpdef tuple names(self)

cpdef list upd(curShapes, newShapes)

################################################################################

cdef class SymbolGenerator:
     cdef  dict counts
     cpdef str gensym(self, str prefix = *)

cpdef list smash(list lists)
cpdef bool within(double v1, double v2, double eps)
cpdef bool nearAngle(double a1, double a2, double eps)
cpdef bool nearlyEqual(double x, double y)
cpdef double fixAnglePlusMinusPi(double a)
cpdef fixAngle02Pi(a)
cpdef argmax(list l, f)
cpdef tuple argmaxWithVal(list l, f)
cpdef tuple argmaxIndex(list l, f)
cpdef tuple argmaxIndexWithTies(list l, f)
cpdef double clip(double v, vMin, vMax)
cpdef int sign(double x)
cpdef str prettyString(struct)
cpdef double logGaussian(double x, double mu, double sigma)
cpdef double gaussian(double x, double mu, double sigma)
cpdef list lineIndices(tuple one, tuple two)
cpdef double angleDiff(double x, double y)
cpdef bool inRange(v, tuple r)
cpdef bool rangeOverlap(tuple r1, tuple r2)
cpdef tuple rangeIntersect(tuple r1, tuple r2)
cpdef double average(list stuff)
cpdef tuple tuplify(x)
cpdef list squash(list listOfLists)
cpdef double angleAverage(double th1, double th2, double alpha)
cpdef list floatRange(double lo, double hi, double stepsize)
cpdef pop(x)


