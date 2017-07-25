import geometry.hu as hu
import math
import numpy
cimport numpy

cimport ik2
import ik2

from autil.utils import prettyString
from geometry.transformations import rotation_matrix

# The nominal tool offset in ikFast
gripperTip = hu.Pose(0.18,0.0,0.0,0.0)
gripperToolOffset = numpy.dot(gripperTip.matrix,
                           rotation_matrix(math.pi/2,(0,1,0)))

cdef dict cache = {}
def clearInvKinCache2():
    cache.clear()

invKinStats2 = [0, 0]
cdef double IK_FAST_STEP = 0.1

def armInvKin2(chains, arm, torso, target, conf, 
              returnAll = False):
    name = 'ArmInvKin'

    assert not returnAll                # Just for one solution !!

    # The tool pose relative to the torso frame 
    newHandPoseRel = reduce(numpy.dot, [torso.inverse().matrix,
                                        target.matrix,
                                        gripperToolOffset])
    invKinStats2[0] += 1
    newHandPoseRel.flags.writeable = False
    armName = 'pr2LeftArm' if arm=='l' else 'pr2RightArm'
    current = conf[armName]
    key = (arm, newHandPoseRel.data, current[2])
    if cache.get(key, None) is not None:
        invKinStats2[1] += 1
        return cache[key]
    newArmAngles = pr2KinIKfast(arm, newHandPoseRel, current,
                                chain=chains.chainsByName[armName])

    cache[key] = newArmAngles
    return newArmAngles

nsolnsKin = 10
cdef numpy.ndarray rotKin, transKin, freeKin, solnsKin, jtKin
rotKin = numpy.ascontiguousarray(numpy.zeros(9, dtype=numpy.double), dtype=numpy.double)
transKin = numpy.ascontiguousarray(numpy.zeros(3, dtype=numpy.double), dtype=numpy.double)
freeKin = numpy.ascontiguousarray(numpy.zeros(1, dtype=numpy.double), dtype=numpy.double)
solnsKin = numpy.ascontiguousarray(numpy.zeros(7*nsolnsKin, dtype=numpy.double), dtype=numpy.double)
jtKin = numpy.ascontiguousarray(numpy.zeros(7, dtype=numpy.double), dtype=numpy.double)

cdef inline double angleDiff(double x, double y):
    cdef:
        double twoPi = 2*math.pi
        double z = (x - y)%twoPi
    if z > math.pi:
        return z - twoPi
    else:
        return z

cdef inline double solnDist(sol1, sol2):
    cdef:
        double maxDiff = 0.0
        double diff = 0.0
        int i
    for i in range(7):
        diff = abs(angleDiff(sol1[i], sol2[i]))
        if diff > maxDiff:
            maxDiff = diff
    return maxDiff

# cdef inline double solnDist(sol1, sol2):
#     return max([abs(hu.angleDiff(th1, th2)) for (th1, th2) in zip(sol1, sol2)])

cdef list pr2KinIKfast(arm, T, current, chain):
    cdef double bestDist = float('inf')
    cdef double dist
    cdef set sols
    sols = pr2KinIKfastAll(arm, T, current, chain)
    if not sols: return []
    bestSol = None
    for s in sols:
        dist = solnDist(current, s)
        if dist < bestDist:
            bestDist = dist
            bestSol = s
    return list(bestSol)

cdef collectSols(int n, chain, set sols):
    cdef int i
    for i in range(n):
        sol = solnsKin[i*7 : (i+1)*7]
        if sol is not None:
            sol_list = sol.tolist()
            if chain.valid(sol_list):  # inside joint limits
                sols.add(tuple(sol_list))

cdef set pr2KinIKfastAll(arm, T, current, chain):
    cdef int i, j, nsols, n
    cdef double upper, lower, th0, stepSize, step
    cdef set sols

    nsols = nsolnsKin
    for i in range(3):
        for j in range(3):
            rotKin[i*3+j] = T[i, j]
    for i in range(3):
        transKin[i] = T[i, 3]
    if arm=='r':
        # bug in right arm kinematics, this is the distance between the shoulders
        transKin[1] += 0.376
    step = IK_FAST_STEP
    lower, upper = chain.limits()[2]
    th0 = current[2]
    if not lower <= th0 <= upper: return set([])
    nsteps = int(max((upper-th0)/step, (th0-lower)/step))
    sols = set([])
    for i in range(nsteps):
        stepsize = i*step
        freeKin[0] = th0 + stepsize
        if freeKin[0] <= upper:
            if arm == 'l':
                n = ikLeft(<const double *>transKin.data, <const double *>rotKin.data, <const double *>freeKin.data, <const int>nsols, <double *>solnsKin.data)
            else:
                n = ikRight(<const double *>transKin.data, <const double *>rotKin.data, <const double *>freeKin.data, <const int>nsols, <double *>solnsKin.data)
            collectSols(n, chain, sols)
            if len(sols) >= 2: return sols
        freeKin[0] = th0 - stepsize
        if freeKin[0] >= lower:
            if arm == 'l':
                n = ikLeft(<const double *>transKin.data, <const double *>rotKin.data, <const double *>freeKin.data, <const int>nsols, <double *>solnsKin.data)
            else:
                n = ikRight(<const double *>transKin.data, <const double *>rotKin.data, <const double *>freeKin.data, <const int>nsols, <double *>solnsKin.data)
            collectSols(n, chain, sols)
            if len(sols) >= 2: return sols

    return sols
