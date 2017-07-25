import numpy as np
cimport numpy as np
import time

from cpython cimport bool
from cython cimport boundscheck, wraparound

from cpython.mem cimport PyMem_Malloc, PyMem_Realloc, PyMem_Free

cimport geometry.chains as chains
cimport geometry.shapes as shapes
cimport geometry.c_gjk as c_gjk
from geometry.c_gjk cimport Object_structure, gjk_distance
cimport gjk2
import gjk2

from geometry.hu cimport Violations
import geometry.transformations as transf

# from libc.math cimport sin, cos, sqrt
cdef extern from "math.h":
    double cos(double)
    double sin(double)
    double sqrt(double)

from collections import deque

cdef tuple handName = ('left', 'right')

# Don't let the arms of the robot get too close
cdef double minSelfDistance = 0.01

cdef bool debug0 = False
cdef bool debug1 = False
cdef bool debug2 = False

# The C structure for an object or link (note that this is fixed size)
cdef struct OS:
    int nOS                   # 1 (only object), 2 (object and shadow)
    double trans[4][4]        # single transform
    Object_structure obj[2]   # object and shadow
    int perm[2]               # permanent flag
    double bbox[2][2][3]      # [O/S][Lo/Hi][x/y/z]
    int name_id[2]            # index to external names

###################
# Chain Frames
##################

# The python definition for a "link frame" (also for objects)
# cdef class ChainFrameOS:                     # object and shadow
#     def __init__(self,
#                  str base=None,
#                  joint=None,            # an chains.Joint
#                  int qi=-1,
#                  np.ndarray[np.float64_t, ndim=2] frame=None,
#                  list link=None,
#                  list linkVerts=None,
#                  list bbox = None,
#                  list permanent = None):
#         self.joint = joint
#         self.base = base
#         self.qi = qi
#         self.osaIndices = []             # list of indices in OS_array
#         # The link frame
#         self.frame = frame
#         # The args below are lists, for object and shadow
#         self.link = link
#         self.linkVerts = linkVerts
#         self.bbox = bbox
#         self.permanent = permanent
#         # Radius squared for the link(s), computed
#         if linkVerts:
#             self.radius = []
#             for lv in linkVerts:
#                 if lv is None:
#                     self.radius.append(None)
#                 else:
#                     radSq = 0.0
#                     for verts in lv:
#                         for i in xrange(verts.shape[0]):
#                             radSq = max(radSq,
#                                         verts[i,0]*verts[i,0] + \
#                                         verts[i,1]*verts[i,1] + \
#                                         verts[i,2]*verts[i,2] )
#                     rad = radSq**0.5
#                     self.radius.append(rad)
#         else:
#             self.radius = None

#     def __str__(self):
#         if self.joint: name=self.joint.name
#         else: name='base:%s'%self.base
#         return 'ChainFrame(%s)'%name

@boundscheck(False)
@wraparound(False)
cpdef double gjkDist(shapes.Prim prim1, shapes.Prim prim2):
    cdef Object_structure obj1
    cdef Object_structure obj2
    cdef np.ndarray[double, ndim=2, mode="c"] bv1, bv2, tr1, tr2

    bv1 = np.ascontiguousarray(prim1.basePrim.baseVerts[:3,:].T, dtype=np.double)
    bv2 = np.ascontiguousarray(prim2.basePrim.baseVerts[:3,:].T, dtype=np.double)

    obj1.numpoints = bv1.shape[0]
    obj2.numpoints = bv2.shape[0]
    obj1.vertices = <double (*)[3]>bv1.data
    obj2.vertices = <double (*)[3]>bv2.data
    obj1.rings = NULL
    obj2.rings = NULL

    tr1 = np.ascontiguousarray(prim1.origin().matrix, dtype=np.double)
    tr2 = np.ascontiguousarray(prim2.origin().matrix, dtype=np.double)

    ans = gjk_distance(&obj1, <double (*)[4]>tr1.data, &obj2, <double (*)[4]>tr2.data,
                        NULL, NULL, NULL, 0)

    return ans

################
# C-intensive definition of confViolationsChain
################

cdef np.ndarray ztrans = np.zeros((3,4), dtype=np.double)

# Safe array of OS to pass back to Python (courtesy of Cython Memory Allocation doc)
cdef class OS_Array:

    # cdef OS *data
    # cdef size_t number

    def __cinit__(self, size_t number):
        # allocate some memory (uninitialised, may contain arbitrary data)
        self.data = <OS*> PyMem_Malloc(number * sizeof(OS))
        self.number = number
        if not self.data:
            raise MemoryError()
    def __dealloc__(self):
        PyMem_Free(self.data)     # no-op if self.data is NULL
    def set(self, int i, np.ndarray[double, ndim=2] trans,
            list verts, list bboxes, list perm, list name_id):

        if debug0:
            print '*** set', i
            print trans
            print verts
            print bboxes
            print perm
            print name_id
            print '*** set', i

        if i > self.number: return
        cdef OS *os = &self.data[i]
        cdef Object_structure *obj
        cdef np.ndarray[double, ndim=2, mode="c"] bv
        cdef int nOS = len(verts)
        cdef int j, k, l
        cdef np.ndarray[np.float64_t, ndim=2] b
        os.nOS = nOS
        if not trans is None:
            for j in range(4):
                for k in range(4):
                    os.trans[j][k] = trans[j,k]
        for j in range(nOS):
            os.perm[j] = 1 if (perm and perm[j]) else 0
            os.name_id[j] = <int>name_id[j]
            if bboxes and not bboxes[j] is None:
                bb = bboxes[j]
                for k in range(2):
                    for l in range(3):
                        os.bbox[j][k][l] = bb[k,l]
            obj = &os.obj[j]

            bv = np.ascontiguousarray(verts[j], dtype=np.double)
            assert bv.shape[0] > 3 and bv.shape[1] == 3
            obj.vertices = <double (*)[3]>bv.data
            obj.numpoints = bv.shape[0]

            for k in range(obj.numpoints):
                for l in range(3):
                    if obj.vertices[k][l] != bv[k,l]:
                        print obj.vertices[k][l], bv[k,l]
                        raw_input('Copy error?')
                    if bv[k,l] != verts[j][k,l]:
                        print bv[k,l], verts[j][k,l]
                        raw_input('numpy error?')

            obj.rings = NULL

cpdef printOSa(OS_Array osa, title, index = None):
    cdef OS *o
    print 'OSa=', title, 'number=', osa.number
    for i in range(osa.number) if index is None else [index]:
        o = &osa.data[i]
        print '***', i, '***'

        print '    nOS', o.nOS
        print '    trans', [o.trans[i][j] for i in range(4) for j in range(4)]
        # print '    obj', [o.obj[i].numpoints for i in range(o.nOS)]
        sizes = [o.obj[i].numpoints for i in range(o.nOS)]
        print '    obj', [[[o.obj[i].vertices[j][k] for k in range(3)] for j in range(sizes[i])] for i in range(o.nOS)]
        print '    perm', [o.perm[i] for i in range(o.nOS)]
        print '    bbox', [[o.bbox[k][i][j] for i in range(2) for j in range(3)] for k in range(o.nOS)]

cdef int cmethod = 0

# Returns a Violations object or None
cpdef confViolationsOS(conf, tuple CC, OS_Array robOSa, list attOSal, OS_Array obOSa,
                       list selfCollidePairs, np.ndarray np_robMove,
                       dict objShapes, list objNames, bool prdebug=False, minDist=1.0e-6,
                       method=0):
    # The response array, an entry for each object
    cdef int i, c, nobj = len(objShapes)
    cdef np.ndarray coll = np.zeros(nobj, dtype=np.dtype("i"))
    cdef int[:] collision = coll
    cdef int[:] robMove = np_robMove
    cdef int[:] stat = zzTop        # empty
    cdef int cr, chl, chr        # collision counts, can also be -1

    global cmethod
    cmethod = method
    t1 = time.clock()

    if debug1:
        prdebug = True

    if prdebug:
        print 'robMove', np_robMove

    if attOSal is None: attOSal = [None, None]
    if prdebug: print 'attOSal', attOSal

    # Make sure the names are not repeated.
    assert len(objNames) == len(set(objNames))

    # Initialize the transform data
    placeChains(conf, CC, robOSa, attOSal[0], attOSal[1], 0)
    # Check for self collision
    if not selfCollidePairs is None:
        if confSelfCollideOS(robOSa, attOSal[0], attOSal[1], selfCollidePairs, minSelfDistance, prdebug=prdebug):
            if prdebug or debug2: print 'Self collision'
            confViolTime[method] += time.clock() - t1
            return None, True    # irremediable self collision
    # Check for robot and held collisions
    cr = checkCollisionsOS(robOSa, robMove, obOSa, stat, collision, 1, prdebug, minDist)
    if cr < 0:
        if prdebug: print 'cr', cr, 'confViolationsOS =>', None
        confViolTime[method] += time.clock() - t1
        return None, False
    chl = checkCollisionsOS(attOSal[0], stat, obOSa, stat, collision, 5, prdebug, minDist)
    if chl < 0:
        if prdebug: print 'chl', chl, 'confViolationsOS =>', None
        confViolTime[method] += time.clock() - t1
        return None, False
    chr = checkCollisionsOS(attOSal[1], stat, obOSa, stat, collision, 9, prdebug, minDist)
    if chr < 0:
        if prdebug: print 'chr', chr, 'confViolationsOS =>', None
        confViolTime[method] += time.clock() - t1
        return None, False
    # Report the results
    if cr + chl + chr == 0:
        if prdebug: print 'Total violations = ', cr + chl + chr
        confViolTime[method] += time.clock() - t1
        return Violations(), False      # no collisions
    else:
        if prdebug:
            print 'collision', coll
    cdef list robObColl = []                      # rob collisions with objects
    cdef list robShColl = []                      # rob collisions with shadows
    cdef list heldObColl = [[], []]               # held object collisions
    cdef list heldShColl = [[], []]               # held shadow collisions
    # Recall that (OO, OS, SO, SS) = range(4), the entries in
    # collision are an offset (1 for robot, 5 for left held and 9 for
    # right held) + the OS index.
    for i in range(nobj):
        c = collision[i]
        o = objShapes[objNames[i]]      # look up shape
        if 1 <= c <= 4:                 # robot body collision
            if c == 1: robObColl.append(o)
            elif c == 2: robShColl.append(o)
            else: assert None, 'Illegal robot shadow collision'
        elif 5 <= c <= 8:               # held left collision
            if c <= 6: heldObColl[0].append(o)
            else: heldShColl[0].append(o)
        elif 9 <= c <= 12:               # held right collision
            if c <= 10: heldObColl[1].append(o)
            else: heldShColl[1].append(o)
    viol = Violations(robObColl, robShColl, heldObColl, heldShColl)
    if prdebug: print 'viol', viol
    confViolTime[method] += time.clock() - t1
    return viol, False

cdef int checkCollisionsOS(OS_Array moveOSa, int[:] move,
                           OS_Array statOSa, int[:] stat,
                           int[:] collision, int off,
                           bool prdebug, double minDist):
    if statOSa is None or moveOSa is None:
        return 0                        # no collisions

    cdef bool thisColl
    cdef double dist, eps
    cdef int mi, si, count
    cdef size_t nmove = moveOSa.number, nstat = statOSa.number
    cdef OS *moveOS = moveOSa.data
    cdef OS *statOS = statOSa.data
    cdef OS *statOSi
    cdef OS *moveOSi

    if debug1:
        print '*** checkCollisionsOS ***'
        print 'nmove', nmove, 'nstat', nstat
        prdebug = True

    if prdebug:
        printOSa(statOSa, 'stat')
        printOSa(moveOSa, 'move')

    minDist = max(minDist, 1.0e-6)
    eps = minDist*0.5                   # clearance for bbox
    count = 0                           # collision count
    # Check for collisions
    for stati in range(nstat):          # stationary objects
        statOSi = &statOS[stati]
        if prdebug: print 'Considering stati', stati
        if stat.shape[0] and stat[stati] == 0: continue # this object is not moving, skip
        for si in range(statOSi.nOS-1,-1,-1):    # shadow then obj
            if prdebug: print 'Checking', stati, 's/o=', si, 'alias', statOSi.name_id[si]
            if collision[statOSi.name_id[si]]: # already a collision with this static object
                if prdebug: print '...already colliding', stati, si
                continue
            thisColl = False
            for movei in range(nmove):
                if prdebug: print '    Considering move', movei
                if move.shape[0] and move[movei] == 0: continue # this object is not moving, skip
                moveOSi = &moveOS[movei]
                for mi in range(moveOSi.nOS-1,-1,-1):
                    if prdebug: print '    Checking', movei, 's/o=', mi
                    if (moveOSi.bbox[mi][0][0]-eps >= statOSi.bbox[si][1][0]+eps or \
                           moveOSi.bbox[mi][1][0]+eps <= statOSi.bbox[si][0][0]-eps or \
                           moveOSi.bbox[mi][0][1]-eps >= statOSi.bbox[si][1][1]+eps or \
                           moveOSi.bbox[mi][1][1]+eps <= statOSi.bbox[si][0][1]-eps or \
                           moveOSi.bbox[mi][0][2]-eps >= statOSi.bbox[si][1][2]+eps or \
                           moveOSi.bbox[mi][1][2]+eps <= statOSi.bbox[si][0][2]-eps):
                        if prdebug: print '    bbox non-collision'
                        continue
                    dist = gjk_distance(&moveOSi.obj[mi], <double (*)[4]>moveOSi.trans,
                                        &statOSi.obj[si], <double (*)[4]>statOSi.trans,
                                        NULL, NULL, NULL, 0)
                    if dist < minDist: # collision
                        if prdebug or debug2:
                            print '    *collision', stati, si, movei, mi, 'perm', statOSi.perm[si], moveOSi.perm[mi], 'dist', dist,'<', minDist
                            printOSa(statOSa, 'stat', stati)
                            printOSa(moveOSa, 'move', movei)
                        collision[statOSi.name_id[si]] = 2*mi+si+off
                        # stationary obst is permanent and either
                        # collision with arm or with a fixed held/grasp.
                        if statOSi.perm[si] and (off == 1 or moveOSi.perm[mi]):
                            return -1
                        thisColl = True # collision with this stationary link
                        count += 1      # collision count
                        break
                    else:
                        if prdebug: print '    no collision', stati, si, movei, mi
                    # if mover shadow does not collide with
                    # stationary part, don't check the mover obj.
                    if mi == 1 and not thisColl:
                        if prdebug: print '    mover shadow does not collide', stati, si, movei, mi
                        break
                if thisColl:            # already found a collision with this stationary part
                    if prdebug: print '...mover already colliding', movei, mi
                    break
            # if stationary shadow does not collide with any part of
            # mover, then don't bother checking the stationary object
            if si == 1 and not thisColl:
                if prdebug: print '...stationary shadow does not collide', stati, mi
                break
    if prdebug:
        print '*** checkCollisionsOS =>', count, '***'
    return count

cdef np.ndarray zzTop = np.zeros(0, dtype=np.dtype("i"))

cdef bool confSelfCollideOS(OS_Array robOSa,
                            OS_Array attOSa_l, OS_Array attOSa_r,
                            list selfCollidePairs, minDist, prdebug=False):
    if not selfCollidePairs: return False
    left_arm, left_gripper, right_arm, right_gripper, body = selfCollidePairs
    if right_arm is None or right_arm.shape[0] == 0 \
           or left_arm is None or left_arm.shape[0] == 0:
        return False      # only one hand...
    minDist = minDist*minDist  # gjk_distance returns squared distance
    cdef int[:] la_move = zzTop if left_arm is None else left_arm
    cdef int[:] lg_move = zzTop if left_gripper is None else left_gripper
    cdef int[:] ra_move = zzTop if right_arm is None else right_arm
    cdef int[:] rg_move = zzTop if right_gripper is None else right_gripper
    cdef int[:] ba_move = zzTop if body is None else body
    cdef int[:] collision = np.zeros(robOSa.number, dtype=np.dtype("i"))

    if debug1:
        print 'left_arm', left_arm
        print 'la_move.shape[0]', la_move.shape[0]
        print 'right_arm', right_arm
        print 'ra_move.shape[0]', ra_move.shape[0]
        print 'body', body
        print 'ba_move.shape[0]', ba_move.shape[0]

    # Check arms collide, held object collides with other arm or body
    return checkCollisionsOS(robOSa, la_move, robOSa, ra_move, collision, 1, False, minDist) != 0 \
           or checkCollisionsOS(robOSa, lg_move, robOSa, rg_move, collision, 1, False, minDist) != 0 \
           or checkCollisionsOS(robOSa, lg_move, robOSa, ba_move, collision, 1, False, minDist) != 0 \
           or checkCollisionsOS(robOSa, rg_move, robOSa, ba_move, collision, 1, False, minDist) != 0 \
           or checkCollisionsOS(attOSa_l, None, robOSa, ra_move, collision, 5, False, minDist) != 0 \
           or checkCollisionsOS(attOSa_l, None, robOSa, ba_move, collision, 5, False, minDist) != 0 \
           or checkCollisionsOS(attOSa_r, None, robOSa, la_move, collision, 9, False, minDist) != 0 \
           or checkCollisionsOS(attOSa_r, None, robOSa, ba_move, collision, 9, False, minDist) != 0

placeCache = [{}, {}]
placeCacheKeys = [deque([]), deque([])]   # in order of arrival
maxPlaceCacheSize = 40*10**3

cpdef clearPlaceCache():
    global placeCache, placeCacheKeys
    for i in range(len(placeCacheKeys)):
        if len(placeCache[i]) > 0:
            # print '!! Clearing place cache with', len(placeCache[i]), 'entries'
            placeCache[i] = {}
            placeCacheKeys[i] = deque([])

cdef updatePlaceCache(key, value, i):
    while len(placeCacheKeys[i]) > maxPlaceCacheSize:
        oldKey = placeCacheKeys[i].popleft()
        del(placeCache[i][oldKey])
    placeCacheKeys[i].append(key)
    placeCache[i][key] = value

placeCacheStats = [[0, 0], [0, 0]]
chainTime = 5*[0]
osaTime1 = 5*[0]
osaTime2 = 5*[0]
cacheTime = 5*[0]
confViolTime = 5*[0]

#@boundscheck(False)
#@wraparound(False)
# cdef void placeChainsOld(conf, tuple CC, OS_Array robOSa, OS_Array attOSa_l, OS_Array attOSa_r):
#     cdef list framesList, q, vals
#     cdef str frame, base, chain, name
#     cdef dict frames, attachedCCd
#     cdef np.ndarray[np.float64_t, ndim=2] baseFrame, origin, jointMatrix, bb
#     cdef np.ndarray[double, ndim=2, mode="c"] fr
#     cdef chains.Joint joint
#     cdef np.ndarray[double, ndim=1] mat
#     cdef double rad, v, qc
#     cdef int i, j, lj, k, bbl, r, c, qi
#     cdef tuple robCC
#     cdef chains.ChainFrameOS entry
#     cdef OS *frObj,
#     cdef OS_Array attOSa

#     robCC, attachedCCd = CC
#     (frames, framesList, chainNames, _) = robCC

#     placeCacheStats[0] += 1
#     if conf in placeCache:
#         t1 = time.clock()
#         placeCacheStats[1] += 1
#         vals = placeCache[conf]
#         for i, frame in enumerate(framesList):
#             frames[frame].frame = vals[i]
#         cacheTime[0] += time.clock() - t1
#     else:
#         t1 = time.clock()
#         q = []
#         vals = []
#         for name in chainNames:
#             if 'Gripper' in name:
#                 qc = conf[name][0]
#                 q.extend([qc*0.5, qc])
#             else:
#                 q.extend(conf[name])
#         for frame in framesList:
#             entry = frames[frame]
#             baseFrame = frames[entry.base].frame
#             qi = entry.qi
#             if qi >= 0:
#                 joint = entry.joint
#                 v = q[qi]
#                 jointMatrix = joint.matrix(v)
#                 origin = np.dot(baseFrame, jointMatrix)
#             elif entry.joint is not None:
#                 joint = entry.joint
#                 jointMatrix = joint.matrix()
#                 origin = np.dot(baseFrame, jointMatrix)
#             else:                           # attached objects don't have joint
#                 origin = baseFrame
#             entry.frame = origin
#             vals.append(entry.frame)
#         chainTime[0] += time.clock() - t1
#         t1 = time.clock()
#         updatePlaceCache(conf, vals)
#         cacheTime[0] += time.clock() - t1

#     # Copy the trans and bbox into the OS_Array entries
#     t1 = time.clock()
#     for frame in framesList:
#         entry = frames[frame]
#         fr = entry.frame
#         for k in entry.osaIndices:
#             frObj = &robOSa.data[k]
#             if not fr is None:
#                 frObj.trans = <double (*)[4]> fr.data
#                 # for r in range(3):
#                 #     for c in range(4):
#                 #         frObj.trans[r][c] = fr[r,c]
#             if entry.link:
#                 for j in range(len(entry.radius)):
#                     rad = entry.radius[j]
#                     for bbl in range(3):
#                         v = fr[bbl][3]
#                         frObj.bbox[j][0][bbl] = v-rad
#                         frObj.bbox[j][1][bbl] = v+rad
#     osaTime1[0] += time.clock() - t1

#     if debug0: printOSa(robOSa, 'robOSa')

#     # Update the attached chain frames
#     t1 = time.clock()
#     for hand in ('left', 'right'):
#         attCC = attachedCCd[hand]
#         if attCC is None: continue
#         attOSa = attOSa_l if hand=='left' else attOSa_r
#         if attOSa is None: continue
#         entry = attCC[0]['attached']
#         entry.frame = frames[entry.base].frame # just use the same frame
#         fr = entry.frame
#         # Copy the trans and bbox into the OS_Array entries
#         for k in entry.osaIndices:
#             frObj = &attOSa.data[k]
#             if not fr is None:
#                 frObj.trans = <double (*)[4]> fr.data
#                 # for r in range(3):
#                 #     for c in range(4):
#                 #         frObj.trans[r][c] = entry.frame[r,c]
#             if entry.link:
#                 for j in range(len(entry.radius)):
#                     rad = entry.radius[j]
#                     for bbl in range(3):
#                         v = fr[bbl][3]
#                         frObj.bbox[j][0][bbl] = v-rad
#                         frObj.bbox[j][1][bbl] = v+rad
#     osaTime2[0] += time.clock() - t1
    
#     if debug0 or debug2:
#         if attOSa_l: printOSa(attOSa_l, 'attOSa_l')
#         if attOSa_r: printOSa(attOSa_r, 'attOSa_r')

cdef np.ndarray rx = np.eye(4, dtype=np.double)
cdef np.ndarray ry = np.eye(4, dtype=np.double)
cdef np.ndarray rz = np.eye(4, dtype=np.double)
cdef np.ndarray tr = np.eye(4, dtype=np.double)

@boundscheck(False)
@wraparound(False)
cdef void placeChains(conf, tuple CC, OS_Array robOSa, OS_Array attOSa_l, OS_Array attOSa_r, int method):
    cdef list framesList, q, vals
    cdef str frame, base, chain, name
    cdef dict frames, attachedCCd, cache
    cdef np.ndarray[double, ndim=2, mode="c"] baseFrame, origin, jointMatrix, bb
    cdef np.ndarray[double, ndim=2, mode="c"] fr, transMatrix, rot, trans
    cdef chains.Joint joint
    cdef double rad, v, qc, cv, sv
    cdef int i, j, lj, k, bbl, r, c, qi
    cdef tuple robCC, axis
    cdef chains.ChainFrameOS entry
    cdef OS *frObj,
    cdef OS_Array attOSa

    robCC, attachedCCd = CC
    (frames, framesList, chainNames, _) = robCC

    placeCacheStats[cmethod][0] += 1
    cache = placeCache[cmethod]
    if conf in cache:
        t1 = time.clock()
        placeCacheStats[cmethod][1] += 1
        vals = cache[conf]
        for i, frame in enumerate(framesList):
            frames[frame].frame = vals[i]
        cacheTime[method] += time.clock() - t1
    else:
        t1 = time.clock()
        q = []
        vals = []
        for name in chainNames:
            if 'Gripper' in name:
                qc = conf[name][0]
                q.extend([qc*0.5, qc])
            else:
                q.extend(conf[name])
        for frame in framesList:
            entry = frames[frame]
            baseFrame = frames[entry.base].frame
            joint = entry.joint
            transMatrix = joint.trans.matrix
            qi = entry.qi
            if qi >= 0:
                axis = tuple(joint.axis)
                v = q[qi]
                if joint.jtype in ('revolute','continuous'):
                    cv = cos(v); sv = sin(v)
                    if axis[2] == 1.0:
                        if method==0:
                            rot = rx
                            rot[0,0] = cv; rot[0,1] = -sv
                            rot[1,0] = sv; rot[1,1] = cv
                        else:
                            rot = np.array([[cv, -sv, 0., 0.],
                                            [sv,  cv, 0., 0.],
                                            [0.,  0., 1., 0.],
                                            [0.,  0., 0., 1.]],
                                           dtype=np.double)
                    elif axis[1] == 1.0:
                        if method == 0:
                            rot = ry
                            rot[0,0] = cv; rot[0,2] = sv
                            rot[2,0] = -sv; rot[2,2] = cv
                        else:
                            rot = np.array([[cv,  0., sv, 0.],
                                            [0.,  1., 0., 0.],
                                            [-sv,  0., cv, 0.],
                                            [0.,  0., 0., 1.]],
                                           dtype=np.double)
                    elif axis[0] == 1.0:
                        if method == 0:
                            rot = rz
                            rot[1,1] = cv; rot[1,2] = -sv
                            rot[2,1] = sv; rot[2,2] = cv
                        else:
                            rot = np.array([[1.,  0., 0., 0.],
                                            [0., cv, -sv, 0.],
                                            [0., sv,  cv, 0.],
                                            [0.,  0., 0., 1.]],
                                           dtype=np.double)
                    else:
                        # raw_input('Non-orthogonal rotation axis %s'%str(axis))
                        rot = transf.rotation_matrix(v, axis)
                    jointMatrix = np.dot(transMatrix, rot)
                elif joint.jtype == 'prismatic':
                    if method == 0:
                        trans = tr
                        trans[0,3] = v*axis[0];
                        trans[1,3] = v*axis[1];
                        trans[2,3] = v*axis[2];
                    else:
                        trans = np.array([[1., 0., 0., v*axis[0]],
                                          [0., 1., 0., v*axis[1]],
                                          [0., 0., 1., v*axis[2]],
                                          [0., 0., 0., 1.]],
                                         dtype=np.double)
                    jointMatrix = np.dot(transMatrix, trans)
                else:
                    raw_input('Unknown joint type %s'%joint.jtype)
                origin = np.dot(baseFrame, jointMatrix)
            elif joint is not None:
                assert joint.jtype == 'fixed'
                origin = np.dot(baseFrame, transMatrix)
            else:                           # attached objects don't have joint
                origin = baseFrame
            entry.frame = origin
            vals.append(origin)
        chainTime[method] += time.clock() - t1
        t1 = time.clock()
        updatePlaceCache(conf, vals, cmethod)
        cacheTime[method] += time.clock() - t1

    # Copy the trans and bbox into the OS_Array entries
    t1 = time.clock()
    for frame in framesList:
        entry = frames[frame]
        fr = entry.frame
        for k in entry.osaIndices:
            frObj = &robOSa.data[k]
            if not fr is None:
                frObj.trans = <double (*)[4]> fr.data
                # for r in range(3):
                #     for c in range(4):
                #         frObj.trans[r][c] = fr[r,c]
            if entry.link:
                for j in range(len(entry.radius)):
                    rad = entry.radius[j]
                    for bbl in range(3):
                        v = fr[bbl][3]
                        frObj.bbox[j][0][bbl] = v-rad
                        frObj.bbox[j][1][bbl] = v+rad

    osaTime1[method] += time.clock() - t1
    
    if debug0: printOSa(robOSa, 'robOSa')

    # Update the attached chain frames
    t1 = time.clock()
    for hand in ('left', 'right'):
        attCC = attachedCCd[hand]
        if attCC is None: continue
        attOSa = attOSa_l if hand=='left' else attOSa_r
        if attOSa is None: continue
        entry = attCC[0]['attached']
        entry.frame = frames[entry.base].frame # just use the same frame
        fr = entry.frame
        # Copy the trans and bbox into the OS_Array entries
        for k in entry.osaIndices:
            frObj = &attOSa.data[k]
            if not fr is None:
                frObj.trans = <double (*)[4]> fr.data
                # for r in range(3):
                #     for c in range(4):
                #         frObj.trans[r][c] = entry.frame[r,c]
            if entry.link:
                for j in range(len(entry.radius)):
                    rad = entry.radius[j]
                    for bbl in range(3):
                        v = fr[bbl][3]
                        frObj.bbox[j][0][bbl] = v-rad
                        frObj.bbox[j][1][bbl] = v+rad

    osaTime2[method] += time.clock() - t1
    
    if debug0 or debug2:
        if attOSa_l: printOSa(attOSa_l, 'attOSa_l')
        if attOSa_r: printOSa(attOSa_r, 'attOSa_r')


def printPlaceTimes():
    print 'cacheTime', cacheTime
    print 'chainTime', chainTime
    print 'osaTime1', osaTime1
    print 'osaTime2', osaTime2

