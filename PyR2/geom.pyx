import numpy as np
cimport numpy as np
from cpython cimport bool
cimport PyR2.shapes as shapes
import PyR2.shapes as shapes
cimport PyR2.hu as hu

#################################################################################
# BBox operations -- uses Numpy arrays
#################################################################################

# verts is a 4xn array
cpdef np.ndarray vertsBBox(np.ndarray[np.float64_t, ndim=2] verts,
                           np.ndarray[np.int_t, ndim=1] indices):
    cdef np.ndarray[np.float64_t, ndim=2] selected
    if indices is None:
        # Do min and max along the points dimension
        return np.array([np.min(verts, axis=1)[:3], np.max(verts, axis=1)[:3]])
    else:
        selected = verts[:, indices]    # pick out some of the points
        return np.array([np.min(selected, axis=1)[:3], np.max(selected, axis=1)[:3]])

cpdef np.ndarray bboxCenter(np.ndarray[np.float64_t, ndim=2] bb,
                            bool base = False):
    cdef np.ndarray[np.float64_t, ndim=1] c
    c =  0.5*(bb[0] + bb[1])
    if base: c[2] = bb[0,2]             # bottom z value
    return c

cpdef np.ndarray bboxDims(np.ndarray[np.float64_t, ndim=2] bb):
    return bb[1] - bb[0]

cpdef np.ndarray bboxUnion(list bboxes):
    minVals = 3*[float('inf')]
    maxVals = 3*[-float('inf')]
    for bb in bboxes:
        for i in range(3):
            minVals[i] = min(minVals[i], bb[0][i])
            maxVals[i] = max(maxVals[i], bb[1][i])
    return np.array([minVals, maxVals])

cpdef np.ndarray bboxIsect(list bboxes):
    return np.array([np.max(np.vstack([bb[0] for bb in bboxes]), axis=0),
                     np.min(np.vstack([bb[1] for bb in bboxes]), axis=0)])

cpdef bool bboxOverlap(np.ndarray[np.float64_t, ndim=2] bb1,
                       np.ndarray[np.float64_t, ndim=2] bb2):
    # Touching is not overlap
    # return not (np.any(bb1[0] >= bb2[1]) or np.any(bb1[1] <= bb2[0]))
    # Due to a Cython bug... cannot convert numpy.bool_ to bool
    return False if \
           bb1[0][0] >= bb2[1][0] or bb1[1][0] <= bb2[0][0] or \
           bb1[0][1] >= bb2[1][1] or bb1[1][1] <= bb2[0][1] or \
           bb1[0][2] >= bb2[1][2] or bb1[1][2] <= bb2[0][2] else True

cpdef bool bboxIn(np.ndarray[np.float64_t, ndim=2] bb,
                  np.ndarray[np.float64_t, ndim=1] p):
    # Due to a Cython bug... cannot convert numpy.bool_ to bool
    return True if np.all(p >= bb[0]) and np.all(p <= bb[1]) else False

cpdef bool bboxContains(np.ndarray[np.float64_t, ndim=2] bb,
                        np.ndarray[np.float64_t, ndim=1] pt):
    return bboxIn(bb, pt[:3])

# bb1 inside bb2?
cpdef bool bboxInside(np.ndarray[np.float64_t, ndim=2] bb1,
                      np.ndarray[np.float64_t, ndim=2] bb2):
    return bboxIn(bb2, bb1[0]) and bboxIn(bb2, bb1[1])

cpdef double bboxVolume(np.ndarray[np.float64_t, ndim=2] bb):
    return np.prod(bb[1]-bb[0])

cpdef np.ndarray bboxGrow(np.ndarray[np.float64_t, ndim=2] bb,
                          np.ndarray[np.float64_t, ndim=1] off):
    return np.array([bb[0]-off, bb[1]+off])

# Produces xy bbox, by setting z values to 0
cpdef np.ndarray bboxZproject(np.ndarray[np.float64_t, ndim=2] bb):
    cdef np.ndarray[np.float64_t, ndim=2] bbc
    bbc = bb.copy()
    bbc[0,2] = bbc[1,2] = 0.
    return bbc

cpdef np.ndarray bboxMinkowskiXY(np.ndarray[np.float64_t, ndim=2] bb1,
                                 np.ndarray[np.float64_t, ndim=2] bb2):
    cdef np.ndarray[np.float64_t, ndim=2] b
    b = bboxGrow(bb2, 0.5*bboxDims(bb1))
    b[0,2] = min(bb1[0,2], bb2[0,2])
    b[1,2] = max(bb1[1,2], bb2[1,2])
    return b

# The bbox referenced to its base center
cpdef np.ndarray bboxRefXY(np.ndarray[np.float64_t, ndim=2] bb):
    return bb - bboxCenter(bboxZproject(bb))

cpdef np.ndarray bboxOrigin(np.ndarray[np.float64_t, ndim=2] bb):
    trans = np.eye(4, dtype=np.float64)
    trans[:3, 3] = bboxCenter(bb)[:3]
    return trans

cpdef bool bboxGrownOverlap(np.ndarray[np.float64_t, ndim=2] bb1,
                            np.ndarray[np.float64_t, ndim=2] bb2,
                            double delta = 0.01):
    # Touching is not overlap
    # return not (np.any(bb1[0] >= bb2[1]) or np.any(bb1[1] <= bb2[0]))
    # Due to a Cython bug... cannot convert numpy.bool_ to bool
    return False if \
           bb1[0][0]-delta >= bb2[1][0]+delta or bb1[1][0]+delta <= bb2[0][0]-delta or \
           bb1[0][1]-delta >= bb2[1][1]+delta or bb1[1][1]+delta <= bb2[0][1]-delta or \
           bb1[0][2]-delta >= bb2[1][2]+delta or bb1[1][2]+delta <= bb2[0][2]-delta else True
