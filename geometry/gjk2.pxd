import numpy as np
cimport numpy as np
cimport geometry.shapes as shapes
from cpython cimport bool
from geometry.c_gjk cimport Object_structure

# The C structure for an object or link (note that this is fixed size)
cdef struct OS:
    int nOS                   # 1 (only object), 2 (object and shadow)
    double trans[4][4]        # single transform
    Object_structure obj[2]   # object and shadow
    int perm[2]               # permanent flag
    double bbox[2][2][3]      # [O/S][Lo/Hi][x/y/z]
    int name_id[2]            # index to external names

cdef class OS_Array:
    cdef OS *data
    cdef public size_t number

cpdef double gjkDist(shapes.Prim prim1, shapes.Prim prim2)
cpdef confViolationsOS(conf, tuple CC, OS_Array robOSa, list attOSal, OS_Array obOSa,
                       list selfCollidePairs, np.ndarray np_robMove,
                       dict objShapes, list objNames, bool prdebug=*, minDist=*, method=*)
cdef int checkCollisionsOS(OS_Array moveOSa, int[:] move,
                           OS_Array statOSa, int[:] stat,
                           int[:] collision, int off,
                           bool prdebug, double minDist)
cdef bool confSelfCollideOS(OS_Array robOSa,
                            OS_Array attOSa_l, OS_Array attOSa_r,
                            list selfCollidePairs, minDist, prdebug=*)
cpdef clearPlaceCache()
cdef updatePlaceCache(key, value, i)
cdef void placeChains(conf, tuple CC, OS_Array robOSa, OS_Array attOSa_l, OS_Array attOSa_r, int method)

cpdef printOSa(OS_Array osa, title, index = *)
