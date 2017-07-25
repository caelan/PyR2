import numpy as np
cimport numpy as np
from cpython cimport bool
cimport PyR2.hu as hu

# BBoxes

cpdef np.ndarray[np.float64_t, ndim=2] vertsBBox(np.ndarray[np.float64_t, ndim=2] verts,
                                                 np.ndarray[np.int_t, ndim=1] indices)
cpdef np.ndarray[np.float64_t, ndim=1] bboxCenter(np.ndarray[np.float64_t, ndim=2] bb,
                               	       	          bool base = *)
cpdef np.ndarray[np.float64_t, ndim=2] bboxDims(np.ndarray[np.float64_t, ndim=2] bb)
cpdef np.ndarray[np.float64_t, ndim=2] bboxUnion(list bboxes)
cpdef np.ndarray[np.float64_t, ndim=2] bboxIsect(list bboxes)
cpdef bool bboxOverlap(np.ndarray[np.float64_t, ndim=2] bb1,
                       np.ndarray[np.float64_t, ndim=2] bb2)
cpdef bool bboxIn(np.ndarray[np.float64_t, ndim=2] bb,
                  np.ndarray[np.float64_t, ndim=1] p)
cpdef bool bboxContains(np.ndarray[np.float64_t, ndim=2] bb,
                        np.ndarray[np.float64_t, ndim=1] pt)
cpdef bool bboxInside(np.ndarray[np.float64_t, ndim=2] bb1,
                      np.ndarray[np.float64_t, ndim=2] bb2)
cpdef double bboxVolume(np.ndarray[np.float64_t, ndim=2] bb)
cpdef np.ndarray[np.float64_t, ndim=2] bboxGrow(np.ndarray[np.float64_t, ndim=2] bb,
                          np.ndarray[np.float64_t, ndim=1] off)
cpdef np.ndarray[np.float64_t, ndim=2] bboxZproject(np.ndarray[np.float64_t, ndim=2] bb)
cpdef np.ndarray[np.float64_t, ndim=2] bboxMinkowskiXY(np.ndarray[np.float64_t, ndim=2] bb1,
                                 np.ndarray[np.float64_t, ndim=2] bb2)
cpdef np.ndarray[np.float64_t, ndim=2] bboxRefXY(np.ndarray[np.float64_t, ndim=2] bb)
cpdef np.ndarray[np.float64_t, ndim=2] bboxOrigin(np.ndarray[np.float64_t, ndim=2] bb)
cpdef bool bboxGrownOverlap(np.ndarray[np.float64_t, ndim=2] bb1,
                  	    np.ndarray[np.float64_t, ndim=2] bb2,
		            double delta = *)			
