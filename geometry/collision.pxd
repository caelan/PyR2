import math
import numpy as np
cimport numpy as np
from cpython cimport bool
cimport geometry.shapes as shapes
import geometry.shapes as shapes

cpdef bool primPrimCollides(shapes.Prim p1, shapes.Prim p2)
cpdef bool primPrimCollidesReal(shapes.Prim t1, shapes.Prim t2)
cpdef bool primPrimCollidesAux(shapes.Prim prim1, shapes.Prim prim2, 
                               np.ndarray[np.float64_t, ndim=2] f2xv1)
cpdef bool edgeCross(np.ndarray[np.float64_t, ndim=1] p0, # row vector
                     np.ndarray[np.float64_t, ndim=1] p1, # row vector
                     np.ndarray[np.float64_t, ndim=2] planes,
                     shapes.Prim prim)

