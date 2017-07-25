from cpython cimport bool
import numpy as np
cimport numpy as np

cpdef np.ndarray[np.float64_t, ndim=1] quaternion_from_matrix(np.ndarray[np.float64_t, ndim=2] M)
cpdef np.ndarray[np.float64_t, ndim=1] quaternion_about_axis(double angle, tuple axis)
cpdef np.ndarray[np.float64_t, ndim=2] quaternion_matrix(np.ndarray[np.float64_t, ndim=1] iq)
cpdef np.ndarray[np.float64_t, ndim=2] translation_matrix(np.ndarray[np.float64_t, ndim=1] direction)

