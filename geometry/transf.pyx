import math
import numpy as np
cimport numpy as np
from cpython cimport bool

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0
# Types for transform matrices, for points and point matrices
FL = np.float64

cpdef np.ndarray[np.float64_t, ndim=1] quaternion_from_matrix(np.ndarray[np.float64_t, ndim=2] M):
    """Return quaternion from rotation matrix.

    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R)
    >>> np.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
    True

    """
    cdef np.ndarray[np.float64_t, ndim=1] q
    cdef double t
    cdef int i, j, k
    q = np.empty((4, ), dtype=FL)
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q

cpdef np.ndarray[np.float64_t, ndim=2] quaternion_matrix(np.ndarray[np.float64_t, ndim=1] iq):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    cdef double nq
    cdef np.ndarray[np.float64_t, ndim=2] q
    iq = np.array(iq[:4], dtype=FL, copy=True)
    nq = np.dot(iq, iq)
    if nq < _EPS:
        return np.identity(4)
    iq *= math.sqrt(2.0 / nq)
    q = np.outer(iq, iq)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=FL)

cpdef np.ndarray[np.float64_t, ndim=2] translation_matrix(np.ndarray[np.float64_t, ndim=1] direction):
    """Return matrix to translate by direction vector.

    >>> v = np.random.random(3) - 0.5
    >>> np.allclose(v, translation_matrix(v)[:3, 3])
    True

    """
    cdef np.ndarray[np.float64_t, ndim=2] M
    M = np.identity(4)
    M[:3, 3] = direction[:3]
    return M

cpdef np.ndarray[np.float64_t, ndim=1] quaternion_about_axis(double angle, tuple axis):
    """Return quaternion for rotation about axis.

    >>> q = quaternion_about_axis(0.123, (1, 0, 0))
    >>> np.allclose(q, [0.06146124, 0, 0, 0.99810947])
    True

    """
    cdef np.ndarray[np.float64_t, ndim=1] quaternion
    cdef double qlen
    quaternion = np.zeros((4, ), dtype=FL)
    quaternion[:3] = axis[:3]
    qlen = np.linalg.norm(quaternion)
    if qlen > _EPS:
        quaternion *= math.sin(angle/2.0) / qlen
    quaternion[3] = math.cos(angle/2.0)
    return quaternion
