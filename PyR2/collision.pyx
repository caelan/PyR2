import math
import numpy as np
cimport numpy as np
from cpython cimport bool
cimport PyR2.shapes as shapes
import PyR2.gjk2 as gjk

tiny = 1.0e-6
cpdef bool primPrimCollides(shapes.Prim t1, shapes.Prim t2):
    return gjk.gjkDist(t1, t2) < 1.0e-6

cpdef bool primPrimCollidesReal(shapes.Prim t1, shapes.Prim t2):
    cdef:
        np.ndarray[np.float64_t, ndim=2] verts1, verts2, planes1, planes2, f2xv1, f1xv2
        int nv1, nv2, nt1, nt2, i, j, inside, separate
    planes1 = t1.planes(); nt1 = planes1.shape[0]
    verts2 = t2.vertices(); nv2 = verts2.shape[1]
    f1xv2 = np.dot(planes1, verts2)
    # if np.any(np.all(f1xv2 <= 0, axis=0)): return True # some v2 inside t1
    for i in xrange(nv2):
        inside = 1
        for j in xrange(nt1):
            if f1xv2[j,i] > 0.:         # face x vertex
                inside = 0
                break
        if inside==1: return True
    # if np.any(np.all(f1xv2 > 0, axis=1)): return False # some plane separates
    for i in xrange(nt1):
        separate = 1
        for j in xrange(nv2):
            if f1xv2[i,j] <= 0.:
                separate = 0
                break
        if separate==1: return False
    planes2 = t2.planes(); nt2 = planes2.shape[0]
    verts1 = t1.vertices(); nv1 = verts1.shape[1]
    f2xv1 = np.dot(planes2, verts1);
    # if np.any(np.all(f2xv1 <= 0, axis=0)): return True # some v1 inside t2
    for i in xrange(nv1):
        inside = 1
        for j in xrange(nt2):
            if f2xv1[j,i] > 0.:         # face x vertex
                inside = 0
                break
        if inside==1: return True
    # if np.any(np.all(f2xv1 > 0, axis=1)): return False # some plane separates
    for i in xrange(nt2):
        separate = 1
        for j in xrange(nv1):
            if f2xv1[i,j] <= 0.:
                separate = 0
                break
        if separate==1: return False
    # Check if centers are inside, mostly to handle alignments.
    if np.all(np.dot(planes1, np.resize((1./nv2)*np.sum(verts2, axis=1),(4,1)))<=0): return True
    if np.all(np.dot(planes2, np.resize((1./nv1)*np.sum(verts1, axis=1),(4,1)))<=0): return True
    return primPrimCollidesAux(t1, t2, f2xv1) or\
           primPrimCollidesAux(t2, t1, f1xv2)

# check edges of 1 vs faces of 2
cpdef bool primPrimCollidesAux(shapes.Prim prim1, shapes.Prim prim2,
                                np.ndarray[np.float64_t, ndim=2] f2xv1): 
    cdef:
        np.ndarray[np.float64_t, ndim=2] verts
        np.ndarray[np.float64_t, ndim=1] crossPlanes, p0, p1
        np.ndarray[np.int_t, ndim=2] edges
        np.ndarray[np.int_t, ndim=1] indices
        int e
    verts = prim1.vertices()            # 4xn array
    edges = prim1.edges()               # ex2 array
    for e in range(edges.shape[0]):
        crossPlanes = f2xv1[:, edges[e,0]] * f2xv1[:, edges[e,1]]
        indices = np.where(crossPlanes < 0)[0]
        if np.any(indices):
            p0 = verts[:, edges[e,0]]
            p1 = verts[:, edges[e,1]]
            if edgeCross(p0, p1,
                         f2xv1[np.ix_(indices, edges[e])],
                         prim2):
                return True
    return False

cpdef bool edgeCross(np.ndarray[np.float64_t, ndim=1] p0, # row vector
                     np.ndarray[np.float64_t, ndim=1] p1, # row vector
                     np.ndarray[np.float64_t, ndim=2] dots,
                     shapes.Prim prim):
    cdef:
        np.ndarray[np.float64_t, ndim=1] diff, pt
        double d0, d1, prod, t
        int i
    diff = p1 - p0
    for i in range(dots.shape[0]):
        d0 = dots[i, 0]; d1 = dots[i, 1]
        prod = d0 * d1
        if prod >= 0: continue          # same side
        t = - d0/(d1 - d0)
        assert 0 <= t <= 1, 'Invalid t'
        pt = p0 + t*diff
        if np.all(np.dot(prim.planes(), pt.reshape(4,1)) <= tiny):          # brute force
            return True
    return False

