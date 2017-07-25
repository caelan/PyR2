#import math
import numpy as np
import geometry.shapes as shapes
import geometry.geom as geom
import itertools

tiny = 1.0e-6

def CO(A, B, signs=(-1,1)):
    Averts = A.vertices()
    Bverts = B.vertices()
    verts = []
    for i in range(Bverts.shape[1]):
        verts.append(signs[1]*Bverts[:,i:i+1] + signs[0]*Averts)
    for v in verts: v[3,:]=1.0
    return shapes.convexHullPrim(np.hstack(verts), B.origin())

def COParts(A, B, signs=(-1,1)):
    co = []
    for a in A.parts():
        for b in B.parts():
           co.append(CO(a,b,signs))
    return co

def xyCO(A, B, signs=(-1,1)):
    Averts = A.vertices()
    Bverts = B.vertices()
    verts = []
    for i in range(Bverts.shape[1]):
        verts.append(signs[1]*Bverts[:,i:i+1] + signs[0]*Averts)
    for v in verts: v[3,:]=1.0
    zrA = [signs[0]*x for x in A.zRange()]
    zrB = [signs[1]*x for x in B.zRange()]
    zr = min(zrB[0]+zrA[0],zrB[0]+zrA[1]),max(zrB[1]+zrA[0],zrB[1]+zrA[1])

    return shapes.Polygon(shapes.convexHullVertsXY(np.hstack(verts)),
                          zr, B.origin())

def xyCOParts(A, B, signs=(-1,1)):
    co = []
    for a in A.parts():
        for b in B.parts():
           co.append(xyCO(a,b,signs))
    return co

def xyCI(A, B):                         # Only for convex B
    Az = A.zRange()
    Bz = B.zRange()
    zRange = (Bz[0] - Az[0], Bz[1]-Az[1])
    if zRange[1] < zRange[0]:
        return None
    aVerts = A.vertices()
    bPlanes = B.planes()
    fxv = np.dot(bPlanes[:,:3], aVerts[:3,:])
    off = np.resize(np.max(fxv, axis=1), (bPlanes.shape[0], 1))
    cPlanes = np.hstack([bPlanes.copy()[:,:3], bPlanes[:,3:4]+off])
    indices = [i for i in range(cPlanes.shape[0]) if abs(cPlanes[i,2]) < 0.01]
    cVerts = xyVertsFromPlanes(cPlanes[indices, :])
    return shapes.Polygon(cVerts, zRange, B.origin()) if not cVerts is None else None

def xyVertsFromPlanes(planes):           # only side planes
    verts = []
    one = np.array([0., 1.])
    for inds in itertools.combinations(range(planes.shape[0]), 2):
        try:
            v = np.linalg.solve(planes[inds,:2], planes[inds,3])
        except:
            continue
        pt = np.hstack([v, one])
        if np.all(np.dot(planes, pt) <= tiny):
            verts.append(pt)
    return shapes.convexHullVertsXY(np.vstack(verts).T) if verts else None

def CI(A, B):                           # Only for convex B
    aVerts = A.vertices()
    bPlanes = B.planes()
    fxv = np.dot(bPlanes[:,:3], aVerts[:3,:])
    off = np.resize(np.max(fxv, axis=1), (bPlanes.shape[0], 1))
    cPlanes = np.hstack([bPlanes.copy()[:,:3], bPlanes[:,3:4]+off])
    cVerts = vertsFromPlanes(cPlanes)
    return shapes.convexHullPrim(cVerts, B.origin()) if not cVerts is None else None

def vertsFromPlanes(planes):
    verts = []
    one = np.ones(1)
    for inds in itertools.combinations(range(planes.shape[0]), 3):
        try:
            v = np.linalg.solve(planes[inds,:3], -planes[inds,3])
        except:
            continue
        pt = np.hstack([v, one])
        if np.all(np.dot(planes, pt) <= tiny):
            verts.append(pt)
    return np.vstack(verts).T if verts else None

