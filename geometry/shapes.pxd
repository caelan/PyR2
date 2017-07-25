import math
import numpy as np
cimport numpy as np
cimport geometry.hu as hu
import geometry.hu as hu
from cpython cimport bool

cdef class Thing:
    cdef public dict properties
    cdef np.ndarray thingBBox
    cdef Prim thingPrim
    cdef hu.Transform thingOrigin
    cdef str thingString
    cdef np.ndarray thingVerts, thingPlanes, thingEdges

    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self)
    cpdef str name(self)
    cpdef hu.Transform origin(self)
    cpdef list parts(self)
    cpdef tuple zRange(self)
    cpdef np.ndarray[np.float64_t, ndim=1] center(self)
    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self)
    cpdef np.ndarray[np.float64_t, ndim=2] planes(self)
    cpdef np.ndarray[np.int_t, ndim=2] edges(self)
    cpdef Thing applyTrans(self, hu.Transform trans, str frame=*)
    cpdef Thing applyLoc(self, hu.Transform trans, str frame=*)
    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt)
    # cpdef Prim prim(self)
    # cpdef bool collides(self, Thing obj)
    # cpdef Shape cut(self, Thing obj, bool isect = *)
    # cpdef Prim xyPrim(self)
    # cpdef Prim boundingRectPrim(self)

# cdef class BasePrim:
#     cdef public dict properties
#     cdef public np.ndarray baseVerts, basePlanes, baseEdges, baseBBox, baseRings
#     cdef public list baseFaceFrames, baseFaces
#     cdef public Prim basePrim
#     cdef public hu.Transform baseOrigin
#     cdef public str baseString

cdef class BasePrim:
    cdef public np.ndarray baseVerts, baseRings
    cdef public list baseFaces
    cdef np.ndarray i_baseBBox, i_basePlanes, i_baseEdges
    cdef str i_baseString
    cdef list i_baseFaceFrames
    cpdef np.ndarray[np.float64_t, ndim=2] baseBBox(self)
    cpdef np.ndarray[np.float64_t, ndim=2] basePlanes(self)
    cpdef np.ndarray[np.int_t, ndim=2] baseEdges(self)
    cpdef list baseFaceFrames(self)
    cpdef str baseString(self)

cdef class Prim:
    cdef public BasePrim basePrim
    cdef public dict properties
    cdef public hu.Transform primOrigin
    cdef np.ndarray primVerts, primPlanes, primBBox
    cdef tuple tupleBBox

    cpdef Prim prim(self)
    cpdef list parts(self)
    cpdef list toPrims(self)	
    cpdef str name(self)
    cpdef hu.Transform origin(self)
    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self)
    cpdef np.ndarray[np.float64_t, ndim=2] getVertices(self)
    cpdef list faces(self)
    cpdef np.ndarray[np.float64_t, ndim=2] planes(self)
    cpdef np.ndarray[np.int_t, ndim=2] edges(self)
    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self)
    cpdef tuple zRange(self)
    cpdef Prim applyTrans(self, hu.Transform trans, str frame=*)
    cpdef Prim applyLoc(self, hu.Transform trans, str frame=*)
    cpdef Shape cut(self, obj, bool isect = *)
    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt)
    # cpdef np.ndarray containsPts(self, np.ndarray[np.float64_t, ndim=2] pts)
    cpdef list faceFrames(self)
    cpdef bool collides(self, obj)
    cpdef Prim xyPrim(self)
    cpdef Prim boundingRectPrim(self)
    cpdef tuple desc(self)

cdef class Shape:
    cdef list shapeParts
    cdef public dict properties
    cdef public hu.Transform shapeOrigin
    cdef public np.ndarray shapeBBox
    cdef public tuple tupleBBox
    
    cpdef list parts(self)
    cpdef list toPrims(self)	
    cpdef str name(self)
    cpdef tuple zRange(self)
    cpdef hu.Transform origin(self)
    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self)
    cpdef np.ndarray[np.float64_t, ndim=2] getVertices(self)
    cpdef Shape applyTrans(self, hu.Transform, str frame=*)
    cpdef Shape applyLoc(self, hu.Transform trans, str frame=*)
    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt)
    # cpdef np.ndarray containsPts(self, np.ndarray[np.float64_t, ndim=2] pts)
    cpdef bool collides(self, obj)
    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self)
    cpdef Shape cut(self, obj, bool isect = *)
    cpdef Prim prim(self)
    cpdef Prim xyPrim(self)
    cpdef Prim boundingRectPrim(self)
    cpdef list faceFrames(self)
    cpdef tuple desc(self)

cdef class Box(Prim):
    cdef nothing

cdef class BoxScale(Prim):
    cdef nothing

cdef class SkewCone(Prim):
    cdef nothing

cdef class Ngon(Prim):
    cdef nothing

cdef class BoxAligned(Prim):
    cdef nothing

cdef class Polygon(Prim):
    cdef nothing

cpdef np.ndarray[np.float64_t, ndim=2] convexHullVertsXY(np.ndarray[np.float64_t, ndim=2] verts)
cpdef Prim convexHullPrim(np.ndarray[np.float64_t, ndim=2] verts, hu.Transform origin)

