import math

import numpy as np
cimport numpy as np
from cpython cimport bool

cimport geometry.hu as hu
import geometry.hu as hu

from geometry.geom import vertsBBox, bboxCenter, bboxUnion, bboxOverlap, bboxGrownOverlap, bboxOrigin
from geometry.cut import primPrimCut

import graphics.windowManager3D as wm

cimport geometry.collision as collision
import geometry.collision as collision
from geometry.collision import primPrimCollides

from scipy.spatial import ConvexHull

Ident = hu.Transform(np.eye(4))

#################################
# Object classes: Thing, Prim, Shape
#################################

cdef double tiny = 1.0e-6

cdef class Thing:
    """Most unspecific class of object, characterized by a bbox. All
    Things also have a properties dictionary, which includes a name."""
    def __init__(self,
                 np.ndarray[np.float64_t, ndim=2] bbox,
                 hu.Transform origin,
                 **props):
        self.properties = props.copy()
        if not 'name' in self.properties:
            self.properties['name'] = hu.gensym('Thing')
        self.thingBBox = bbox
        if origin:
            self.thingOrigin = origin
        else:
            trans = np.eye(4, dtype=np.float64)
            trans[:3, 3] = bboxCenter(bbox)[:3] if (not bbox is None) else np.zeros(3)
            self.thingOrigin = hu.Transform(trans)
        self.thingVerts = None
        self.thingPlanes = None
        self.thingEdges = None

    cpdef str name(self):
        return self.properties.get('name', 'noName')

    cpdef hu.Transform origin(self):
        return self.thingOrigin

    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self):
        """Returns bbox (not a copy)."""
        return self.thingBBox

    cpdef list parts(self):
        return [self]

    cpdef tuple zRange(self):
        """Summarizes z range of bbox by a tuple"""
        cdef np.ndarray[np.float64_t, ndim=2] bb
        bb = self.bbox()
        return (bb[0,2], bb[1,2])

    cpdef np.ndarray[np.float64_t, ndim=1] center(self):
        """Returns a point at the center of the bbox."""
        cdef np.ndarray[np.float64_t, ndim=1] center = bboxCenter(self.bbox())
        return center

    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self):
        cdef:
            double xlo, ylo, zlo, xhi, yhi, zhi
            np.ndarray[np.float64_t, ndim=2] points, bb
        if self.thingVerts is None:
            bb = self.bbox()
            xlo = bb[0,0]; ylo = bb[0,1]; zlo = bb[0,2]
            xhi = bb[1,0]; yhi = bb[1,1]; zhi = bb[1,2]
            points = np.array([[xlo, ylo, zlo, 1.], [xhi, ylo, zlo, 1.],
                               [xhi, yhi, zlo, 1.], [xlo, yhi, zlo, 1.]]).T
            self.thingVerts = vertsFrom2D(points, zlo, zhi)
        return self.thingVerts

    cpdef np.ndarray[np.float64_t, ndim=2] planes(self):
        cdef:
            double xlo, ylo, zlo, xhi, yhi, zhi
            np.ndarray[np.float64_t, ndim=2] bb
        if self.thingPlanes is None:
            bb = self.bbox()
            self.thingPlanes = np.array([[-1.,0.,0., bb[0,0]], [1.,0.,0., -bb[1,0]],
                                         [0.,-1,0., bb[0,1]], [0.,1.,0., -bb[1,1]],
                                         [0.,0.,-1., bb[0,2]], [0.,0.,1., -bb[1,2]]])
        return self.thingPlanes

    cpdef np.ndarray[np.int_t, ndim=2] edges(self):
        if self.thingEdges is None:
            self.thingEdges= np.array([[3, 2], [2, 6], [6, 7], [1, 5], [4, 5], [2, 1],
                                       [7, 4], [5, 6], [3, 7], [0, 3], [1, 0], [4, 0]], dtype=np.int)
        return self.thingEdges

    cpdef Thing applyTrans(self, hu.Transform trans, str frame='unspecified'):
        """Displace the Thing; returns a Prim."""
        return self.prim().applyTrans(trans, frame)

    cpdef Thing applyLoc(self, hu.Transform trans, str frame='unspecified'):
        """Displace the Thing to a location; returns a Prim."""
        return self.applyTrans(trans.compose(self.origin().inverse()), frame)

    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt):
    #     """Test whether the Thing's bbox contains the Point pt."""
    #     return bboxContains(self.bbox(), pt)

    # cpdef Prim prim(self):
    #     """Constructs a Prim that matches the bbox.  Useful when computing
    #     collisions, etc."""
    #     prim = BoxAligned(self.bbox(), self.thingOrigin, **self.properties)
    #     return prim

    # cpdef bool collides(self, Thing obj):
    #     """Test whether the Thing's collides with another obj, that
    #     could be any of the types of Thing."""
    #     if self.bbox() is None or obj.bbox() is None: return False
    #     if  bboxOverlap(self.bbox(), obj.bbox()): 
    #         if isinstance(obj, (Prim, Shape)): # more general type, pass the buck
    #             return obj.collides(self)
    #         elif isinstance(obj, Thing):
    #             return True             # already checked bbox
    #         else:
    #             raise Exception, 'Unknown obj type'%str(obj)
    #     return False

    # cpdef Shape cut(self, Thing obj, bool isect = False):
    #     if not (obj.bbox() is None) and bboxOverlap(self.bbox(), obj.bbox()):
    #         if isinstance(obj, Shape):
    #             return self.prim().cut(obj, isect=isect)
    #         else:
    #             ans = primPrimCut(self.prim(), obj.prim(), isect=isect)
    #             if ans:
    #                 return Shape([ans], self.origin(), **self.properties)
    #     return None if isect else self

    # cpdef Prim xyPrim(self):
    #     return self.prim()

    # cpdef Prim boundingRectPrim(self):
    #     return self.prim()

    cpdef draw(self, str window, str color = 'black', double opacity = 1.0):
        """Ask the window to draw this object."""
        wm.getWindow(window).draw(self, color, opacity)
        
    def __str__(self):
        return self.properties['name']+':'+str(self.bbox().tolist())
    def __repr__(self):
        return str(self)
    def __hash__(self):
        return repr(self).__hash__()
    def __richcmp__(self, other, int op):
        if not (other and isinstance(other, Thing)):
            return True if op == 3 else False
        if op == 2:
            ans = self.name() == other.name() and repr(self) == repr(other)
        elif op == 3:
            ans = self.name() != other.name() or repr(self) != repr(other)
        else:
            ans = False
        return ans

# Prim class: convex "chunk", has 3D description and we can get 2.5D
# approx via convex hull of projection.

cdef vertsRadius(np.ndarray[np.float64_t, ndim=2] verts):
    cdef double rad = 0.0
    cdef int i
    for i in verts.shape[1]:
        rad = max(rad, verts[0,i]*verts[0,i] + verts[1,i]*verts[1,i] +  verts[2,i]*verts[2,i] )
    return math.sqrt(rad)

# This is the base (unchanging) description of a convex chunk.
# cdef class BasePrim:
#     def __init__(self,
#                  np.ndarray[np.float64_t, ndim=2] verts,
#                  list faces,
#                  **props):
#         cdef np.ndarray[np.float64_t, ndim=2] bb
#         self.properties = props.copy()
#         if not 'name' in self.properties:
#             self.properties['name'] = hu.gensym('Base')
#         self.baseBBox = bb = vertsBBox(verts, None)   # the bbox for original verts
#         self.baseVerts = verts
#         self.baseFaces = faces
#         self.basePlanes = primPlanes(self.baseVerts, faces)
#         self.baseEdges = primEdges(self.baseVerts, faces)
#         self.baseRings = None # primRings(self.baseEdges)
#         # TODO: Should use (large) convex hull faces -- how to refer to them?
#         bbPlanes = np.array([[-1.,0.,0., bb[0,0]], [1.,0.,0., -bb[1,0]],
#                              [0.,-1,0., bb[0,1]], [0.,1.,0., -bb[1,1]],
#                              [0.,0.,-1., bb[0,2]], [0.,0.,1., -bb[1,2]]])
#         # We're using the Ident as the origin, by construction.
#         self.baseFaceFrames = thingFaceFrames(bbPlanes, Ident)
#         self.baseString = self.properties['name']+':'+str(self.baseBBox.tolist())
#     def __str__(self):
#         return self.baseString
#     def __repr__(self):
#         return self.baseString

cdef class BasePrim:
    def __init__(self,
                 np.ndarray[np.float64_t, ndim=2] verts,
                 list faces):
        self.baseVerts = verts
        self.baseFaces = faces
        self.baseRings = None
        self.i_baseBBox = None
        self.i_basePlanes = None
        self.i_baseEdges = None
        self.i_baseFaceFrames = None
        self.i_baseString = None
        
    cpdef np.ndarray[np.float64_t, ndim=2] baseBBox(self):
        if self.i_baseBBox is None:
            self.i_baseBBox = vertsBBox(self.baseVerts, None) # the bbox for original verts
        return self.i_baseBBox

    cpdef np.ndarray[np.float64_t, ndim=2] basePlanes(self):
        if self.i_basePlanes is None:
            self.i_basePlanes = primPlanes(self.baseVerts, self.baseFaces)
        return self.i_basePlanes

    cpdef np.ndarray[np.int_t, ndim=2] baseEdges(self):
        if self.i_baseEdges is None:
            self.i_baseEdges = primEdges(self.baseVerts, self.baseFaces)
        return self.i_baseEdges

    cpdef list baseFaceFrames(self):
        cdef np.ndarray[np.float64_t, ndim=2] bb
        if self.i_baseFaceFrames is None:
            bb = self.baseBBox()
            # TODO: Should use (large) convex hull faces -- how to refer to them?
            bbPlanes = np.array([[-1.,0.,0., bb[0,0]], [1.,0.,0., -bb[1,0]],
                                 [0.,-1,0., bb[0,1]], [0.,1.,0., -bb[1,1]],
                                 [0.,0.,-1., bb[0,2]], [0.,0.,1., -bb[1,2]]], dtype=np.float64)
            # We're using the Ident as the origin, by construction.
            self.i_baseFaceFrames = thingFaceFrames(bbPlanes, Ident)
        return self.i_baseFaceFrames

    cpdef str baseString(self):
        if self.i_baseString is None:
            self.i_baseString = self.properties['name']+':'+str(self.baseBBox.tolist())
        return self.i_baseString
    def __str__(self):
        return self.baseString()
    def __repr__(self):
        return self.baseString()


# This is a located convex chunk, need to provide (verts, faces, origin) or a BasePrim
cdef class Prim:
    def __init__(self,
                 np.ndarray[np.float64_t, ndim=2] verts,
                 list faces,            # since faces have variable length
                 hu.Transform origin,
                 BasePrim bs,
                 **props):
        self.properties = props.copy()
        self.primOrigin = origin or Ident
        self.basePrim = bs or BasePrim(np.dot(self.primOrigin.inverse().matrix, verts), faces)
        if not 'name' in self.properties:
            self.properties['name'] = hu.gensym('Prim')
        self.primVerts = None
        self.primPlanes = None
        self.primBBox = None
        self.tupleBBox = None

    cpdef Prim prim(self):
        return self

    cpdef list toPrims(self):
        return [self]

    cpdef list parts(self):
        return [self]

    cpdef str name(self):
        return self.properties.get('name', 'noName')

    cpdef hu.Transform origin(self):
        return self.primOrigin

    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self):
        if self.primVerts is None:
            self.primVerts = np.dot(self.primOrigin.matrix, self.basePrim.baseVerts)
        return self.primVerts

    cpdef np.ndarray[np.float64_t, ndim=2] getVertices(self): # same as vertices
        if self.primVerts is None:
            self.primVerts = np.dot(self.primOrigin.matrix, self.basePrim.baseVerts)
        return self.primVerts

    cpdef np.ndarray[np.float64_t, ndim=2] planes(self):
        if self.primPlanes is None:
            self.primPlanes = np.dot(self.basePrim.basePlanes(),
                                     self.primOrigin.inverse().matrix)
        return self.primPlanes

    cpdef list faces(self):
        return self.basePrim.baseFaces

    cpdef np.ndarray[np.int_t, ndim=2] edges(self):
        return self.basePrim.baseEdges()

    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self):
        if self.primBBox is None:
            self.primBBox = vertsBBox(self.vertices(), None)
            self.tupleBBox = tuple([tuple(x) for x in self.primBBox.tolist()])
        return self.primBBox

    cpdef tuple zRange(self):
        """Summarizes z range of bbox by a tuple"""
        cdef np.ndarray[np.float64_t, ndim=2] bb
        bb = self.bbox()
        return (bb[0,2], bb[1,2])

    cpdef Prim applyTrans(self, hu.Transform trans, str frame='unspecified',):
        return Prim(None, None,         # basePrim has the relevant info
                    trans.compose(self.primOrigin),
                    self.basePrim,
                    **mergeProps(self.properties, {'frame':frame}))

    cpdef Prim applyLoc(self, hu.Transform trans, str frame='unspecified'):
        """Displace the Thing to a location; returns a Prim."""
        return self.applyTrans(trans.compose(self.origin().inverse()), frame)

    cpdef list faceFrames(self):
        return [self.primOrigin.compose(fr) for fr in self.basePrim.baseFaceFrames()]

    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt):
    #     return True if np.all(np.dot(self.planes(), pt.reshape(4,1)) <= tiny) else False

    # cpdef np.ndarray containsPts(self, np.ndarray[np.float64_t, ndim=2] pts):
    #     """Returns array of booleans"""
    #     return np.all(np.dot(self.planes(), pts) <= tiny, axis=0)

    cpdef bool collides(self, obj):
        if isinstance(obj, Shape):  # more general type, pass the buck
            return obj.collides(self)
        else:
            if not bboxGrownOverlap(self.bbox(), obj.bbox()):
                return False
            return primPrimCollides(self, obj)

    cpdef Shape cut(self, obj, bool isect = False):
        if bboxOverlap(self.bbox(), obj.bbox()):
            if isinstance(obj, Shape):  # Shape is a union of objects
                ans = []
                p1 = self
                if bboxOverlap(p1.bbox(), obj.bbox()):
                    if isect:
                        # We want union of intersections.
                        for p2 in obj.parts():
                            cut = p1.cut(p2, isect).parts()
                            ans.extend(cut)
                    else:
                        # For diff, find pieces of p1 outside all of obj
                        p1Parts = [p1]
                        for p2 in obj.parts():
                            temp = [] # will hold pieces of p1 outside of p2
                            for p in p1Parts: # loop over every piece of p1
                                cut = p.cut(p2, isect).parts()
                                temp.extend(cut) # add result to temp
                            p1Parts = temp       # set up p1 parts for next p2
                        ans.extend(p1Parts)
                elif not isect:     # doing diff
                    ans.append(p1)  # keep p1, else ignore for isect
                return Shape(ans, self.origin(), **self.properties) if ans else None
            else:
                ans = primPrimCut(self, obj.prim(), isect=isect)
                if ans:
                    Shape([ans], self.origin(), **self.properties)
        return None if isect else Shape([self], self.origin(), **self.properties)
    
    # Compute XY convex hull
    cpdef Prim xyPrim(self):
        return xyPrimAux(self.vertices(), self.zRange(), self.primOrigin, self.properties)

    # Compute least inertia box
    cpdef Prim boundingRectPrim(self):
        return boundingRectPrimAux(self.vertices(), self.primOrigin, self.properties)

    cpdef draw(self, str window, str color = 'black', double opacity = 1.0):
        """Ask the window to draw this object."""
        wm.getWindow(window).draw(self, color, self.properties.get('opacity', opacity))

    cpdef tuple desc(self):
        if not self.tupleBBox:
            self.bbox()                 # sets it
        return self.tupleBBox
    def __str__(self):
        return self.properties['name']+':'+str(self.desc())
    def __repr__(self):
        return str(self)
    def __hash__(self):
        return hash(self.properties['name']+':'+str(self.desc()))
    def __richcmp__(self, other, int op):
        if not (other and isinstance(other, Prim)):
            return True if op == 3 else False
        if op == 2:
            ans = self.name() == other.name() and self.desc() == other.desc()
        elif op == 3:
            ans = self.name() != other.name() or self.desc() != other.desc()
        else:
            ans = False
        return ans

cdef class Shape:
    def __init__(self, list parts, hu.Transform origin, **props):
        self.properties = props.copy()
        self.shapeParts = parts
        if origin:
            self.shapeOrigin = origin
        elif parts:
            self.shapeOrigin = hu.Transform(bboxOrigin(bboxUnion([p.bbox() for p in parts])))
        else:
            self.shapeOrigin = Ident
        if not 'name' in self.properties:
            self.properties['name'] = hu.gensym('Shape')
        self.shapeBBox = None
        self.tupleBBox = None

    cpdef list parts(self):
        return self.shapeParts

    cpdef list toPrims(self):
        prims = []
        for p in self.parts():
            prims.extend(p.toPrims())
        return prims

    cpdef tuple zRange(self):
        """Summarizes z range of bbox by a tuple"""
        cdef np.ndarray[np.float64_t, ndim=2] bb
        bb = self.bbox()
        return (bb[0,2], bb[1,2])

    cpdef str name(self):
        return self.properties.get('name', 'noName')

    cpdef hu.Transform origin(self):
        return self.shapeOrigin

    cpdef np.ndarray[np.float64_t, ndim=2] vertices(self):
        raw_input('Calling for vertices of compound shape')
        return None

    cpdef np.ndarray[np.float64_t, ndim=2] getVertices(self):
        return np.hstack([p.vertices() for p in self.toPrims() \
                          if not p.vertices() is None])

    cpdef Shape applyTrans(self, hu.Transform trans, str frame='unspecified'):
        return Shape([p.applyTrans(trans, frame) for p in self.parts()],
                     trans.compose(self.shapeOrigin),
                     **mergeProps(self.properties, {'frame':frame}))

    cpdef Shape applyLoc(self, hu.Transform trans, str frame='unspecified'):
        """Displace the Thing to a location; returns a Prim."""
        return self.applyTrans(trans.compose(self.origin().inverse()), frame)

    cpdef np.ndarray[np.float64_t, ndim=2] bbox(self):
        if self.shapeBBox is None:
            self.shapeBBox = bboxUnion([x.bbox() for x in self.parts()])
            self.tupleBBox = tuple([tuple(x) for x in self.shapeBBox.tolist()])
        return self.shapeBBox

    # cpdef bool containsPt(self, np.ndarray[np.float64_t, ndim=1] pt):
    #     for p in self.parts():
    #         if p.containsPt(pt): return True
    #     return False

    # cpdef np.ndarray containsPts(self, np.ndarray[np.float64_t, ndim=2] pts):
    #     """Returns array of booleans"""
    #     return np.array([self.containsPt(pts[i]) for i in range(pts.shape[0])])

    cpdef list faceFrames(self):
        # Use faceFrames for bounding box -- FOR NOW -- should be for convex hull?
        cdef:
            np.ndarray[np.float64_t, ndim=2] bb
        bb = self.bbox()
        bbPlanes = np.array([[-1.,0.,0., bb[0,0]], [1.,0.,0., -bb[1,0]],
                             [0.,-1,0., bb[0,1]], [0.,1.,0., -bb[1,1]],
                             [0.,0.,-1., bb[0,2]], [0.,0.,1., -bb[1,2]]])
        return thingFaceFrames(bbPlanes, self.shapeOrigin)

    cpdef bool collides(self, obj):
        if not bboxGrownOverlap(self.bbox(), obj.bbox()):
            return False
        # Is there any pairwise collision
        for p1 in self.toPrims():
            for p2 in obj.toPrims():
                if p1.collides(p2): return True
        return False

    cpdef Shape cut(self, obj, bool isect = False):
        # Shape is a union of objects
        cdef Prim p1, p2, op
        if bboxOverlap(self.bbox(), obj.bbox()):
            ans = []
            for p1 in self.parts(): # loop over parts of self
                if bboxOverlap(p1.bbox(), obj.bbox()):
                    if isect:
                        # We want union of intersections.
                        for p2 in obj.parts():
                            cut = p1.cut(p2, isect).parts()
                            ans.extend(cut)
                    else:
                        # For diff, find pieces of p1 outside all of obj
                        p1Parts = [p1]
                        for p2 in obj.parts():
                            temp = [] # will hold pieces of p1 outside of p2
                            for p in p1Parts: # loop over every piece of p1
                                cut = p.cut(p2, isect).parts()
                                temp.extend(cut) # add result to temp
                            p1Parts = temp       # set up p1 parts for next p2
                        ans.extend(p1Parts)
                elif not isect:     # doing diff
                    ans.append(p1)  # keep p1, else ignore for isect
            return Shape(ans, self.origin(), **self.properties) if ans else None
        return None if isect else self

    # Compute 3d convex hull
    cpdef Prim prim(self):
        verts = np.hstack([p.vertices() for p in self.toPrims() \
                           if not p.vertices() is None])
        return convexHullPrim(verts, self.shapeOrigin) \
               if self.parts() else None

    # Compute XY convex hull
    cpdef Prim xyPrim(self):
        cdef np.ndarray[np.float64_t, ndim=2] bb
        verts = np.hstack([p.vertices() for p in self.toPrims() \
                           if not p.vertices() is None])
        bb = vertsBBox(verts, None)
        zr = (bb[0,2], bb[1,2])
        return xyPrimAux(verts, zr, self.shapeOrigin, self.properties) \
               if self.parts() else None

    # Compute least inertia box
    cpdef Prim boundingRectPrim(self):
        return boundingRectPrimAux(self.vertices(), self.shapeOrigin, self.properties) \
               if self.parts() else None

    cpdef tuple desc(self):
        if not self.tupleBBox:
            self.bbox()                 # sets it
        return self.tupleBBox

    cpdef draw(self, str window, str color = 'black', double opacity = 1.0):
        """Ask the window to draw this object."""
        wm.getWindow(window).draw(self, color, self.properties.get('opacity', opacity))
    
    def __str__(self):
        return self.properties['name']+':'+str(self.desc())
    def __repr__(self):
        return str(self)
    def __hash__(self):
        return hash(self.properties['name']+':'+str(self.desc()))
    def __richcmp__(self, other, int op):
        if not (other and isinstance(other, Shape)):
            return True if op == 3 else False
        if op == 2:
            ans = self.name() == other.name() and self.desc() == other.desc()
        elif op == 3:
            ans = self.name() != other.name() or self.desc() != other.desc()
        else:
            ans = False
        return ans


#################################
# Object creation: Prims
#################################

cdef class Box(Prim):
    def __init__(self, double dx, double dy, double dz, hu.Transform origin, **props):
        cdef:
            double hdx = 0.5*dx
            double hdy = 0.5*dy
            double hdz = 0.5*dz
            np.ndarray[np.float64_t, ndim=2] points
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("box")})
        points = np.array([[-hdx, -hdy, -hdz, 1.], [hdx, -hdy, -hdz, 1.],
                           [hdx, hdy, -hdz, 1.], [-hdx, hdy, -hdz, 1.]]).T
        Prim.__init__(self,
                      vertsFrom2D(points, -hdz, hdz),
                      facesFrom2D(<int>points.shape[1]),
                      origin, None,
                      **props)

cdef class BoxScale(Prim):
    def __init__(self, double dx, double dy, double dz, hu.Transform origin, double scale, **props):
        cdef:
            double hdx = 0.5*dx
            double hdy = 0.5*dy
            double hdz = 0.5*dz
            np.ndarray[np.float64_t, ndim=2] points
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("box")})
        points = np.array([[-hdx, -hdy, -hdz, 1.], [hdx, -hdy, -hdz, 1.],
                           [hdx, hdy, -hdz, 1.], [-hdx, hdy, -hdz, 1.]]).T
        Prim.__init__(self,
                      vertsFrom2DScale(points, -hdz, hdz, scale),
                      facesFrom2D(<int>points.shape[1]),
                      origin, None,
                      **props)

cdef class SkewCone(Prim):
    def __init__(self, vertsLo, vertsHi, hu.Transform origin, **props):
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("box")})
        Prim.__init__(self,
                      np.hstack([vertsLo.T, vertsHi.T]),
                      facesFrom2D(<int>vertsLo.shape[1]),
                      origin, None,
                      **props)

cdef class Ngon(Prim):
    def __init__(self, double r, dz, int nsides, hu.Transform origin, **props):
        cdef:
            double hdz, ang
            int i
            np.ndarray[np.float64_t, ndim=2] points
        hdz = 0.5*dz
        ang = 2*math.pi/nsides
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("ngon")})
        points = np.array([[r*math.cos(i*ang), r*math.sin(i*ang), -hdz, 1.] \
                           for i in range(nsides)]).T
        Prim.__init__(self,
                      vertsFrom2D(points, -hdz, hdz),
                      facesFrom2D(<int>points.shape[1]),
                      origin, None,
                      **props)

cdef class BoxAligned(Prim):
    def __init__(self, np.ndarray[np.float64_t, ndim=2] bbox, hu.Transform origin, **props):
        cdef:
            double xlo, ylo, zlo, xhi, yhi, zhi
            np.ndarray[np.float64_t, ndim=2] points
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("box")})
        ((xlo, ylo, zlo), (xhi, yhi, zhi)) = bbox.tolist()
        if xhi < xlo or yhi < ylo or zhi < zlo:
            print 'bbox', bbox, props
            print 'BBox for BoxAligned is inside out, flipping it'
            xlo, xhi = min(xlo, xhi), max(xlo, xhi)
            ylo, yhi = min(ylo, yhi), max(ylo, yhi)
            zlo, zhi = min(zlo, zhi), max(zlo, zhi)
            bbox = np.array(((xlo, ylo, zlo), (xhi, yhi, zhi)))
        center = hu.Transform(bboxOrigin(bbox))
        points = np.array([[xlo, ylo, zlo, 1.], [xhi, ylo, zlo, 1.],
                           [xhi, yhi, zlo, 1.], [xlo, yhi, zlo, 1.]]).T
        if origin:
            center = origin.compose(center)
        Prim.__init__(self,
                      vertsFrom2D(points, zlo, zhi),
                      facesFrom2D(<int>points.shape[1]),
                      center, None,
                      **props)

cpdef Thing pointBox(pt, r = 0.02):
    return Thing(np.array([(pt[0]-r, pt[1]-r, pt[2]-r), (pt[0]+r, pt[1]+r, pt[2]+r)]),
                 Ident)

cdef class Polygon(Prim):
    def __init__(self, np.ndarray[np.float64_t, ndim=2] verts,
                 tuple zr, hu.Transform origin, **props):
        if not 'name' in props:
            props = mergeProps(props, {'name':hu.gensym("box")})
        Prim.__init__(self,
                      vertsFrom2D(verts, zr[0], zr[1]),
                      facesFrom2D(<int>verts.shape[1]),
                      origin, None,
                      **props)

#################################################################################
# Prim code
#################################################################################

cpdef np.ndarray[np.float64_t, ndim=2] vertsFrom2D(np.ndarray[np.float64_t, ndim=2] verts,
                                                   double zlo, double zhi):
    cdef:
        np.ndarray[np.float64_t, ndim=2] vertsLo, vertsHi
        int i
    vertsLo = verts.copy()
    vertsHi = verts.copy()
    for i in xrange(verts.shape[1]):
        vertsLo[2,i] = zlo
        vertsHi[2,i] = zhi
    return np.hstack([vertsLo, vertsHi])

cpdef np.ndarray[np.float64_t, ndim=2] vertsFrom2DScale(np.ndarray[np.float64_t, ndim=2] verts,
                                                        double zlo, double zhi, double scale):
    cdef:
        np.ndarray[np.float64_t, ndim=2] vertsLo, vertsHi
        int i
    vertsLo = verts.copy()
    vertsHi = verts.copy()
    for i in xrange(verts.shape[1]):
        vertsHi[0,i] = vertsLo[0,i]*scale
        vertsHi[1,i] = vertsLo[1,i]*scale
        vertsLo[2,i] = zlo
        vertsHi[2,i] = zhi
    return np.hstack([vertsLo, vertsHi])

cpdef list facesFrom2D(int n):
    cdef:
        list faces
        int i, ip1
    faces = []
    # Bottom face, order is reversed
    faces.append(np.array(range(n-1, -1, -1), dtype=np.int)) # reversed for bottom face
    faces.append(np.array(range(n,2*n,1), dtype=np.int))     # top face 
    for i in range(n):
        ip1 = (i+1)%n
        faces.append(np.array([i, ip1, n+ip1, n+i], dtype=np.int))
    return faces

# Returns an array of planes, one for each face
# The plane equations is n x + d = 0.  n;d are rows of matrix.
cpdef np.ndarray[np.float64_t, ndim=2] primPlanes(np.ndarray[np.float64_t, ndim=2] verts,
                                                 list faces):
    cdef:
        double mag, d
        np.ndarray[np.float64_t, ndim=1] n
        np.ndarray[np.float64_t, ndim=2] planes
        np.ndarray[np.int_t, ndim=1] face
    planes = np.zeros((len(faces), 4), dtype=np.float64)
    for f, face in enumerate(faces):
        n = np.cross(verts[:, face[1]][:3] - verts[:, face[0]][:3],
                     verts[:, face[2]][:3] - verts[:, face[1]][:3])
        mag = np.linalg.norm(n)
        if mag > 0:                     # otherwise, leave as all zeros
            n /= mag
            d = (np.dot(n, verts[:, face[0]][:3]) + \
                 np.dot(n, verts[:, face[1]][:3]) + \
                 np.dot(n, verts[:, face[2]][:3]))/3.0
            planes[f,:3] = n            # unit normal
            planes[f,3] = -d            # -distance from origin
    if np.any(np.dot(planes, np.average(verts, axis=1)) > 0.0):
        print 'Planes do not enclose centroid'
    return planes

# The indices of edges
cdef np.ndarray[np.int_t, ndim=2] primEdges(np.ndarray[np.float64_t, ndim=2] verts,
                                            list faces):
    cdef:
        np.ndarray[np.int_t, ndim=2] edges
        np.ndarray[np.int_t, ndim=1] face
        set done
        int f, v, v1, tail, head, i
    done = set([])
    for f in range(len(faces)):
        face = faces[f]
        k = face.shape[0]
        for v in range(k):
            v1 = (v+1)%k
            tail = face[v]; head = face[v1]
            if not ((tail, head) in done or (head, tail) in done):
                done.add((tail, head))
    edges = np.zeros((len(done), 2), dtype=np.int)
    for i, (tail, head) in enumerate(done):
        edges[i,0] = tail
        edges[i,1] = head
    return edges

cdef np.ndarray[np.int_t, ndim=1] primRings(np.ndarray[np.int_t, ndim=2] edges):
    cdef:
        dict vconn = {}
        int i, j, ind, nv, n
        list vc, entries
        np.ndarray[np.int_t, ndim=1] rings
    for i in range(edges.shape[0]):
        for (a,b) in ((0,1),(1,0)):
            if edges[i,a] in vconn:
                vconn[edges[i,a]].append(edges[i, b])
            else:
                vconn[edges[i,a]] = [edges[i,b]]
    nv = len(vconn)
    n = nv + sum([len(vc)+1 for vc in vconn.values()])
    rings = np.zeros(n, dtype=np.int)
    ind = nv
    for i in range(nv):
        rings[i] = ind
        entries = vconn[i]
        for j, e in enumerate(entries):
            rings[j + ind] = e
        ind += len(entries)
        rings[ind] = -1
        ind += 1
    return rings

cdef Prim xyPrimAux(np.ndarray[np.float64_t, ndim=2] verts,
                    tuple zr, hu.Transform origin, dict props):
    cdef:
        np.ndarray[np.float64_t, ndim=2] points
    points = convexHullVertsXY(verts)
    return Prim(vertsFrom2D(points, zr[0], zr[1]),
                facesFrom2D(<int>points.shape[1]),
                origin, None,
                **props)

cdef Prim boundingRectPrimAux(np.ndarray[np.float64_t, ndim=2] verts,
                              hu.Transform origin, dict props):
    cdef:
        np.ndarray[np.float64_t, ndim=2] mu, centered, u, v, bbox
        np.ndarray[np.float64_t, ndim=1] l
    mu = np.resize(np.mean(verts, axis=1), (4,1))
    centered = verts-mu
    (u, l, v) = np.linalg.svd(centered)
    bbox = vertsBBox(np.dot(u.T, centered), None)
    tr = np.hstack([u[:,:3], mu])
    return BoxAligned(bbox, origin).applyTrans(hu.Transform(tr),
                                               props.get('frame', 'unspecified'))

# values in d2 take precedence
cdef mergeProps(dict d1, dict d2):
    cdef dict d
    if d1:
        d = d1.copy()
        d.update(d2)
        return d
    else:
        return d2.copy()

# Returns a list of transforms for the faces.
cpdef list thingFaceFrames(np.ndarray[np.float64_t, ndim=2] planes,
                         hu.Transform origin):
    cdef:
        double d, cr
        int i, yi
        list vo
        np.ndarray[np.float64_t, ndim=1] x, y, z
        np.ndarray[np.float64_t, ndim=2] mat
    vo = [origin.matrix[:, i][:3] for i in range(4)]
    frames = []
    for f in range(planes.shape[0]):
        mat = np.eye(4)
        z = -planes[f, :3]
        d = -planes[f,3]
        for i in (1, 0, 2):
            cr = abs(np.linalg.norm(np.cross(z,vo[i])))
            yi = i
            if cr >= 0.1: break
        x = -np.cross(z, vo[yi])
        y = np.cross(z, x)
        p = vo[3] - (np.dot(z,vo[3])+d)*z
        for i, v in enumerate((x, y, z, p)):
            mat[:3,i] = v
        frames.append(origin.inverse().compose(hu.Transform(mat)))
    return frames

# Writing OFF files for a union of convex prims
def writeOff(obj, filename, scale = 1):
    prims = obj.toPrims()
    nv = sum([o.vertices().shape[1] for o in prims])
    nf = sum([len(o.faces()) for o in prims])
    ne = 0       # nv + nf - 2                    # Euler's formula...
    fl = open(filename, 'w')
    fl.write('OFF\n')
    fl.write('%d %d %d\n'%(nv, nf, ne))
    for o in prims:
        verts = o.vertices()
        for p in range(verts.shape[1]):
            fl.write('  %6.3f %6.3f %6.3f\n'%tuple([x*scale for x in verts[0:3,p]]))
    v = 0
    for o in prims:
        verts = o.vertices()
        faces = o.faces()
        for f in range(len(faces)):
            face = faces[f]
            fl.write('  %d'%face.shape[0])
            for k in range(face.shape[0]):
                fl.write(' %d'%(v+face[k]))
            fl.write('\n')
        v += verts.shape[1]
    fl.close()

# Reading OFF files
def readOff(filename, name='offObj', scale=1.0, trans=None):
    fl = open(filename)
    assert fl.readline().split()[0] == 'OFF', 'Not OFF file in readOff'
    (nv, nf, ne) = [int(x) for x in fl.readline().split()]
    vl = []
    for v in range(nv):
        vl.append(np.array([scale*float(x) for x in fl.readline().split()]+[1.0]))
    verts = np.vstack(vl).T
    if trans:
        verts = np.dot(trans.matrix, verts)
    if nf == 0:
        return verts
    faces = []
    for f in range(nf):
        faces.append(np.array([int(x) for x in fl.readline().split()][1:]))
    return Prim(verts, faces, hu.Pose(0,0,0,0), None, name=name)

# Reading WRL files
def readWrl(filename, name='wrlObj', scale=1.0, color='black'):

    def readOneObj():
        vl = []
        while True:
            line = fl.readline()
            split = line.split(',')
            if len(split) != 2:
                break
            split = split[0].split()
            if len(split) == 3:
                vl.append(np.array([scale*float(x) for x in split]+[1.0]))
            else:
                break
        print '    verts', len(vl), 
        verts = np.vstack(vl).T
        while line.split()[0] != 'coordIndex':
            line = fl.readline()
        line = fl.readline()
        faces = []
        while True:
            line = fl.readline()
            split = line.split(',')
            if len(split) > 3:
                faces.append(np.array([int(x) for x in split[:3]]))
            else:
                break
        print 'faces', len(faces)
        return Prim(verts, faces, hu.Pose(0,0,0,0), None,
                    name=name+str(len(prims)), color=color)

    fl = open(filename)
    assert fl.readline().split()[0] == '#VRML', 'Not VRML file?'
    prims = []
    while True:
        line = fl.readline()
        if not line: break
        split = line.split()
        if not split or split[0] != 'point':
            continue
        else:
            print 'Object', len(prims)
            prims.append(readOneObj())
    # Have one "part" so that shadows are simpler
    part = Shape(prims, None, name=name+'_part', color=color)
    return Shape([part], None, name=name, color=color)

def drawFrame(tr, window = 'W', r = 0.01, length=0.25):
    rayx = BoxAligned(np.array([(-r, -r, -r), (length/3., r, r)]), None)
    rayx.applyTrans(tr).draw(window, 'red')
    rayy = BoxAligned(np.array([(-r, -r, -r), (r, length/3., r)]), None)
    rayy.applyTrans(tr).draw(window, 'green')
    rayz = BoxAligned(np.array([(-r, -r, -r), (r, r, length/3.)]), None)
    rayz.applyTrans(tr).draw(window, 'blue')


#################################################################################
# Convex Hull operations -- uses Numpy arrays
#################################################################################

cpdef np.ndarray convexHullVertsXY(np.ndarray[np.float64_t, ndim=2] verts):
    """
    Compute vertices of convex hull of an array of points projected on xy plane.
    The returned points are sorted by angle, suitable for using as polygon.
    """
    cdef:
        np.ndarray[np.float64_t, ndim=2] xyVerts, chVerts, vecs
        np.ndarray[np.float64_t, ndim=1] angles
        np.ndarray[np.int_t, ndim=1] indices
        list ch
    xyVerts = verts[0:2,:]              # the xy coordinates (2xn)
    # Note the Transpose, points are rows in the input
    ch = ConvexHull(xyVerts.T).simplices.flatten().tolist()
    indices = np.array(sorted(list(set(ch))))  # unique indices of ch vertices
    chVerts = xyVerts[:, indices]      # the xy verts of ch (2xm)
    vecs = chVerts - np.resize(np.mean(chVerts, axis=1), (2,1))
    angles = np.arctan2(vecs[1,:], vecs[0,:])
    return verts[:, indices[np.argsort(angles)]]

cpdef Prim convexHullPrim(np.ndarray[np.float64_t, ndim=2] verts,
                                 hu.Transform origin):
    """
    Return a Prim that is the convex hull of the input verts.
    """
    cdef np.ndarray[np.int_t, ndim=1] indices
    ch = ConvexHull(verts[:3,:].T)
    # unique indices of ch vertices
    indices = np.array(sorted(list(set(ch.simplices.flatten().tolist()))))
    # create the prim from verts and faces
    pr = Prim(verts[:, indices], chFaces(ch, indices), origin, None)
    # pr.draw('W')
    # raw_input('Next')
    return pr

cdef list chFaces(ch, np.ndarray indices, merge = True):
    """
    Unfortunately ConvexHull triangulates faces, so we have to undo it...
    """
    cdef:
        int fi, i
        bool found
        dict mapping
        list group, mergedFaces, remaining, normSimp, groups
    if merge:
        # This cleans up the faces, but it really slows things down a lot.
        # Normalize the faces, a list of lists
        normSimp = normSimplices(ch)
        # Group simplices on each face
        groups = groupSimplices(ch)
        # Merge the normed simplices; this is reduce with mergedFace
        mergedFaces = []
        for group in groups:
            face = normSimp[group[0]]
            remaining = group[1:]
            while remaining:
                found = False
                for fi in remaining:
                    if adjacent(face, normSimp[fi]):
                        face = mergedFace(face, normSimp[fi])
                        remaining.remove(fi)
                        found = True
                        break
                if not found:
                    raise Exception, 'Could not find adjacent face'
            mergedFaces.append(face)
        # The simplex specifies indices into the full point set.
    else:
        mergedFaces = ch.simplices.tolist()
    mapping = dict([(indices[i], i) for i in range(indices.shape[0])])
    faces = [np.array([mapping[fi] for fi in face]) for face in mergedFaces]
    return faces

cdef bool adjacent(s1, s2):             # two faces share an edge
    return len(set(s1).intersection(set(s2))) == 2 

cdef list normSimplices(ch):
    """
    Sometimes the simplices don't go around CCW, this reverses them if
    they don't.  Stupid wasteful...
    """
    cdef:
        int f
        list normSimp
        np.ndarray[np.int32_t, ndim=2] simplices
        np.ndarray[np.float64_t, ndim=2] eqns, points
        np.ndarray[np.float64_t, ndim=1] normal
    points = ch.points
    simplices = ch.simplices
    eqns = ch.equations
    normSimp = []
    for f in range(simplices.shape[0]):
        normal = np.cross(points[simplices[f][1]] - points[simplices[f][0]],
                          points[simplices[f][2]] - points[simplices[f][1]])
        if np.dot(normal, eqns[f][:3]) > 0:
            normSimp.append(simplices[f].tolist())
        else:
            normSimp.append(simplices[f].tolist()[::-1]) # reverse
    return normSimp

cdef list groupSimplices(ch):
    """
    Find all the simplices that share a face.  Only works for convex
    solids, that is, we assume those simplices are adjacent.
    """
    cdef:
        list groups, face
        set done
        int ne
        np.ndarray[np.int32_t, ndim=2] simplices
        np.ndarray[np.float64_t, ndim=2] eqns
    groups = []
    simplices = ch.simplices
    eqns = ch.equations
    ne = eqns.shape[0]
    face = []                           # current "face"
    done = set([])                      # faces already done
    for e in range(ne):                 # loop over eqns
        if e in done: continue
        face = [e]
        done.add(e)
        for e1 in range(e+1, ne):       # loop for remaining eqns
            if e1 in done: continue
            if np.all(np.abs(eqns[e] - eqns[e1]) < 1e-6):
                # close enough
                face.append(e1)
                done.add(e1)
        # remember this "face"
        groups.append(face)
    # all the faces.
    return groups

cdef list mergedFace(list face, list simplex):
    """
    Insert the new vertex from the simplex into the growing face.
    """
    cdef:
        set setFace, setSimp, diff
        list newFace
        int n, i
    setFace = set(face)
    setSimp = set(simplex)
    common = setSimp.intersection(setFace)
    diff = setSimp.difference(setFace)
    assert len(diff)==1 and len(common)==2, 'mergedFace - Inconsistent'
    newFace = face[:]                   # copy
    n = len(face)
    for i in range(n):
        if newFace[i] in common and newFace[(i+1)%n] in common:
            newFace[i+1:] = [diff.pop()] + newFace[i+1:]
            break
    return newFace

