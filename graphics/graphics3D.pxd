from cpython cimport bool

cdef class Window3D:
     cdef public list capture
     cdef public bool capturing, modified
     cdef public tuple xzOffset
     cdef public window

     cpdef draw(self, thing, str color = *, float opacity = *)
     cpdef update(self)