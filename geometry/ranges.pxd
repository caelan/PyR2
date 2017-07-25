cimport numpy as np
from cpython cimport bool

cdef class Range:
    cdef public str kind

# Range classes
cdef class realRange(Range):
    cdef public float lo, hi
    cpdef bool inside(self, float value)
    cpdef float width(self)
    cpdef bool overlaps(self, realRange other, float minOverlap = *)
    cpdef bool included(self, realRange other)
    cpdef realRange intersect(self, realRange other)
    cpdef realRange expand(self, float mag)              # mag can be pos or neg
    cpdef realRange next(self, int sign, float mag)
    cpdef float middle(self)
    cpdef list split(self)
    cpdef realRange copy(self)
    cpdef tuple toTuple(self)
    cpdef float pick(self)

cdef class angleRange(Range):
    cdef public float lo, hi
    cpdef bool inside(self, float value)
    cpdef float width(self)
    cpdef bool overlaps(self, angleRange other, float minOverlap =*)
    cpdef bool included(self, angleRange other)
    cpdef angleRange expand(self, float mag)              # mag can be pos or neg
    cpdef angleRange next(self, int sign, float mag)
    cpdef float middle(self)
    cpdef list split(self)
    cpdef float pick(self)
    cpdef angleRange copy(self)
    cpdef tuple toTuple(self)
    
cdef class discreteRange(Range):
    cdef public list values
    cpdef bool inside(self, int value)
    cpdef float width(self)
    cpdef bool overlaps(self, discreteRange other, int minOverlap)
    cpdef bool included(self, discreteRange other)
    cpdef discreteRange expand(self, mag=*)
    cpdef discreteRange next(self, sign=*, mag=*)
    cpdef int middle(self)
    cpdef list split(self)
    cpdef discreteRange copy(self)
    cpdef tuple toTuple(self)
    
# nRange operations

cpdef float nRangeWidth(list nRange)
cpdef bool nRangeInside(list value, list nRange)
cpdef bool nRangeOverlaps(r1, r2, minOverlaps = *)
cpdef bool nRangeIncluded(list r1, list r2)
cpdef np.ndarray nRangesBBox(ranges, expand = *)
cpdef list splits(Range r, int n)

    
