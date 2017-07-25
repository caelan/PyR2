cimport geometry.hu as hu
import math
import numpy as np
from cpython cimport bool
from random import uniform

PI2 = 2*math.pi

cdef class Range:
    def __init__(self):
        self.kind = ''

# Range classes
cdef class realRange(Range):
    def __init__(self, float lo, float hi):
        self.kind = 'realRange'
        self.lo = lo
        self.hi = hi
    cpdef bool inside(self, float value):
        return self.lo <= value < self.hi
    cpdef float width(self):
        return self.hi - self.lo
    cpdef bool overlaps(self, realRange other, float minOverlap = 0.0):
        cdef float overlap = min(self.hi, other.hi) - max(self.lo, other.lo)
        return overlap >= minOverlap
    cpdef bool included(self, realRange other):
        return other.inside(self.lo) and other.inside(self.hi)
    cpdef realRange expand(self, float mag):              # mag can be pos or neg
        return realRange(self.lo-mag, self.hi+mag)
    cpdef realRange next(self, int sign, float mag):
        if sign > 0:
            return realRange(self.hi + mag, self.hi + 2*mag)
        else:
            return realRange(self.lo - 2*mag, self.lo - mag)
    cpdef realRange intersect(self, realRange other):
        return realRange(max(self.lo, other.lo), min(self.hi, other.hi))
    cpdef float middle(self):
        return 0.5*(self.lo+self.hi)
    cpdef list split(self):
        cdef float mid = self.middle()
        return [realRange(self.lo, mid), realRange(mid, self.hi)]
    cpdef realRange copy(self):
        return realRange(self.lo, self.hi)
    cpdef tuple toTuple(self):
        return (self.lo, self.hi)
    cpdef float pick(self):
        return uniform(self.lo, self.hi)
    def __richcmp__(self, realRange other, int op):
        return (self.lo, self.hi) == (other.lo, other.hi)
    def __str__(self):
        return 'R[%f,%f]'%(self.lo, self.hi)
    def __repr__(self):
        return 'R[%f,%f]'%(self.lo, self.hi)
    def __hash__(self):
        return hash(str(self))

cdef class angleRange(Range):
    def __init__(self, float lo, float hi):
        self.kind = 'angleRange'
        # Normalize the range so width <= 2pi and lo >= 0, hi < 4pi
        if hi - lo > PI2:
            self.lo = 0.
            self.hi = PI2
        while lo < 0:
            lo += PI2; hi += PI2
        while hi >= 2*PI2:
            lo -= PI2; hi -= PI2
        assert lo >= 0 and hi < 2*PI2, 'angleRange invalid angle'
        self.lo = lo
        self.hi = hi
    cpdef bool inside(self, float value):
        cdef float val = hu.fixAngle02Pi(value)
        return (self.lo <= value < self.hi) or (self.lo-PI2 <= value < self.hi-PI2)
    cpdef float width(self):
        return self.hi - self.lo
    cpdef bool overlaps(self, angleRange other, float minOverlap = 0.0):
        cdef float overlap
        overlap = min(self.hi, other.hi) - max(self.lo, other.lo)
        if overlap >= minOverlap: return True
        overlap = min(self.hi, other.hi-PI2) - max(self.lo, other.lo-PI2)
        if overlap >= minOverlap: return True
        overlap = min(self.hi, other.hi+PI2) - max(self.lo, other.lo+PI2)
        if overlap >= minOverlap: return True
        return False
    cpdef bool included(self, angleRange other):
        return other.inside(self.lo) and other.inside(self.hi)
    cpdef angleRange expand(self, float mag):              # mag can be pos or neg
        return angleRange(self.lo-mag, self.hi+mag)
    cpdef angleRange next(self, int sign, float mag):
        if sign > 0:
            return angleRange(self.hi + mag, self.hi + 2*mag)
        else:
            return angleRange(self.lo - 2*mag, self.lo - mag)
    cpdef float middle(self):
        return 0.5*(self.lo+self.hi)
    cpdef list split(self):
        cdef float mid = self.middle()
        if mid >= PI2:
            return [angleRange(self.lo, mid), angleRange(mid-PI2, self.hi-PI2)]
        else:
            return [angleRange(self.lo, mid), angleRange(mid, self.hi)]
    cpdef float pick(self):
        return uniform(self.lo, self.hi)
    cpdef angleRange copy(self):
        return angleRange(self.lo, self.hi)
    cpdef tuple toTuple(self):
        return (self.lo, self.hi)
    def __richcmp__(self, realRange other, int op):
        return (self.lo, self.hi) == (other.lo, other.hi)
    def __str__(self):
        return 'A[%f,%f]'%(self.lo, self.hi)
    def __repr__(self):
        return 'A[%f,%f]'%(self.lo, self.hi)
    def __hash__(self):
        return hash(str(self))
    
cdef class discreteRange(Range):
    def __init__(self, list values):
        self.values = set(values)
    cpdef bool inside(self, int value):
        return value in self.values
    cpdef float width(self):
        return float(len(self.values))
    cpdef bool overlaps(self, discreteRange other, int minOverlap):
        return len(self.values.intersection(other.values)) > minOverlap
    cpdef bool included(self, discreteRange other):
        return self.values.issubset(other.values)
    cpdef discreteRange expand(self, mag=None):
        return self                     # no expansion defined
    cpdef discreteRange next(self, sign=None, mag=None):
        return self                     # no next defined
    cpdef int middle(self):
        return sorted(list(self.values))[int(self.values)//2]
    cpdef list split(self):
        cdef int n = len(self.values)
        if n == 1:
            return []
        elif n <= 16:
            return [discreteRange([x]) for x in self.values]
        else:
            vals = sorted(list(self.values))
            return [discreteRange(vals[ : n//2]),
                    discreteRange(vals[n//2 : ])]
    cpdef discreteRange copy(self):
        return discreteRange(self.values)
    cpdef tuple toTuple(self):
        return tuple(self.values)
    def __richcmp__(self, realRange other, int op):
        return self.values == other.values
    def __str__(self):
        return 'D[%s]'%str(self.values)
    def __repr__(self):
        return 'D[%s]'%str(self.values)
    def __hash__(self):
        return hash(str(self))
    
# nRange operations

cpdef float nRangeWidth(list nRange):
    cdef Range r
    return [r.width() for r in nRange]

cpdef bool nRangeInside(list value, list nRange):
    cdef int i
    cdef Range r
    for i in range(len(nRange)):
        r = nRange[i]
        if not r.inside(value[i]): return False
    return True

cpdef bool nRangeOverlaps(r1, r2, minOverlaps = None):
    for i in xrange(len(r1)):
        if not r1[i].overlaps(r2[i], minOverlaps[i] if minOverlaps else 0):
            return False
    return True

cpdef bool nRangeIncluded(list r1, list r2):
    cdef int i
    for i in xrange(len(r1)):
        if not r1[i].included(r2[i]): return False
    return True

cpdef np.ndarray nRangesBBox(ranges, expand = 0.0):
    cdef:
        list lo = []
        list hi = []
        int i
    for i in xrange(len(ranges)):
        lo.append(ranges[i].lo - expand)
        hi.append(ranges[i].hi + expand)
    return np.array((lo, hi))

cpdef list splits(Range r, int n):
    cdef Range lo, hi
    if n > 1:
        (lo, hi) = r.split()
        return splits(lo, n//2) + splits(hi, n//2)
    else:
        return [r]
    
