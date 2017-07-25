
cdef extern from "gjk/gjkHPN.h":
   cdef struct Object_structure:
            int numpoints
            double (*vertices)[3]
            int *rings
   ctypedef Object_structure Object

   cdef double gjk_distance(Object *obj1, double (* tr1)[4], Object *obj2, double (* tr2)[4], double wpt1[3], double wpt2[3], void * simplex, int use_seed)
   
