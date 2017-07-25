
cdef extern from "IK/ikFast.h":
   cdef int ikRight(const double* eetrans, const double* eerot, const double* pfree, const int max_sol, double* solutions)
   cdef int ikLeft(const double* eetrans, const double* eerot, const double* pfree, const int max_sol, double* solutions)
  