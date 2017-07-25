
#define IKFAST_API extern "C"
typedef double IKReal;

IKFAST_API int ikRight(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, const int max_sol, IKReal* solutions);
IKFAST_API int ikLeft(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, const int max_sol, IKReal* solutions) ;
