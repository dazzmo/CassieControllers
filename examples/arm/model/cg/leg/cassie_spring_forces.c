/* This file was automatically generated by CasADi 3.6.3+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) cassie_spring_forces_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};

/* cassie_spring_forces:(i0[8],i1[8])->(o0[8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  if (res[0]!=0) res[0][3]=a0;
  a1=2300.;
  a2=arg[0]? arg[0][4] : 0;
  a1=(a1*a2);
  a2=4.5999999999999996e+00;
  a3=arg[1]? arg[1][4] : 0;
  a2=(a2*a3);
  a1=(a1+a2);
  if (res[0]!=0) res[0][4]=a1;
  if (res[0]!=0) res[0][5]=a0;
  a1=2000.;
  a2=arg[0]? arg[0][6] : 0;
  a1=(a1*a2);
  a2=4.;
  a3=arg[1]? arg[1][6] : 0;
  a2=(a2*a3);
  a1=(a1+a2);
  if (res[0]!=0) res[0][6]=a1;
  if (res[0]!=0) res[0][7]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_spring_forces(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_spring_forces_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_spring_forces_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_spring_forces_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_spring_forces_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_spring_forces_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_spring_forces_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_spring_forces_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_spring_forces_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_spring_forces_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real cassie_spring_forces_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_spring_forces_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_spring_forces_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_spring_forces_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_spring_forces_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_spring_forces_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
