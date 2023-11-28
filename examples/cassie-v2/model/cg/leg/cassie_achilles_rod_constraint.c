/* This file was automatically generated by CasADi 3.6.3.
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
  #define CASADI_PREFIX(ID) cassie_achilles_rod_constraint_ ## ID
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
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

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
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[19] = {1, 8, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0, 0};
static const casadi_int casadi_s3[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};

/* cassie_achilles_rod_constraint:(i0[8],i1[8])->(o0,o1[1x8],o2,o3[8x8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0;
  a0=-2.5120144000000000e-01;
  if (res[0]!=0) res[0][0]=a0;
  a0=0.;
  if (res[1]!=0) res[1][0]=a0;
  if (res[1]!=0) res[1][1]=a0;
  if (res[1]!=0) res[1][2]=a0;
  if (res[1]!=0) res[1][3]=a0;
  if (res[1]!=0) res[1][4]=a0;
  if (res[1]!=0) res[1][5]=a0;
  if (res[1]!=0) res[1][6]=a0;
  if (res[1]!=0) res[1][7]=a0;
  if (res[2]!=0) res[2][0]=a0;
  if (res[3]!=0) res[3][0]=a0;
  if (res[3]!=0) res[3][1]=a0;
  if (res[3]!=0) res[3][2]=a0;
  if (res[3]!=0) res[3][3]=a0;
  if (res[3]!=0) res[3][4]=a0;
  if (res[3]!=0) res[3][5]=a0;
  if (res[3]!=0) res[3][6]=a0;
  if (res[3]!=0) res[3][7]=a0;
  if (res[3]!=0) res[3][8]=a0;
  if (res[3]!=0) res[3][9]=a0;
  if (res[3]!=0) res[3][10]=a0;
  if (res[3]!=0) res[3][11]=a0;
  if (res[3]!=0) res[3][12]=a0;
  if (res[3]!=0) res[3][13]=a0;
  if (res[3]!=0) res[3][14]=a0;
  if (res[3]!=0) res[3][15]=a0;
  if (res[3]!=0) res[3][16]=a0;
  if (res[3]!=0) res[3][17]=a0;
  if (res[3]!=0) res[3][18]=a0;
  if (res[3]!=0) res[3][19]=a0;
  if (res[3]!=0) res[3][20]=a0;
  if (res[3]!=0) res[3][21]=a0;
  if (res[3]!=0) res[3][22]=a0;
  if (res[3]!=0) res[3][23]=a0;
  if (res[3]!=0) res[3][24]=a0;
  if (res[3]!=0) res[3][25]=a0;
  if (res[3]!=0) res[3][26]=a0;
  if (res[3]!=0) res[3][27]=a0;
  if (res[3]!=0) res[3][28]=a0;
  if (res[3]!=0) res[3][29]=a0;
  if (res[3]!=0) res[3][30]=a0;
  if (res[3]!=0) res[3][31]=a0;
  if (res[3]!=0) res[3][32]=a0;
  if (res[3]!=0) res[3][33]=a0;
  if (res[3]!=0) res[3][34]=a0;
  if (res[3]!=0) res[3][35]=a0;
  if (res[3]!=0) res[3][36]=a0;
  if (res[3]!=0) res[3][37]=a0;
  if (res[3]!=0) res[3][38]=a0;
  if (res[3]!=0) res[3][39]=a0;
  if (res[3]!=0) res[3][40]=a0;
  if (res[3]!=0) res[3][41]=a0;
  if (res[3]!=0) res[3][42]=a0;
  if (res[3]!=0) res[3][43]=a0;
  if (res[3]!=0) res[3][44]=a0;
  if (res[3]!=0) res[3][45]=a0;
  if (res[3]!=0) res[3][46]=a0;
  if (res[3]!=0) res[3][47]=a0;
  if (res[3]!=0) res[3][48]=a0;
  if (res[3]!=0) res[3][49]=a0;
  if (res[3]!=0) res[3][50]=a0;
  if (res[3]!=0) res[3][51]=a0;
  if (res[3]!=0) res[3][52]=a0;
  if (res[3]!=0) res[3][53]=a0;
  if (res[3]!=0) res[3][54]=a0;
  if (res[3]!=0) res[3][55]=a0;
  if (res[3]!=0) res[3][56]=a0;
  if (res[3]!=0) res[3][57]=a0;
  if (res[3]!=0) res[3][58]=a0;
  if (res[3]!=0) res[3][59]=a0;
  if (res[3]!=0) res[3][60]=a0;
  if (res[3]!=0) res[3][61]=a0;
  if (res[3]!=0) res[3][62]=a0;
  if (res[3]!=0) res[3][63]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_achilles_rod_constraint(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_achilles_rod_constraint_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_achilles_rod_constraint_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_achilles_rod_constraint_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_achilles_rod_constraint_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_achilles_rod_constraint_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_achilles_rod_constraint_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_achilles_rod_constraint_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_achilles_rod_constraint_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int cassie_achilles_rod_constraint_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real cassie_achilles_rod_constraint_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_achilles_rod_constraint_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_achilles_rod_constraint_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_achilles_rod_constraint_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_achilles_rod_constraint_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s2;
    case 2: return casadi_s1;
    case 3: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_achilles_rod_constraint_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 4;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif