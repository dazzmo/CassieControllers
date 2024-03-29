/* This file was automatically generated by CasADi 3.6.4.
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
  #define CASADI_PREFIX(ID) cassie_actuation_map_ ## ID
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

static const casadi_int casadi_s0[27] = {23, 1, 0, 23, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
static const casadi_int casadi_s1[233] = {22, 10, 0, 22, 44, 66, 88, 110, 132, 154, 176, 198, 220, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};

/* cassie_actuation_map:(i0[23])->(o0[22x10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=0.;
  if (res[0]!=0) res[0][0]=a0;
  if (res[0]!=0) res[0][1]=a0;
  if (res[0]!=0) res[0][2]=a0;
  if (res[0]!=0) res[0][3]=a0;
  if (res[0]!=0) res[0][4]=a0;
  if (res[0]!=0) res[0][5]=a0;
  a1=25.;
  if (res[0]!=0) res[0][6]=a1;
  if (res[0]!=0) res[0][7]=a0;
  if (res[0]!=0) res[0][8]=a0;
  if (res[0]!=0) res[0][9]=a0;
  if (res[0]!=0) res[0][10]=a0;
  if (res[0]!=0) res[0][11]=a0;
  if (res[0]!=0) res[0][12]=a0;
  if (res[0]!=0) res[0][13]=a0;
  if (res[0]!=0) res[0][14]=a0;
  if (res[0]!=0) res[0][15]=a0;
  if (res[0]!=0) res[0][16]=a0;
  if (res[0]!=0) res[0][17]=a0;
  if (res[0]!=0) res[0][18]=a0;
  if (res[0]!=0) res[0][19]=a0;
  if (res[0]!=0) res[0][20]=a0;
  if (res[0]!=0) res[0][21]=a0;
  if (res[0]!=0) res[0][22]=a0;
  if (res[0]!=0) res[0][23]=a0;
  if (res[0]!=0) res[0][24]=a0;
  if (res[0]!=0) res[0][25]=a0;
  if (res[0]!=0) res[0][26]=a0;
  if (res[0]!=0) res[0][27]=a0;
  if (res[0]!=0) res[0][28]=a0;
  if (res[0]!=0) res[0][29]=a1;
  if (res[0]!=0) res[0][30]=a0;
  if (res[0]!=0) res[0][31]=a0;
  if (res[0]!=0) res[0][32]=a0;
  if (res[0]!=0) res[0][33]=a0;
  if (res[0]!=0) res[0][34]=a0;
  if (res[0]!=0) res[0][35]=a0;
  if (res[0]!=0) res[0][36]=a0;
  if (res[0]!=0) res[0][37]=a0;
  if (res[0]!=0) res[0][38]=a0;
  if (res[0]!=0) res[0][39]=a0;
  if (res[0]!=0) res[0][40]=a0;
  if (res[0]!=0) res[0][41]=a0;
  if (res[0]!=0) res[0][42]=a0;
  if (res[0]!=0) res[0][43]=a0;
  if (res[0]!=0) res[0][44]=a0;
  if (res[0]!=0) res[0][45]=a0;
  if (res[0]!=0) res[0][46]=a0;
  if (res[0]!=0) res[0][47]=a0;
  if (res[0]!=0) res[0][48]=a0;
  if (res[0]!=0) res[0][49]=a0;
  if (res[0]!=0) res[0][50]=a0;
  if (res[0]!=0) res[0][51]=a0;
  a2=16.;
  if (res[0]!=0) res[0][52]=a2;
  if (res[0]!=0) res[0][53]=a0;
  if (res[0]!=0) res[0][54]=a0;
  if (res[0]!=0) res[0][55]=a0;
  if (res[0]!=0) res[0][56]=a0;
  if (res[0]!=0) res[0][57]=a0;
  if (res[0]!=0) res[0][58]=a0;
  if (res[0]!=0) res[0][59]=a0;
  if (res[0]!=0) res[0][60]=a0;
  if (res[0]!=0) res[0][61]=a0;
  if (res[0]!=0) res[0][62]=a0;
  if (res[0]!=0) res[0][63]=a0;
  if (res[0]!=0) res[0][64]=a0;
  if (res[0]!=0) res[0][65]=a0;
  if (res[0]!=0) res[0][66]=a0;
  if (res[0]!=0) res[0][67]=a0;
  if (res[0]!=0) res[0][68]=a0;
  if (res[0]!=0) res[0][69]=a0;
  if (res[0]!=0) res[0][70]=a0;
  if (res[0]!=0) res[0][71]=a0;
  if (res[0]!=0) res[0][72]=a0;
  if (res[0]!=0) res[0][73]=a0;
  if (res[0]!=0) res[0][74]=a0;
  if (res[0]!=0) res[0][75]=a2;
  if (res[0]!=0) res[0][76]=a0;
  if (res[0]!=0) res[0][77]=a0;
  if (res[0]!=0) res[0][78]=a0;
  if (res[0]!=0) res[0][79]=a0;
  if (res[0]!=0) res[0][80]=a0;
  if (res[0]!=0) res[0][81]=a0;
  if (res[0]!=0) res[0][82]=a0;
  if (res[0]!=0) res[0][83]=a0;
  if (res[0]!=0) res[0][84]=a0;
  if (res[0]!=0) res[0][85]=a0;
  if (res[0]!=0) res[0][86]=a0;
  if (res[0]!=0) res[0][87]=a0;
  if (res[0]!=0) res[0][88]=a0;
  if (res[0]!=0) res[0][89]=a0;
  if (res[0]!=0) res[0][90]=a0;
  if (res[0]!=0) res[0][91]=a0;
  if (res[0]!=0) res[0][92]=a0;
  if (res[0]!=0) res[0][93]=a0;
  if (res[0]!=0) res[0][94]=a0;
  if (res[0]!=0) res[0][95]=a0;
  if (res[0]!=0) res[0][96]=a0;
  if (res[0]!=0) res[0][97]=a0;
  if (res[0]!=0) res[0][98]=a0;
  if (res[0]!=0) res[0][99]=a0;
  if (res[0]!=0) res[0][100]=a0;
  a3=50.;
  if (res[0]!=0) res[0][101]=a3;
  if (res[0]!=0) res[0][102]=a0;
  if (res[0]!=0) res[0][103]=a0;
  if (res[0]!=0) res[0][104]=a0;
  if (res[0]!=0) res[0][105]=a0;
  if (res[0]!=0) res[0][106]=a0;
  if (res[0]!=0) res[0][107]=a0;
  if (res[0]!=0) res[0][108]=a0;
  if (res[0]!=0) res[0][109]=a0;
  if (res[0]!=0) res[0][110]=a0;
  if (res[0]!=0) res[0][111]=a0;
  if (res[0]!=0) res[0][112]=a0;
  if (res[0]!=0) res[0][113]=a0;
  if (res[0]!=0) res[0][114]=a0;
  if (res[0]!=0) res[0][115]=a0;
  if (res[0]!=0) res[0][116]=a0;
  if (res[0]!=0) res[0][117]=a0;
  if (res[0]!=0) res[0][118]=a0;
  if (res[0]!=0) res[0][119]=a0;
  if (res[0]!=0) res[0][120]=a0;
  if (res[0]!=0) res[0][121]=a0;
  if (res[0]!=0) res[0][122]=a0;
  if (res[0]!=0) res[0][123]=a0;
  if (res[0]!=0) res[0][124]=a1;
  if (res[0]!=0) res[0][125]=a0;
  if (res[0]!=0) res[0][126]=a0;
  if (res[0]!=0) res[0][127]=a0;
  if (res[0]!=0) res[0][128]=a0;
  if (res[0]!=0) res[0][129]=a0;
  if (res[0]!=0) res[0][130]=a0;
  if (res[0]!=0) res[0][131]=a0;
  if (res[0]!=0) res[0][132]=a0;
  if (res[0]!=0) res[0][133]=a0;
  if (res[0]!=0) res[0][134]=a0;
  if (res[0]!=0) res[0][135]=a0;
  if (res[0]!=0) res[0][136]=a0;
  if (res[0]!=0) res[0][137]=a0;
  if (res[0]!=0) res[0][138]=a0;
  if (res[0]!=0) res[0][139]=a0;
  if (res[0]!=0) res[0][140]=a0;
  if (res[0]!=0) res[0][141]=a0;
  if (res[0]!=0) res[0][142]=a0;
  if (res[0]!=0) res[0][143]=a0;
  if (res[0]!=0) res[0][144]=a0;
  if (res[0]!=0) res[0][145]=a0;
  if (res[0]!=0) res[0][146]=a0;
  if (res[0]!=0) res[0][147]=a1;
  if (res[0]!=0) res[0][148]=a0;
  if (res[0]!=0) res[0][149]=a0;
  if (res[0]!=0) res[0][150]=a0;
  if (res[0]!=0) res[0][151]=a0;
  if (res[0]!=0) res[0][152]=a0;
  if (res[0]!=0) res[0][153]=a0;
  if (res[0]!=0) res[0][154]=a0;
  if (res[0]!=0) res[0][155]=a0;
  if (res[0]!=0) res[0][156]=a0;
  if (res[0]!=0) res[0][157]=a0;
  if (res[0]!=0) res[0][158]=a0;
  if (res[0]!=0) res[0][159]=a0;
  if (res[0]!=0) res[0][160]=a0;
  if (res[0]!=0) res[0][161]=a0;
  if (res[0]!=0) res[0][162]=a0;
  if (res[0]!=0) res[0][163]=a0;
  if (res[0]!=0) res[0][164]=a0;
  if (res[0]!=0) res[0][165]=a0;
  if (res[0]!=0) res[0][166]=a0;
  if (res[0]!=0) res[0][167]=a0;
  if (res[0]!=0) res[0][168]=a0;
  if (res[0]!=0) res[0][169]=a0;
  if (res[0]!=0) res[0][170]=a2;
  if (res[0]!=0) res[0][171]=a0;
  if (res[0]!=0) res[0][172]=a0;
  if (res[0]!=0) res[0][173]=a0;
  if (res[0]!=0) res[0][174]=a0;
  if (res[0]!=0) res[0][175]=a0;
  if (res[0]!=0) res[0][176]=a0;
  if (res[0]!=0) res[0][177]=a0;
  if (res[0]!=0) res[0][178]=a0;
  if (res[0]!=0) res[0][179]=a0;
  if (res[0]!=0) res[0][180]=a0;
  if (res[0]!=0) res[0][181]=a0;
  if (res[0]!=0) res[0][182]=a0;
  if (res[0]!=0) res[0][183]=a0;
  if (res[0]!=0) res[0][184]=a0;
  if (res[0]!=0) res[0][185]=a0;
  if (res[0]!=0) res[0][186]=a0;
  if (res[0]!=0) res[0][187]=a0;
  if (res[0]!=0) res[0][188]=a0;
  if (res[0]!=0) res[0][189]=a0;
  if (res[0]!=0) res[0][190]=a0;
  if (res[0]!=0) res[0][191]=a0;
  if (res[0]!=0) res[0][192]=a0;
  if (res[0]!=0) res[0][193]=a2;
  if (res[0]!=0) res[0][194]=a0;
  if (res[0]!=0) res[0][195]=a0;
  if (res[0]!=0) res[0][196]=a0;
  if (res[0]!=0) res[0][197]=a0;
  if (res[0]!=0) res[0][198]=a0;
  if (res[0]!=0) res[0][199]=a0;
  if (res[0]!=0) res[0][200]=a0;
  if (res[0]!=0) res[0][201]=a0;
  if (res[0]!=0) res[0][202]=a0;
  if (res[0]!=0) res[0][203]=a0;
  if (res[0]!=0) res[0][204]=a0;
  if (res[0]!=0) res[0][205]=a0;
  if (res[0]!=0) res[0][206]=a0;
  if (res[0]!=0) res[0][207]=a0;
  if (res[0]!=0) res[0][208]=a0;
  if (res[0]!=0) res[0][209]=a0;
  if (res[0]!=0) res[0][210]=a0;
  if (res[0]!=0) res[0][211]=a0;
  if (res[0]!=0) res[0][212]=a0;
  if (res[0]!=0) res[0][213]=a0;
  if (res[0]!=0) res[0][214]=a0;
  if (res[0]!=0) res[0][215]=a0;
  if (res[0]!=0) res[0][216]=a0;
  if (res[0]!=0) res[0][217]=a0;
  if (res[0]!=0) res[0][218]=a0;
  if (res[0]!=0) res[0][219]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_actuation_map(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int cassie_actuation_map_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int cassie_actuation_map_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_actuation_map_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int cassie_actuation_map_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void cassie_actuation_map_release(int mem) {
}

CASADI_SYMBOL_EXPORT void cassie_actuation_map_incref(void) {
}

CASADI_SYMBOL_EXPORT void cassie_actuation_map_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int cassie_actuation_map_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int cassie_actuation_map_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real cassie_actuation_map_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_actuation_map_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* cassie_actuation_map_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_actuation_map_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* cassie_actuation_map_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int cassie_actuation_map_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
