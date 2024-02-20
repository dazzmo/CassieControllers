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

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int cassie_RightFootBack(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cassie_RightFootBack_alloc_mem(void);
int cassie_RightFootBack_init_mem(int mem);
void cassie_RightFootBack_free_mem(int mem);
int cassie_RightFootBack_checkout(void);
void cassie_RightFootBack_release(int mem);
void cassie_RightFootBack_incref(void);
void cassie_RightFootBack_decref(void);
casadi_int cassie_RightFootBack_n_in(void);
casadi_int cassie_RightFootBack_n_out(void);
casadi_real cassie_RightFootBack_default_in(casadi_int i);
const char* cassie_RightFootBack_name_in(casadi_int i);
const char* cassie_RightFootBack_name_out(casadi_int i);
const casadi_int* cassie_RightFootBack_sparsity_in(casadi_int i);
const casadi_int* cassie_RightFootBack_sparsity_out(casadi_int i);
int cassie_RightFootBack_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cassie_RightFootBack_SZ_ARG 2
#define cassie_RightFootBack_SZ_RES 3
#define cassie_RightFootBack_SZ_IW 0
#define cassie_RightFootBack_SZ_W 86
#ifdef __cplusplus
} /* extern "C" */
#endif
