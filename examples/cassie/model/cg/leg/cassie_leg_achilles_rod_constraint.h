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

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int cassie_leg_achilles_rod_constraint(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cassie_leg_achilles_rod_constraint_alloc_mem(void);
int cassie_leg_achilles_rod_constraint_init_mem(int mem);
void cassie_leg_achilles_rod_constraint_free_mem(int mem);
int cassie_leg_achilles_rod_constraint_checkout(void);
void cassie_leg_achilles_rod_constraint_release(int mem);
void cassie_leg_achilles_rod_constraint_incref(void);
void cassie_leg_achilles_rod_constraint_decref(void);
casadi_int cassie_leg_achilles_rod_constraint_n_in(void);
casadi_int cassie_leg_achilles_rod_constraint_n_out(void);
casadi_real cassie_leg_achilles_rod_constraint_default_in(casadi_int i);
const char* cassie_leg_achilles_rod_constraint_name_in(casadi_int i);
const char* cassie_leg_achilles_rod_constraint_name_out(casadi_int i);
const casadi_int* cassie_leg_achilles_rod_constraint_sparsity_in(casadi_int i);
const casadi_int* cassie_leg_achilles_rod_constraint_sparsity_out(casadi_int i);
int cassie_leg_achilles_rod_constraint_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cassie_leg_achilles_rod_constraint_SZ_ARG 2
#define cassie_leg_achilles_rod_constraint_SZ_RES 3
#define cassie_leg_achilles_rod_constraint_SZ_IW 0
#define cassie_leg_achilles_rod_constraint_SZ_W 238
#ifdef __cplusplus
} /* extern "C" */
#endif
