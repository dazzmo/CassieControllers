#ifndef INCLUDE_CONTROLLERS_CASADI_INTERFACE_HPP
#define INCLUDE_CONTROLLERS_CASADI_INTERFACE_HPP

typedef int (*f_cg)(const double** arg, double** res);
typedef int (*f_casadi_cg)(const double** arg, double** res, long long int* iw, double* w, int mem);

#endif /* INCLUDE_CONTROLLERS_CASADI_INTERFACE_HPP */
