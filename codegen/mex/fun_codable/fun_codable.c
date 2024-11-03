/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fun_codable.c
 *
 * Code generation for function 'fun_codable'
 *
 */

/* Include files */
#include "fun_codable.h"
#include "rt_nonfinite.h"
#include "F.h"
#include "mwmathutil.h"

/* Function Definitions */
void fun_codable(const emlrtStack *sp, real_T a, real_T *area_sol,
                 real_T center_sol[2])
{
  static const int8_T iv[3] = {0, 3, 9};
  casadi_int iw[4];
  const casadi_real *arg[15];
  casadi_real w[69044];
  casadi_real *res[15];
  real_T b_a;
  real_T p_value;
  int32_T low_i;
  (void)sp;
  /*  Any pre-processing using pure Matlab operations can go here */
  p_value = rtNaN;
  if ((!muDoubleScalarIsNaN(a)) && (!(a > 2.0)) && (!(a < 0.0))) {
    low_i = 1;
    if (a >= 1.0) {
      low_i = 2;
    }
    p_value = a - ((real_T)low_i - 1.0);
    if (p_value == 0.0) {
      p_value = iv[low_i - 1];
    } else if (p_value == 1.0) {
      p_value = iv[low_i];
    } else {
      int8_T p_value_tmp;
      p_value_tmp = iv[low_i - 1];
      if (p_value_tmp == iv[low_i]) {
        p_value = 3.0;
      } else {
        p_value =
            (1.0 - p_value) * (real_T)p_value_tmp + p_value * (real_T)iv[low_i];
      }
    }
  }
  /*  Make sure data-types and sizes are known */
  /*  Anything CasADi related goes here */
  /*  This gets executed when Matlab Coder is parsing the file */
  /*  Hooks up Matlab Coder with CasADi generated C code */
  /*  Connect .c and .h file */
  /*  Set link and include path */
  /*  Link with IPOPT */
  /*  Setting up working space */
  low_i = F_checkout();
  /*  Call the generated CasADi code */
  F_unrolled(&p_value, &b_a, &center_sol[0], &arg[0], &res[0], &iw[0], &w[0],
             low_i);
  /*  % Adapt to as many inputs arguments as your CasADi Function has */
  /*  % Adapt to as many outputs as your CasADi Function has */
  /*   */
  F_release(low_i);
  /*  Any post-processing using pure Matlab operations can go here */
  *area_sol = 3.1415926535897931 * (b_a * b_a);
}

/* End of code generation (fun_codable.c) */
