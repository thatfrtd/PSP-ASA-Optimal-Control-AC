/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fun_codable_initialize.c
 *
 * Code generation for function 'fun_codable_initialize'
 *
 */

/* Include files */
#include "fun_codable_initialize.h"
#include "_coder_fun_codable_mex.h"
#include "fun_codable_data.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void fun_codable_once(void);

/* Function Definitions */
static void fun_codable_once(void)
{
  mex_InitInfAndNan();
}

void fun_codable_initialize(void)
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2022b(&st);
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    fun_codable_once();
  }
}

/* End of code generation (fun_codable_initialize.c) */
