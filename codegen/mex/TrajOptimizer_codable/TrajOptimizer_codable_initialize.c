/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * TrajOptimizer_codable_initialize.c
 *
 * Code generation for function 'TrajOptimizer_codable_initialize'
 *
 */

/* Include files */
#include "TrajOptimizer_codable_initialize.h"
#include "TrajOptimizer_codable_data.h"
#include "_coder_TrajOptimizer_codable_mex.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static void TrajOptimizer_codable_once(void);

/* Function Definitions */
static void TrajOptimizer_codable_once(void)
{
  mex_InitInfAndNan();
}

void TrajOptimizer_codable_initialize(void)
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
    TrajOptimizer_codable_once();
  }
}

/* End of code generation (TrajOptimizer_codable_initialize.c) */
