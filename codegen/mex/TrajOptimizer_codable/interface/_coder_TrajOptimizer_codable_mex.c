/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_TrajOptimizer_codable_mex.c
 *
 * Code generation for function '_coder_TrajOptimizer_codable_mex'
 *
 */

/* Include files */
#include "_coder_TrajOptimizer_codable_mex.h"
#include "TrajOptimizer_codable_data.h"
#include "TrajOptimizer_codable_initialize.h"
#include "TrajOptimizer_codable_terminate.h"
#include "_coder_TrajOptimizer_codable_api.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void TrajOptimizer_codable_mexFunction(int32_T nlhs, mxArray *plhs[2],
                                       int32_T nrhs, const mxArray *prhs[4])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs[2];
  int32_T i;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 4) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 4, 4,
                        21, "TrajOptimizer_codable");
  }
  if (nlhs > 2) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 21,
                        "TrajOptimizer_codable");
  }
  /* Call the function. */
  TrajOptimizer_codable_api(prhs, nlhs, outputs);
  /* Copy over outputs to the caller. */
  if (nlhs < 1) {
    i = 1;
  } else {
    i = nlhs;
  }
  emlrtReturnArrays(i, &plhs[0], &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&TrajOptimizer_codable_atexit);
  TrajOptimizer_codable_initialize();
  TrajOptimizer_codable_mexFunction(nlhs, plhs, nrhs, prhs);
  TrajOptimizer_codable_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, "windows-1252", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_TrajOptimizer_codable_mex.c) */
