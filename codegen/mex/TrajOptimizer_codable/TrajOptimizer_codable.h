/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * TrajOptimizer_codable.h
 *
 * Code generation for function 'TrajOptimizer_codable'
 *
 */

#pragma once

/* Include files */
#include "TrajOptimizer_codable_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void TrajOptimizer_codable(const emlrtStack *sp, const real_T x_initial[6],
                           const real_T x_guess[2400],
                           const real_T u_guess[800], const Vehicle *vehicle,
                           real_T u_opt[800], real_T x_opt[2400]);

/* End of code generation (TrajOptimizer_codable.h) */
