/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * TrajOptimizer_codable_types.h
 *
 * Code generation for function 'TrajOptimizer_codable'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"

/* Type Definitions */
#ifndef typedef_rtString
#define typedef_rtString
typedef struct {
  char_T Value[6];
} rtString;
#endif /* typedef_rtString */

#ifndef typedef_Vehicle
#define typedef_Vehicle
typedef struct {
  real_T m;
  real_T L;
  real_T b_I;
  real_T Len;
  real_T max_gimbal;
  real_T min_thrust;
  real_T max_thrust;
  rtString Name;
} Vehicle;
#endif /* typedef_Vehicle */

/* End of code generation (TrajOptimizer_codable_types.h) */
