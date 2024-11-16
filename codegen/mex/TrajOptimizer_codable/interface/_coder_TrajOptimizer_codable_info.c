/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_TrajOptimizer_codable_info.c
 *
 * Code generation for function 'TrajOptimizer_codable'
 *
 */

/* Include files */
#include "_coder_TrajOptimizer_codable_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5] = {
      "789cdd55cd4ec24010de1a8c2646e5e499ab87120d12a2276b2b8804685290186a6c5316"
      "5becb64dbb107c01e3cd67f2ea89bb6fe11308b4cb4f934d896811e7"
      "323bfbcdcc3733d94e01532c3300803de08bb3e3ebddc04e067a03cc4b1867427eccbc3b"
      "d80489b93882bf065ab32d0cfbd8372c15c14964cb4686a55ab8f6e4",
      "40e042cf367bb03546da86096b0682d2ac511959283f034d8c11343af33ad41ea52e02ae"
      "ee4d2b34678dc93c144abf89887984253c8fb0dfb27c24ff1695cf47"
      "6ea06e68265c597f836ff291fc46041fc19bfc1d7f26d73de87a32d655dc96ab16145ca3"
      "0765c1d6ba085ad893398993393e5575b0815433c50fdfa06b9bb228",
      "89ec1062837b36b867395e0ec6e791431a85fa5328f5ef2fd85f584ffdb7c7faedf08389"
      "934f79793f8f938fc8aaf8fa947c8bbecf030a5f3284e78f4f4a05ad"
      "e46058ae34b2e8c82966c4eec5b40e318227aa0e40b1e3caaf50e2ffdade5c17be65f766"
      "27828fe0cdfaafeccd9aab7632829dbf1ffee7dbc6431aa978be3f85",
      "52ff4f7de73c1befde1c7c969ee3e423f2dff7664e578542fdf4123672ae90cd5d571fcb"
      "b7c2d5faefcd2f4d779d6f",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 2952U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties(void)
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9] = {"Version",
                                    "ResolvedFunctions",
                                    "Checksum",
                                    "EntryPoints",
                                    "CoverageInfo",
                                    "IsPolymorphic",
                                    "PropertyList",
                                    "UUID",
                                    "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8] = {
      "QualifiedName",    "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "ResolvedFilePath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("TrajOptimizer_codable"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "C:\\Users\\thatf\\OneDrive\\Documents\\ASA\\AC Optimal "
                    "Control\\PSP-ASA-Optimal-Control-AC\\3DoF\\TrajOptimizer_"
                    "codable.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739558.51600694447));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("24.2.0.2740171 (R2024b) Update 1"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)c_emlrtMexFcnResolvedFunctionsI());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("KjRiLLMbrZxBVny1UzXFJD"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_TrajOptimizer_codable_info.c) */
