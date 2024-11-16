/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_fun_codable_info.c
 *
 * Code generation for function 'fun_codable'
 *
 */

/* Include files */
#include "_coder_fun_codable_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

/* Function Declarations */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void);

/* Function Definitions */
static const mxArray *c_emlrtMexFcnResolvedFunctionsI(void)
{
  const mxArray *nameCaptureInfo;
  const char_T *data[6] = {
      "789ced57cb6ed340149dd022b1a1840d3ba42ebaa38944a14262834cd286141a47240195"
      "4e691c7b525b9d47349ea42e2b243e0058f2072c91f801967c036c58"
      "2076fc034e3c9387e9e028052345b90bdfb93e9e7bce1c275732c8947733008015104570"
      "35ca97659d95f902988c389e91792956abb8089627f629fcb5cc36a3",
      "0205222aa845d070a7c388472d2aeaa71d0438f219ee216780b43d8cea1e41b5f1a2d2af"
      "c8f618342cfa507f5d70917d5ceb12c05d7fa4108f17433f3e68cebb"
      "3ca51f0f357e6463f87ef9d1d6c11ab104b65a9c31b1060563b8c502880886d86bc10883"
      "1d864fdb5d0abdd02fdeb99927999ae01e3daab35d245ce6948b13fa",
      "df9c53ff7a827e85dbcc413c3f10452d1c2d42a99640721da9f3b768e8fb98bee68cfae2"
      "a1d3a742f17d9a914ff57f9ec0a7f0f07d56ce7e9fbe6b71e4c08161"
      "f27a4326659f5c45fec1dffccb9364ffae4c799e781e3d7f69907fbefa3280d2e27b79ef"
      "e3bb34f954fc2fbe40d36fdadfe3350d5f36869b3db3e46c06ec09dd",
      "3136bc9e7babf5b85b2f8d745413789274004d9d56fff79afdd3fa58d2f4cfc6f073cc69"
      "b93a3c61fc78319f63f5623e9f7d9ec57c8e62319fffcc93a40368ea"
      "b4fa3735fbffd51c9977becf33f2a9fe76029fc2f71b0785bbb0e123ee43e15aa20d4d8a"
      "8adceb2158647697202a7c68d40c681456cd8ef08885570be1b71c67",
      "18566bd55c08e5e4fd9cbc9f330a70fb30fce06b7b47f970024f9caba9d1fdb7fedfdfd6"
      "bfa63a2fdf5efffe234d3e15f33e2f1fec9d6c6c3e7d669bc5ea5e63"
      "e74ee585b85dbc3f07f3f2171c5230e0",
      ""};
  nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&data[0], 4496U, &nameCaptureInfo);
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
  xInputs = emlrtCreateLogicalMatrix(1, 1);
  emlrtSetField(xEntryPoints, 0, "QualifiedName",
                emlrtMxCreateString("fun_codable"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "ResolvedFilePath",
                emlrtMxCreateString(
                    "C:\\Users\\thatf\\OneDrive\\Documents\\ASA\\AC Optimal "
                    "Control\\PSP-ASA-Optimal-Control-AC\\fun_codable.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(739557.4366319445));
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
                emlrtMxCreateString("OcG6XTk1Gf1J3gyYjDBXtH"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

/* End of code generation (_coder_fun_codable_info.c) */
