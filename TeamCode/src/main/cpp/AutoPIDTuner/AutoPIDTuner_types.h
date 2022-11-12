/*
 * AutoPIDTuner_types.h
 *
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 *
 * Code generation for model "AutoPIDTuner".
 *
 * Model version              : 1.5
 * Simulink Coder version : 9.8 (R2022b) 13-May-2022
 * C++ source code generated on : Sat Nov 12 11:26:02 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objective: Debugging
 * Validation result: Not run
 */

#ifndef RTW_HEADER_AutoPIDTuner_types_h_
#define RTW_HEADER_AutoPIDTuner_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_k8WKy8tDBVlN8BL9RXVTiF_
#define DEFINED_TYPEDEF_FOR_struct_k8WKy8tDBVlN8BL9RXVTiF_

struct struct_k8WKy8tDBVlN8BL9RXVTiF
{
  uint8_T estimationMethod;
  int32_T nParameters;
  boolean_T isUsingFrames;
  int32_T windowLength;
};

#endif

/* Custom Type definition for MATLAB Function: '<S19>/RLS' */
#ifndef struct_c_controllib_internal_blocks__T
#define struct_c_controllib_internal_blocks__T

struct c_controllib_internal_blocks__T
{
  int32_T IteratorPosition;
};

#endif                              /* struct_c_controllib_internal_blocks__T */

#ifndef struct_d_controllib_internal_blocks__T
#define struct_d_controllib_internal_blocks__T

struct d_controllib_internal_blocks__T
{
  c_controllib_internal_blocks__T DataIterator;
};

#endif                              /* struct_d_controllib_internal_blocks__T */

/* Parameters (default storage) */
typedef struct P_AutoPIDTuner_T_ P_AutoPIDTuner_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_AutoPIDTuner_T RT_MODEL_AutoPIDTuner_T;

#endif                                 /* RTW_HEADER_AutoPIDTuner_types_h_ */
