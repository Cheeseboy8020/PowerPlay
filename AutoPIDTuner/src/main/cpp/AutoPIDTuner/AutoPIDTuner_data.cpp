/*
 * AutoPIDTuner_data.cpp
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

#include "AutoPIDTuner.h"

/* Block parameters (default storage) */
P_AutoPIDTuner_T AutoPIDTuner::AutoPIDTuner_P{
  /* Mask Parameter: ClosedLoopPIDAutotuner_AmpSine
   * Referenced by: '<S1>/sine Amp constant'
   */
  1.0,

  /* Mask Parameter: ClosedLoopPIDAutotuner_Bandwidt
   * Referenced by: '<S1>/bandwidth constant'
   */
  1.0,

  /* Mask Parameter: PIDController_D
   * Referenced by: '<S100>/Derivative Gain'
   */
  0.0,

  /* Mask Parameter: PIDController_I
   * Referenced by: '<S103>/Integral Gain'
   */
  1.0,

  /* Mask Parameter: PIDController_InitialConditionF
   * Referenced by: '<S101>/Filter'
   */
  0.0,

  /* Mask Parameter: PIDController_InitialConditio_k
   * Referenced by: '<S106>/Integrator'
   */
  0.0,

  /* Mask Parameter: PIDController_N
   * Referenced by: '<S109>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: PIDController_P
   * Referenced by: '<S111>/Proportional Gain'
   */
  1.0,

  /* Mask Parameter: ClosedLoopPIDAutotuner_PIDType
   * Referenced by: '<S1>/PID type constant'
   */
  7.0,

  /* Mask Parameter: ClosedLoopPIDAutotuner_PlantSig
   * Referenced by: '<S1>/plant sign constant'
   */
  1.0,

  /* Mask Parameter: ClosedLoopPIDAutotuner_PlantTyp
   * Referenced by: '<S1>/plant type constant'
   */
  1.0,

  /* Mask Parameter: ClosedLoopPIDAutotuner_TargetPM
   * Referenced by: '<S1>/target PM constant'
   */
  60.0,

  /* Mask Parameter: WrapToZero_Threshold
   * Referenced by: '<S18>/FixPt Switch'
   */
  4294967295U,

  /* Mask Parameter: ClosedLoopPIDAutotuner_Integrat
   * Referenced by: '<S1>/integrator formula constant'
   */
  1U,

  /* Mask Parameter: ClosedLoopPIDAutotuner_PIDForm
   * Referenced by: '<S1>/PID form constant'
   */
  1U,

  /* Computed Parameter: convergence_Y0
   * Referenced by: '<S3>/convergence'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S14>/one_constant'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S14>/zero_constant'
   */
  0.0,

  /* Computed Parameter: signal_Y0
   * Referenced by: '<S14>/signal'
   */
  0.0,

  /* Computed Parameter: regressors_Y0
   * Referenced by: '<S14>/regressors'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S14>/Switch2'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S14>/Switch1'
   */
  0.0,

  /* Computed Parameter: FreqResp_Y0
   * Referenced by: '<S15>/FreqResp'
   */
  0.0,

  /* Expression: initializationParams.adg2
   * Referenced by: '<S20>/Normalization Bias'
   */
  0.0,

  /* Expression: initializationParams.initialOutputs
   * Referenced by: '<S20>/InitialOutputs'
   */
  0.0,

  /* Expression: initializationParams.initialRegressors
   * Referenced by: '<S20>/InitialRegressors'
   */
  0.0,

  /* Expression: initializationParams.theta0
   * Referenced by: '<S20>/InitialParameters'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: initializationParams.L0
   * Referenced by: '<S20>/InitialCovariance'
   */
  { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0 },

  /* Expression: initializationParams.adg2
   * Referenced by: '<S19>/Normalization Bias'
   */
  0.0,

  /* Expression: initializationParams.initialOutputs
   * Referenced by: '<S19>/InitialOutputs'
   */
  0.0,

  /* Expression: initializationParams.initialRegressors
   * Referenced by: '<S19>/InitialRegressors'
   */
  0.0,

  /* Expression: initializationParams.theta0
   * Referenced by: '<S19>/InitialParameters'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: initializationParams.L0
   * Referenced by: '<S19>/InitialCovariance'
   */
  { 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0 },

  /* Expression: cast(ones(5,1),DataType)
   * Referenced by: '<S15>/Gain'
   */
  { 1.0, 1.0, 1.0, 1.0, 1.0 },

  /* Expression: 0
   * Referenced by: '<S6>/P'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S6>/I'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<S6>/D'
   */
  0.0,

  /* Expression: 100
   * Referenced by: '<S6>/N'
   */
  100.0,

  /* Expression: 0
   * Referenced by: '<S6>/achievedPM'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S1>/Negative'
   */
  -1.0,

  /* Expression: 1
   * Referenced by: '<S1>/Positive'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S1>/Integrating'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S1>/Stable'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Constant'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Constant1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Ref'
   */
  1.0,

  /* Expression: cast([1/10 1/3 1 3 10],DataType)
   * Referenced by: '<S1>/GainWC'
   */
  { 0.1, 0.33333333333333331, 1.0, 3.0, 10.0 },

  /* Expression: 1
   * Referenced by: '<S12>/has_integrator_constant1'
   */
  1.0,

  /* Computed Parameter: WeightedTs_WtEt
   * Referenced by: '<S1>/Weighted Ts'
   */
  0.1,

  /* Expression: 1
   * Referenced by: '<S1>/GainTs'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S74>/Switch'
   */
  0.0,

  /* Expression: EstimationWindowSize
   * Referenced by: '<S13>/Constant1'
   */
  3.0,

  /* Expression: 60
   * Referenced by: '<S13>/Constant2'
   */
  60.0,

  /* Expression: 1
   * Referenced by: '<S13>/Constant'
   */
  1.0,

  /* Expression: 1.5
   * Referenced by: '<S1>/Plant Type Switch'
   */
  1.5,

  /* Expression: 1.5
   * Referenced by: '<S1>/Plant Sign Switch'
   */
  1.5,

  /* Expression: cast(1,DataType)
   * Referenced by: '<S1>/GainPM'
   */
  1.0,

  /* Computed Parameter: Constant_Value_f
   * Referenced by: '<S18>/Constant'
   */
  0U,

  /* Computed Parameter: FixPtConstant_Value
   * Referenced by: '<S17>/FixPt Constant'
   */
  1U,

  /* Computed Parameter: Output_InitialCondition
   * Referenced by: '<S16>/Output'
   */
  0U,

  /* Computed Parameter: DeployedMode_TimeDomain
   * Referenced by: '<S71>/DeployedMode'
   */
  1U,

  /* Computed Parameter: filterformulaconstant_Value
   * Referenced by: '<S1>/filter formula constant'
   */
  1U,

  /* Expression: true()
   * Referenced by: '<S20>/Enable'
   */
  true,

  /* Expression: true()
   * Referenced by: '<S63>/Delay'
   */
  true,

  /* Expression: true()
   * Referenced by: '<S19>/Enable'
   */
  true,

  /* Expression: true()
   * Referenced by: '<S38>/Delay'
   */
  true,

  /* Expression: false()
   * Referenced by: '<S38>/Constant'
   */
  false,

  /* Expression: false()
   * Referenced by: '<S63>/Constant'
   */
  false,

  /* Computed Parameter: ManualSwitch_CurrentSetting
   * Referenced by: '<Root>/Manual Switch'
   */
  1U
};
