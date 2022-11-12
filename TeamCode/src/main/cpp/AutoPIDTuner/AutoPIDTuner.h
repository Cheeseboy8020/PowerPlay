/*
 * AutoPIDTuner.h
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

#ifndef RTW_HEADER_AutoPIDTuner_h_
#define RTW_HEADER_AutoPIDTuner_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_logging.h"
#include "AutoPIDTuner_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

#include "rt_zcfcn.h"
#include <cfloat>
#include <cstring>
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRTWLogInfo
#define rtmGetRTWLogInfo(rtm)          ((rtm)->rtwLogInfo)
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

/* Block signals for system '<S19>/RLS' */
struct B_RLS_AutoPIDTuner_T {
  real_T TmpSignalConversionAtSFunctionI[11];/* '<S19>/RLS' */
  real_T e;                            /* '<S19>/RLS' */
  real_T yBuffer;                      /* '<S19>/RLS' */
  real_T HBuffer;                      /* '<S19>/RLS' */
  real_T x[11];                        /* '<S19>/RLS' */
  real_T L[121];                       /* '<S19>/RLS' */
};

/* Block states (default storage) for system '<S19>/RLS' */
struct DW_RLS_AutoPIDTuner_T {
  d_controllib_internal_blocks__T rlsEstimator;/* '<S19>/RLS' */
  boolean_T rlsEstimator_not_empty;    /* '<S19>/RLS' */
};

/* Block signals (default storage) */
struct B_AutoPIDTuner_T {
  creal_T RealImagtoComplex[5];        /* '<S15>/Real-Imag to Complex' */
  creal_T RealImagtoComplex1[5];       /* '<S15>/Real-Imag to Complex1' */
  creal_T Divide1[5];                  /* '<S15>/Divide1' */
  creal_T Gain[5];                     /* '<S15>/Gain' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T ProportionalGain;             /* '<S111>/Proportional Gain' */
  real_T Integrator;                   /* '<S106>/Integrator' */
  real_T DerivativeGain;               /* '<S100>/Derivative Gain' */
  real_T Filter;                       /* '<S101>/Filter' */
  real_T SumD;                         /* '<S101>/SumD' */
  real_T FilterCoefficient;            /* '<S109>/Filter Coefficient' */
  real_T Sum_l;                        /* '<S115>/Sum' */
  real_T ZeroOrderHold1;               /* '<S74>/Zero-Order Hold1' */
  real_T w[5];                         /* '<S1>/GainWC' */
  real_T compiledTs;                   /* '<S1>/Weighted Ts' */
  real_T GainTs;                       /* '<S1>/GainTs' */
  real_T ManualSwitch;                 /* '<Root>/Manual Switch' */
  real_T Sum3;                         /* '<S74>/Sum3' */
  real_T Switch;                       /* '<S74>/Switch' */
  real_T ZeroOrderHold1_h;             /* '<S1>/Zero-Order Hold1' */
  real_T y;                            /* '<S7>/Enabled Delay Y' */
  real_T Sum2;                         /* '<S12>/Sum2' */
  real_T ZeroOrderHold2;               /* '<S74>/Zero-Order Hold2' */
  real_T u;                            /* '<S7>/Enabled Delay U' */
  real_T Sum3_c;                       /* '<S12>/Sum3' */
  real_T Product;                      /* '<S13>/Product' */
  real_T Product1;                     /* '<S13>/Product1' */
  real_T Divide;                       /* '<S13>/Divide' */
  real_T Sum_a;                        /* '<S13>/Sum' */
  real_T PlantTypeSwitch;              /* '<S1>/Plant Type Switch' */
  real_T PlantSignSwitch;              /* '<S1>/Plant Sign Switch' */
  real_T GainPM;                       /* '<S1>/GainPM' */
  real_T IntegralGain;                 /* '<S103>/Integral Gain' */
  real_T P;                            /* '<S71>/DeployedMode' */
  real_T I;                            /* '<S71>/DeployedMode' */
  real_T D;                            /* '<S71>/DeployedMode' */
  real_T N;                            /* '<S71>/DeployedMode' */
  real_T achievedPM;                   /* '<S71>/DeployedMode' */
  real_T delayBuffery;                 /* '<S20>/delayBuffery' */
  real_T delayBufferH;                 /* '<S20>/delayBufferH' */
  real_T delayTheta[11];               /* '<S20>/delayTheta' */
  real_T delayL[121];                  /* '<S20>/delayL' */
  real_T delayBuffery_b;               /* '<S19>/delayBuffery' */
  real_T delayBufferH_h;               /* '<S19>/delayBufferH' */
  real_T delayTheta_p[11];             /* '<S19>/delayTheta' */
  real_T delayL_e[121];                /* '<S19>/delayL' */
  real_T Product_g[5];                 /* '<S14>/Product' */
  real_T TrigonometricFunction[5];     /* '<S14>/Trigonometric Function' */
  real_T Sign;                         /* '<S14>/Sign' */
  real_T Product3;                     /* '<S14>/Product3' */
  real_T Product1_e[5];                /* '<S14>/Product1' */
  real_T TrigonometricFunction1[5];    /* '<S14>/Trigonometric Function1' */
  real_T Product2[5];                  /* '<S14>/Product2' */
  real_T SumofElements;                /* '<S14>/Sum of Elements' */
  real_T Switch2;                      /* '<S14>/Switch2' */
  real_T Sum_az;                       /* '<S14>/Sum' */
  real_T Switch1;                      /* '<S14>/Switch1' */
  uint32_T Output;                     /* '<S16>/Output' */
  uint32_T FixPtSum1;                  /* '<S17>/FixPt Sum1' */
  uint32_T FixPtSwitch;                /* '<S18>/FixPt Switch' */
  uint16_T TmpSignalConversionAtSFunctionI[3];/* '<S71>/DeployedMode' */
  boolean_T Delay;                     /* '<S63>/Delay' */
  boolean_T Delay_n;                   /* '<S38>/Delay' */
  B_RLS_AutoPIDTuner_T sf_RLS_h;       /* '<S20>/RLS' */
  B_RLS_AutoPIDTuner_T sf_RLS;         /* '<S19>/RLS' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_AutoPIDTuner_T {
  real_T EnabledDelayY_DSTATE;         /* '<S7>/Enabled Delay Y' */
  real_T EnabledDelayU_DSTATE;         /* '<S7>/Enabled Delay U' */
  real_T PP;                           /* '<S1>/ExternalModeMonitor' */
  real_T II;                           /* '<S1>/ExternalModeMonitor' */
  real_T DD;                           /* '<S1>/ExternalModeMonitor' */
  real_T NN;                           /* '<S1>/ExternalModeMonitor' */
  real_T PM;                           /* '<S1>/ExternalModeMonitor' */
  real_T LastStartStop;                /* '<S1>/ExternalModeMonitor' */
  real_T delayBuffery_DSTATE;          /* '<S20>/delayBuffery' */
  real_T delayBufferH_DSTATE;          /* '<S20>/delayBufferH' */
  real_T delayTheta_DSTATE[11];        /* '<S20>/delayTheta' */
  real_T delayL_DSTATE[121];           /* '<S20>/delayL' */
  real_T delayBuffery_DSTATE_m;        /* '<S19>/delayBuffery' */
  real_T delayBufferH_DSTATE_k;        /* '<S19>/delayBufferH' */
  real_T delayTheta_DSTATE_a[11];      /* '<S19>/delayTheta' */
  real_T delayL_DSTATE_m[121];         /* '<S19>/delayL' */
  uint32_T Output_DSTATE;              /* '<S16>/Output' */
  boolean_T Delay_DSTATE;              /* '<S63>/Delay' */
  boolean_T Delay_DSTATE_i;            /* '<S38>/Delay' */
  boolean_T icLoad;                    /* '<S7>/Enabled Delay Y' */
  boolean_T icLoad_m;                  /* '<S7>/Enabled Delay U' */
  boolean_T icLoad_n;                  /* '<S20>/delayBuffery' */
  boolean_T icLoad_h;                  /* '<S20>/delayBufferH' */
  boolean_T icLoad_a;                  /* '<S20>/delayTheta' */
  boolean_T icLoad_i;                  /* '<S20>/delayL' */
  boolean_T icLoad_g;                  /* '<S19>/delayBuffery' */
  boolean_T icLoad_i5;                 /* '<S19>/delayBufferH' */
  boolean_T icLoad_o;                  /* '<S19>/delayTheta' */
  boolean_T icLoad_id;                 /* '<S19>/delayL' */
  boolean_T ResponseEstimation_MODE;   /* '<S12>/Response Estimation' */
  boolean_T PerturbationGenerator_MODE;/* '<S12>/Perturbation Generator' */
  DW_RLS_AutoPIDTuner_T sf_RLS_h;      /* '<S20>/RLS' */
  DW_RLS_AutoPIDTuner_T sf_RLS;        /* '<S19>/RLS' */
};

/* Continuous states (default storage) */
struct X_AutoPIDTuner_T {
  real_T Integrator_CSTATE;            /* '<S106>/Integrator' */
  real_T Filter_CSTATE;                /* '<S101>/Filter' */
};

/* State derivatives (default storage) */
struct XDot_AutoPIDTuner_T {
  real_T Integrator_CSTATE;            /* '<S106>/Integrator' */
  real_T Filter_CSTATE;                /* '<S101>/Filter' */
};

/* State disabled  */
struct XDis_AutoPIDTuner_T {
  boolean_T Integrator_CSTATE;         /* '<S106>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S101>/Filter' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_AutoPIDTuner_T {
  ZCSigState EnabledDelayY_Reset_ZCE;  /* '<S7>/Enabled Delay Y' */
  ZCSigState EnabledDelayU_Reset_ZCE;  /* '<S7>/Enabled Delay U' */
  ZCSigState GainsFromOnlineTuningModule_Tri;
                                    /* '<S1>/Gains From Online Tuning Module' */
};

#ifndef ODE3_INTG
#define ODE3_INTG

/* ODE3 Integration Data */
struct ODE3_IntgData {
  real_T *y;                           /* output */
  real_T *f[3];                        /* derivatives */
};

#endif

/* External inputs (root inport signals with default storage) */
struct ExtU_AutoPIDTuner_T {
  real_T EncoderPos;                   /* '<Root>/EncoderPos' */
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_AutoPIDTuner_T {
  real_T Velo;                         /* '<Root>/Velo' */
};

/* Parameters (default storage) */
struct P_AutoPIDTuner_T_ {
  real_T ClosedLoopPIDAutotuner_AmpSine;
                               /* Mask Parameter: ClosedLoopPIDAutotuner_AmpSine
                                * Referenced by: '<S1>/sine Amp constant'
                                */
  real_T ClosedLoopPIDAutotuner_Bandwidt;
                              /* Mask Parameter: ClosedLoopPIDAutotuner_Bandwidt
                               * Referenced by: '<S1>/bandwidth constant'
                               */
  real_T PIDController_D;              /* Mask Parameter: PIDController_D
                                        * Referenced by: '<S100>/Derivative Gain'
                                        */
  real_T PIDController_I;              /* Mask Parameter: PIDController_I
                                        * Referenced by: '<S103>/Integral Gain'
                                        */
  real_T PIDController_InitialConditionF;
                              /* Mask Parameter: PIDController_InitialConditionF
                               * Referenced by: '<S101>/Filter'
                               */
  real_T PIDController_InitialConditio_k;
                              /* Mask Parameter: PIDController_InitialConditio_k
                               * Referenced by: '<S106>/Integrator'
                               */
  real_T PIDController_N;              /* Mask Parameter: PIDController_N
                                        * Referenced by: '<S109>/Filter Coefficient'
                                        */
  real_T PIDController_P;              /* Mask Parameter: PIDController_P
                                        * Referenced by: '<S111>/Proportional Gain'
                                        */
  real_T ClosedLoopPIDAutotuner_PIDType;
                               /* Mask Parameter: ClosedLoopPIDAutotuner_PIDType
                                * Referenced by: '<S1>/PID type constant'
                                */
  real_T ClosedLoopPIDAutotuner_PlantSig;
                              /* Mask Parameter: ClosedLoopPIDAutotuner_PlantSig
                               * Referenced by: '<S1>/plant sign constant'
                               */
  real_T ClosedLoopPIDAutotuner_PlantTyp;
                              /* Mask Parameter: ClosedLoopPIDAutotuner_PlantTyp
                               * Referenced by: '<S1>/plant type constant'
                               */
  real_T ClosedLoopPIDAutotuner_TargetPM;
                              /* Mask Parameter: ClosedLoopPIDAutotuner_TargetPM
                               * Referenced by: '<S1>/target PM constant'
                               */
  uint32_T WrapToZero_Threshold;       /* Mask Parameter: WrapToZero_Threshold
                                        * Referenced by: '<S18>/FixPt Switch'
                                        */
  uint16_T ClosedLoopPIDAutotuner_Integrat;
                              /* Mask Parameter: ClosedLoopPIDAutotuner_Integrat
                               * Referenced by: '<S1>/integrator formula constant'
                               */
  uint16_T ClosedLoopPIDAutotuner_PIDForm;
                               /* Mask Parameter: ClosedLoopPIDAutotuner_PIDForm
                                * Referenced by: '<S1>/PID form constant'
                                */
  real_T convergence_Y0;               /* Computed Parameter: convergence_Y0
                                        * Referenced by: '<S3>/convergence'
                                        */
  real_T one_constant_Value;           /* Expression: 1
                                        * Referenced by: '<S14>/one_constant'
                                        */
  real_T zero_constant_Value;          /* Expression: 0
                                        * Referenced by: '<S14>/zero_constant'
                                        */
  real_T signal_Y0;                    /* Computed Parameter: signal_Y0
                                        * Referenced by: '<S14>/signal'
                                        */
  real_T regressors_Y0;                /* Computed Parameter: regressors_Y0
                                        * Referenced by: '<S14>/regressors'
                                        */
  real_T Switch2_Threshold;            /* Expression: 0
                                        * Referenced by: '<S14>/Switch2'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<S14>/Switch1'
                                        */
  real_T FreqResp_Y0;                  /* Computed Parameter: FreqResp_Y0
                                        * Referenced by: '<S15>/FreqResp'
                                        */
  real_T NormalizationBias_Value;      /* Expression: initializationParams.adg2
                                        * Referenced by: '<S20>/Normalization Bias'
                                        */
  real_T InitialOutputs_Value;/* Expression: initializationParams.initialOutputs
                               * Referenced by: '<S20>/InitialOutputs'
                               */
  real_T InitialRegressors_Value;
                           /* Expression: initializationParams.initialRegressors
                            * Referenced by: '<S20>/InitialRegressors'
                            */
  real_T InitialParameters_Value[11]; /* Expression: initializationParams.theta0
                                       * Referenced by: '<S20>/InitialParameters'
                                       */
  real_T InitialCovariance_Value[121]; /* Expression: initializationParams.L0
                                        * Referenced by: '<S20>/InitialCovariance'
                                        */
  real_T NormalizationBias_Value_m;    /* Expression: initializationParams.adg2
                                        * Referenced by: '<S19>/Normalization Bias'
                                        */
  real_T InitialOutputs_Value_d;
                              /* Expression: initializationParams.initialOutputs
                               * Referenced by: '<S19>/InitialOutputs'
                               */
  real_T InitialRegressors_Value_g;
                           /* Expression: initializationParams.initialRegressors
                            * Referenced by: '<S19>/InitialRegressors'
                            */
  real_T InitialParameters_Value_f[11];
                                      /* Expression: initializationParams.theta0
                                       * Referenced by: '<S19>/InitialParameters'
                                       */
  real_T InitialCovariance_Value_l[121];/* Expression: initializationParams.L0
                                         * Referenced by: '<S19>/InitialCovariance'
                                         */
  real_T Gain_Gain[5];                 /* Expression: cast(ones(5,1),DataType)
                                        * Referenced by: '<S15>/Gain'
                                        */
  real_T P_Y0;                         /* Expression: 0
                                        * Referenced by: '<S6>/P'
                                        */
  real_T I_Y0;                         /* Expression: 0
                                        * Referenced by: '<S6>/I'
                                        */
  real_T D_Y0;                         /* Expression: 0
                                        * Referenced by: '<S6>/D'
                                        */
  real_T N_Y0;                         /* Expression: 100
                                        * Referenced by: '<S6>/N'
                                        */
  real_T achievedPM_Y0;                /* Expression: 0
                                        * Referenced by: '<S6>/achievedPM'
                                        */
  real_T Negative_Value;               /* Expression: -1
                                        * Referenced by: '<S1>/Negative'
                                        */
  real_T Positive_Value;               /* Expression: 1
                                        * Referenced by: '<S1>/Positive'
                                        */
  real_T Integrating_Value;            /* Expression: 1
                                        * Referenced by: '<S1>/Integrating'
                                        */
  real_T Stable_Value;                 /* Expression: 0
                                        * Referenced by: '<S1>/Stable'
                                        */
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<Root>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T Ref_Value;                    /* Expression: 1
                                        * Referenced by: '<Root>/Ref'
                                        */
  real_T GainWC_Gain[5];         /* Expression: cast([1/10 1/3 1 3 10],DataType)
                                  * Referenced by: '<S1>/GainWC'
                                  */
  real_T has_integrator_constant1_Value;/* Expression: 1
                                         * Referenced by: '<S12>/has_integrator_constant1'
                                         */
  real_T WeightedTs_WtEt;              /* Computed Parameter: WeightedTs_WtEt
                                        * Referenced by: '<S1>/Weighted Ts'
                                        */
  real_T GainTs_Gain;                  /* Expression: 1
                                        * Referenced by: '<S1>/GainTs'
                                        */
  real_T Switch_Threshold;             /* Expression: 0
                                        * Referenced by: '<S74>/Switch'
                                        */
  real_T Constant1_Value_p;            /* Expression: EstimationWindowSize
                                        * Referenced by: '<S13>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 60
                                        * Referenced by: '<S13>/Constant2'
                                        */
  real_T Constant_Value_k;             /* Expression: 1
                                        * Referenced by: '<S13>/Constant'
                                        */
  real_T PlantTypeSwitch_Threshold;    /* Expression: 1.5
                                        * Referenced by: '<S1>/Plant Type Switch'
                                        */
  real_T PlantSignSwitch_Threshold;    /* Expression: 1.5
                                        * Referenced by: '<S1>/Plant Sign Switch'
                                        */
  real_T GainPM_Gain;                  /* Expression: cast(1,DataType)
                                        * Referenced by: '<S1>/GainPM'
                                        */
  uint32_T Constant_Value_f;           /* Computed Parameter: Constant_Value_f
                                        * Referenced by: '<S18>/Constant'
                                        */
  uint32_T FixPtConstant_Value;       /* Computed Parameter: FixPtConstant_Value
                                       * Referenced by: '<S17>/FixPt Constant'
                                       */
  uint32_T Output_InitialCondition;
                                  /* Computed Parameter: Output_InitialCondition
                                   * Referenced by: '<S16>/Output'
                                   */
  uint16_T DeployedMode_TimeDomain;
                                  /* Computed Parameter: DeployedMode_TimeDomain
                                   * Referenced by: '<S71>/DeployedMode'
                                   */
  uint16_T filterformulaconstant_Value;
                              /* Computed Parameter: filterformulaconstant_Value
                               * Referenced by: '<S1>/filter formula constant'
                               */
  boolean_T Enable_Value;              /* Expression: true()
                                        * Referenced by: '<S20>/Enable'
                                        */
  boolean_T Delay_InitialCondition;    /* Expression: true()
                                        * Referenced by: '<S63>/Delay'
                                        */
  boolean_T Enable_Value_j;            /* Expression: true()
                                        * Referenced by: '<S19>/Enable'
                                        */
  boolean_T Delay_InitialCondition_j;  /* Expression: true()
                                        * Referenced by: '<S38>/Delay'
                                        */
  boolean_T Constant_Value_j;          /* Expression: false()
                                        * Referenced by: '<S38>/Constant'
                                        */
  boolean_T Constant_Value_b;          /* Expression: false()
                                        * Referenced by: '<S63>/Constant'
                                        */
  uint8_T ManualSwitch_CurrentSetting;
                              /* Computed Parameter: ManualSwitch_CurrentSetting
                               * Referenced by: '<Root>/Manual Switch'
                               */
};

/* Real-time Model Data Structure */
struct tag_RTM_AutoPIDTuner_T {
  const char_T *errorStatus;
  RTWLogInfo *rtwLogInfo;
  RTWSolverInfo solverInfo;
  X_AutoPIDTuner_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_AutoPIDTuner_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[2];
  real_T odeF[3][2];
  ODE3_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    time_T tFinal;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Class declaration for model AutoPIDTuner */
class AutoPIDTuner final
{
  /* public data and function members */
 public:
  /* Copy Constructor */
  AutoPIDTuner(AutoPIDTuner const&) = delete;

  /* Assignment Operator */
  AutoPIDTuner& operator= (AutoPIDTuner const&) & = delete;

  /* Move Constructor */
  AutoPIDTuner(AutoPIDTuner &&) = delete;

  /* Move Assignment Operator */
  AutoPIDTuner& operator= (AutoPIDTuner &&) = delete;

  /* Real-Time Model get method */
  RT_MODEL_AutoPIDTuner_T * getRTM();

  /* External inputs */
  ExtU_AutoPIDTuner_T AutoPIDTuner_U;

  /* External outputs */
  ExtY_AutoPIDTuner_T AutoPIDTuner_Y;
  void ModelPrevZCStateInit();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  AutoPIDTuner();

  /* Destructor */
  ~AutoPIDTuner();

  /* private data and function members */
 private:
  /* Block signals */
  B_AutoPIDTuner_T AutoPIDTuner_B;

  /* Block states */
  DW_AutoPIDTuner_T AutoPIDTuner_DW;

  /* Tunable parameters */
  static P_AutoPIDTuner_T AutoPIDTuner_P;

  /* Block continuous states */
  X_AutoPIDTuner_T AutoPIDTuner_X;

  /* Triggered events */
  PrevZCX_AutoPIDTuner_T AutoPIDTuner_PrevZCX;

  /* private member function(s) for subsystem '<S19>/RLS'*/
  static void AutoPIDTuner_RLS_Reset(DW_RLS_AutoPIDTuner_T *localDW);
  void AutoPIDTuner_RLS(const real_T rtu_H[5], const real_T rtu_H_i[5], real_T
                        rtu_H_ip, real_T rtu_y, boolean_T rtu_isEnabled, real_T
                        rtu_adg1, real_T rtu_yBuffer, real_T rtu_HBuffer, const
                        real_T rtu_x[11], const real_T rtu_L[121],
                        B_RLS_AutoPIDTuner_T *localB, DW_RLS_AutoPIDTuner_T
                        *localDW);
  real_T AutoPIDTuner_xnrm2(int32_T n, const real_T x[12], int32_T ix0);
  real_T AutoPIDTuner_qrFactor(const real_T A[11], const real_T S[121], real_T
    Ns);
  void AutoPIDTuner_trisolve(real_T A, real_T B[11]);
  real_T AutoPIDTuner_xnrm2_o(int32_T n, const real_T x[132], int32_T ix0);
  void AutoPIDTuner_xgemv(int32_T m, int32_T n, const real_T A[132], int32_T ia0,
    const real_T x[132], int32_T ix0, real_T y[11]);
  void AutoPIDTuner_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
    const real_T y[11], real_T A[132], int32_T ia0);
  void AutoPIDTu_sqrtMeasurementUpdate(real_T L[121], const real_T H[11], real_T
    a0, real_T K[11]);

  /* private member function(s) for subsystem '<Root>'*/
  void AutoPIDTuner_generateTargetLoop(const real_T w[3], real_T targetPM,
    creal_T L[3]);
  void AutoPIDTuner_logspace(real_T d1, real_T d2, real_T y[50]);
  void AutoPIDTuner_pchip(const real_T x[5], const real_T y[5], const real_T xx
    [100], real_T v[100]);
  void AutoPIDTuner_localGetRealImag(real_T Ts, const real_T w[3], uint16_T
    Formula, real_T *realX, real_T imagX[3]);
  void AutoPIDTuner_blkdiag(const real_T varargin_1[6], real_T y[14]);
  real_T AutoPIDTuner_minimum(const real_T x[50]);
  boolean_T AutoPIDTuner_vectorAny(const boolean_T x_data[], const int32_T
    *x_size);
  real_T AutoPIDTuner_xnrm2_b(int32_T n, const real_T x[49], int32_T ix0);
  void AutoPIDTuner_xgemv_f(int32_T m, int32_T n, const real_T A[49], int32_T
    ia0, const real_T x[49], int32_T ix0, real_T y[7]);
  void AutoPIDTuner_xgerc_c(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
    const real_T y[7], real_T A[49], int32_T ia0);
  void AutoPIDTuner_xgeqp3(real_T A[49], real_T tau[7], int32_T jpvt[7]);
  void AutoPIDTuner_xorgqr(int32_T k, real_T A[49], const real_T tau[7]);
  void AutoPIDTuner_computeZ(real_T Z[2], const real_T C[14], const real_T d[7],
    const boolean_T p[2]);
  void AutoPIDTun_utilLSQFixedSizeData(const real_T C[14], const real_T d[7],
    real_T x[2]);
  void AutoPIDTuner_blkdiag_c(const real_T varargin_1[12], real_T y[21]);
  real_T AutoPIDTuner_norm(const real_T x[21]);
  void AutoPIDTuner_maximum(const real_T x[3], real_T *ex, int32_T *idx);
  void AutoPIDTuner_computeZ_j(real_T Z[3], const real_T C[21], const real_T d[7],
    const boolean_T p[3]);
  real_T AutoPIDTuner_computeAlpha(const real_T x[3], const real_T z[3], const
    boolean_T Q[3]);
  void AutoPIDT_utilLSQFixedSizeData_j(const real_T C[21], const real_T d[7],
    real_T x[3]);
  real_T AutoPIDTuner_computePM(const creal_T L);
  real_T AutoPIDTuner_maximum_f(const real_T x[50]);
  void AutoPIDTuner_blkdiag_ch(const real_T varargin_1[12], real_T y[32]);
  real_T AutoPIDTuner_norm_p(const real_T x[32]);
  void AutoPIDTuner_maximum_f5(const real_T x[4], real_T *ex, int32_T *idx);
  real_T AutoPIDTuner_xnrm2_bx(int32_T n, const real_T x[64], int32_T ix0);
  void AutoPIDTuner_xgemv_f2(int32_T m, int32_T n, const real_T A[64], int32_T
    ia0, const real_T x[64], int32_T ix0, real_T y[8]);
  void AutoPIDTuner_xgerc_c4(int32_T m, int32_T n, real_T alpha1, int32_T ix0,
    const real_T y[8], real_T A[64], int32_T ia0);
  void AutoPIDTuner_xgeqp3_k(real_T A[64], real_T tau[8], int32_T jpvt[8]);
  void AutoPIDTuner_xorgqr_o(int32_T k, real_T A[64], const real_T tau[8]);
  void AutoPIDTuner_computeZ_j3(real_T Z[4], const real_T C[32], const real_T d
    [8], const boolean_T p[4]);
  real_T AutoPIDTuner_computeAlpha_f(const real_T x[4], const real_T z[4], const
    boolean_T Q[4]);
  void AutoPID_utilLSQFixedSizeData_j2(const real_T C[32], const real_T d[8],
    real_T x[4]);
  void AutoPIDTuner_logspace_l(real_T d1, real_T d2, real_T y[20]);
  void AutoPIDTuner_computeTAU(boolean_T IsDiscrete, const real_T w3[3], real_T
    Ts, uint16_T DF, real_T tau[20]);
  void AutoP_utilLSQFixedSizeData_j2zd(const real_T C[21], const real_T d[7],
    real_T x[3], real_T *resnorm);
  real_T AutoPIDTuner_maximum_f5j(const real_T x[20]);
  real_T AutoPIDTuner_minimum_g(const real_T x[20]);
  real_T AutoPIDTuner_mean(const real_T x[20]);
  void AutoPIDTuner_minimum_g1(const real_T x[20], real_T *ex, int32_T *idx);
  void AutoPI_utilLSQFixedSizeData_j2z(const real_T C[32], const real_T d[8],
    real_T x[4], real_T *resnorm);
  void AutoPIDTuner_blkdiag_chw(const real_T varargin_1[18], real_T y[40]);
  real_T AutoPIDTuner_norm_pt(const real_T x[40]);
  void AutoPIDTuner_maximum_f5jx(const real_T x[5], real_T *ex, int32_T *idx);
  void AutoPIDTuner_computeZ_j3j(real_T Z[5], const real_T C[40], const real_T
    d[8], const boolean_T p[5]);
  real_T AutoPIDTuner_computeAlpha_fd(const real_T x[5], const real_T z[5],
    const boolean_T Q[5]);
  void Auto_utilLSQFixedSizeData_j2zds(const real_T C[40], const real_T d[8],
    real_T x[5]);
  void Aut_utilLSQFixedSizeData_j2zdsx(const real_T C[40], const real_T d[8],
    real_T x[5], real_T *resnorm);
  void AutoPIDTuner_slpidfivepoint(real_T type, uint16_T form, const real_T
    frequencies[5], creal_T responses[5], real_T targetPM, real_T HasIntegrator,
    real_T LoopSign, real_T Ts, uint16_T IF, uint16_T DF, real_T *P, real_T *b_I,
    real_T *D, real_T *N, real_T *achievedPM);

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void AutoPIDTuner_derivatives();

  /* Real-Time Model */
  RT_MODEL_AutoPIDTuner_T AutoPIDTuner_M;
};

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/Constant' : Unused code path elimination
 * Block '<S3>/Abs3' : Unused code path elimination
 * Block '<S3>/Complex to Magnitude-Angle' : Unused code path elimination
 * Block '<S3>/Constant2' : Unused code path elimination
 * Block '<S10>/Diff' : Unused code path elimination
 * Block '<S10>/UD' : Unused code path elimination
 * Block '<S3>/Divide1' : Unused code path elimination
 * Block '<S3>/Gain' : Unused code path elimination
 * Block '<S3>/Gain2' : Unused code path elimination
 * Block '<S3>/MinMax1' : Unused code path elimination
 * Block '<S3>/Saturation' : Unused code path elimination
 * Block '<S3>/Sum of Elements' : Unused code path elimination
 * Block '<S3>/Sum2' : Unused code path elimination
 * Block '<S3>/Tapped Delay' : Unused code path elimination
 * Block '<S11>/Abs3' : Unused code path elimination
 * Block '<S11>/Constant' : Unused code path elimination
 * Block '<S11>/Constant1' : Unused code path elimination
 * Block '<S11>/Switch' : Unused code path elimination
 * Block '<S11>/Switch1' : Unused code path elimination
 * Block '<S1>/DisplayPM' : Unused code path elimination
 * Block '<S1>/DisplayTs' : Unused code path elimination
 * Block '<S1>/DisplayWC' : Unused code path elimination
 * Block '<S12>/DisplayFRD' : Unused code path elimination
 * Block '<S16>/FixPt Data Type Propagation' : Unused code path elimination
 * Block '<S17>/FixPt Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/FixPt Data Type Duplicate1' : Unused code path elimination
 * Block '<S19>/Check Same Ts' : Unused code path elimination
 * Block '<S26>/Output Dimension' : Unused code path elimination
 * Block '<S26>/Regressors Dimension' : Unused code path elimination
 * Block '<S26>/Sample Times and Data Type' : Unused code path elimination
 * Block '<S27>/Data Type Duplicate' : Unused code path elimination
 * Block '<S28>/Data Type Duplicate' : Unused code path elimination
 * Block '<S29>/Data Type Duplicate' : Unused code path elimination
 * Block '<S30>/Data Type Duplicate' : Unused code path elimination
 * Block '<S31>/Data Type Duplicate' : Unused code path elimination
 * Block '<S32>/Data Type Duplicate' : Unused code path elimination
 * Block '<S33>/Data Type Duplicate' : Unused code path elimination
 * Block '<S34>/Data Type Duplicate' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate' : Unused code path elimination
 * Block '<S36>/Data Type Duplicate' : Unused code path elimination
 * Block '<S45>/S-Function' : Unused code path elimination
 * Block '<S44>/Gain' : Unused code path elimination
 * Block '<S44>/Selector' : Unused code path elimination
 * Block '<S20>/Check Same Ts' : Unused code path elimination
 * Block '<S51>/Output Dimension' : Unused code path elimination
 * Block '<S51>/Regressors Dimension' : Unused code path elimination
 * Block '<S51>/Sample Times and Data Type' : Unused code path elimination
 * Block '<S52>/Data Type Duplicate' : Unused code path elimination
 * Block '<S53>/Data Type Duplicate' : Unused code path elimination
 * Block '<S54>/Data Type Duplicate' : Unused code path elimination
 * Block '<S55>/Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S57>/Data Type Duplicate' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 * Block '<S59>/Data Type Duplicate' : Unused code path elimination
 * Block '<S60>/Data Type Duplicate' : Unused code path elimination
 * Block '<S61>/Data Type Duplicate' : Unused code path elimination
 * Block '<S70>/S-Function' : Unused code path elimination
 * Block '<S69>/Gain' : Unused code path elimination
 * Block '<S69>/Selector' : Unused code path elimination
 * Block '<S5>/GainD' : Unused code path elimination
 * Block '<S5>/GainI' : Unused code path elimination
 * Block '<S5>/GainN' : Unused code path elimination
 * Block '<S5>/GainP' : Unused code path elimination
 * Block '<S5>/acheivedPM' : Unused code path elimination
 * Block '<S7>/DisplayU0' : Unused code path elimination
 * Block '<S7>/DisplayY0' : Unused code path elimination
 * Block '<S1>/PM Switch' : Unused code path elimination
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<Root>/Display1' : Unused code path elimination
 * Block '<Root>/Display2' : Unused code path elimination
 * Block '<Root>/Display3' : Unused code path elimination
 * Block '<Root>/Display4' : Unused code path elimination
 * Block '<Root>/Signal Copy' : Unused code path elimination
 * Block '<Root>/Signal Copy1' : Unused code path elimination
 * Block '<Root>/Signal Copy2' : Unused code path elimination
 * Block '<Root>/Signal Copy3' : Unused code path elimination
 * Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S1>/Data Type Conversion_Ts' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_PM' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_StartStop' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_ampSine' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_ui' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_wC' : Eliminate redundant data type conversion
 * Block '<S1>/DataTypeConversion_y' : Eliminate redundant data type conversion
 * Block '<S27>/Conversion' : Eliminate redundant data type conversion
 * Block '<S33>/Conversion' : Eliminate redundant data type conversion
 * Block '<S34>/Conversion' : Eliminate redundant data type conversion
 * Block '<S35>/Conversion' : Eliminate redundant data type conversion
 * Block '<S36>/Conversion' : Eliminate redundant data type conversion
 * Block '<S19>/DataTypeConversionEnable' : Eliminate redundant data type conversion
 * Block '<S52>/Conversion' : Eliminate redundant data type conversion
 * Block '<S58>/Conversion' : Eliminate redundant data type conversion
 * Block '<S59>/Conversion' : Eliminate redundant data type conversion
 * Block '<S60>/Conversion' : Eliminate redundant data type conversion
 * Block '<S61>/Conversion' : Eliminate redundant data type conversion
 * Block '<S20>/DataTypeConversionEnable' : Eliminate redundant data type conversion
 * Block '<S1>/Zero-Order Hold' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'AutoPIDTuner'
 * '<S1>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner'
 * '<S2>'   : 'AutoPIDTuner/PID Controller'
 * '<S3>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Convergence Calculator'
 * '<S4>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator'
 * '<S5>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Gains From Offline Tuning Module'
 * '<S6>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Gains From Online Tuning Module'
 * '<S7>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/New Nominal Detector'
 * '<S8>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Tuning Module Sample Time Parameters'
 * '<S9>'   : 'AutoPIDTuner/Closed-Loop PID Autotuner/Variant Subsystem'
 * '<S10>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Convergence Calculator/Difference1'
 * '<S11>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Convergence Calculator/Zero protection'
 * '<S12>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed'
 * '<S13>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/FF Calculation'
 * '<S14>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Perturbation Generator'
 * '<S15>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation'
 * '<S16>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Perturbation Generator/Counter Free-Running'
 * '<S17>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Perturbation Generator/Counter Free-Running/Increment Real World'
 * '<S18>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Perturbation Generator/Counter Free-Running/Wrap To Zero'
 * '<S19>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)'
 * '<S20>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)'
 * '<S21>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Enable Signal'
 * '<S22>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Initial Covariance'
 * '<S23>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Initial Outputs'
 * '<S24>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Initial Parameters'
 * '<S25>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Initial Regressors'
 * '<S26>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Check Signals'
 * '<S27>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - AdaptationParameter1'
 * '<S28>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - AdaptationParameter2'
 * '<S29>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - InitialCovariance'
 * '<S30>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - InitialOutputs'
 * '<S31>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - InitialParameters'
 * '<S32>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - InitialRegressors'
 * '<S33>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - L'
 * '<S34>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - Theta'
 * '<S35>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - bufferH'
 * '<S36>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Data Type Conversion - buffery'
 * '<S37>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/MultiplyWithTranspose'
 * '<S38>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Process Reset'
 * '<S39>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/ProcessInitialCovariance'
 * '<S40>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/ProcessInitialOutputs'
 * '<S41>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/ProcessInitialParameters'
 * '<S42>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/ProcessInitialRegressors'
 * '<S43>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/RLS'
 * '<S44>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Reset'
 * '<S45>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator 1//(1+GC)/Process Reset/Check Reset'
 * '<S46>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Enable Signal'
 * '<S47>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Initial Covariance'
 * '<S48>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Initial Outputs'
 * '<S49>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Initial Parameters'
 * '<S50>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Initial Regressors'
 * '<S51>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Check Signals'
 * '<S52>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - AdaptationParameter1'
 * '<S53>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - AdaptationParameter2'
 * '<S54>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - InitialCovariance'
 * '<S55>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - InitialOutputs'
 * '<S56>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - InitialParameters'
 * '<S57>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - InitialRegressors'
 * '<S58>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - L'
 * '<S59>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - Theta'
 * '<S60>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - bufferH'
 * '<S61>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Data Type Conversion - buffery'
 * '<S62>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/MultiplyWithTranspose'
 * '<S63>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Process Reset'
 * '<S64>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/ProcessInitialCovariance'
 * '<S65>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/ProcessInitialOutputs'
 * '<S66>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/ProcessInitialParameters'
 * '<S67>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/ProcessInitialRegressors'
 * '<S68>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/RLS'
 * '<S69>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Reset'
 * '<S70>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Frequency Response Estimator/Superimposed/Response Estimation/RLS Estimator G//(1+GC)/Process Reset/Check Reset'
 * '<S71>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Gains From Online Tuning Module/Tune Subsystem'
 * '<S72>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Gains From Online Tuning Module/Tune Subsystem/DeployedMode'
 * '<S73>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Tuning Module Sample Time Parameters/single rate'
 * '<S74>'  : 'AutoPIDTuner/Closed-Loop PID Autotuner/Variant Subsystem/ControlPlusPerturb'
 * '<S75>'  : 'AutoPIDTuner/PID Controller/Anti-windup'
 * '<S76>'  : 'AutoPIDTuner/PID Controller/D Gain'
 * '<S77>'  : 'AutoPIDTuner/PID Controller/Filter'
 * '<S78>'  : 'AutoPIDTuner/PID Controller/Filter ICs'
 * '<S79>'  : 'AutoPIDTuner/PID Controller/I Gain'
 * '<S80>'  : 'AutoPIDTuner/PID Controller/Ideal P Gain'
 * '<S81>'  : 'AutoPIDTuner/PID Controller/Ideal P Gain Fdbk'
 * '<S82>'  : 'AutoPIDTuner/PID Controller/Integrator'
 * '<S83>'  : 'AutoPIDTuner/PID Controller/Integrator ICs'
 * '<S84>'  : 'AutoPIDTuner/PID Controller/N Copy'
 * '<S85>'  : 'AutoPIDTuner/PID Controller/N Gain'
 * '<S86>'  : 'AutoPIDTuner/PID Controller/P Copy'
 * '<S87>'  : 'AutoPIDTuner/PID Controller/Parallel P Gain'
 * '<S88>'  : 'AutoPIDTuner/PID Controller/Reset Signal'
 * '<S89>'  : 'AutoPIDTuner/PID Controller/Saturation'
 * '<S90>'  : 'AutoPIDTuner/PID Controller/Saturation Fdbk'
 * '<S91>'  : 'AutoPIDTuner/PID Controller/Sum'
 * '<S92>'  : 'AutoPIDTuner/PID Controller/Sum Fdbk'
 * '<S93>'  : 'AutoPIDTuner/PID Controller/Tracking Mode'
 * '<S94>'  : 'AutoPIDTuner/PID Controller/Tracking Mode Sum'
 * '<S95>'  : 'AutoPIDTuner/PID Controller/Tsamp - Integral'
 * '<S96>'  : 'AutoPIDTuner/PID Controller/Tsamp - Ngain'
 * '<S97>'  : 'AutoPIDTuner/PID Controller/postSat Signal'
 * '<S98>'  : 'AutoPIDTuner/PID Controller/preSat Signal'
 * '<S99>'  : 'AutoPIDTuner/PID Controller/Anti-windup/Passthrough'
 * '<S100>' : 'AutoPIDTuner/PID Controller/D Gain/Internal Parameters'
 * '<S101>' : 'AutoPIDTuner/PID Controller/Filter/Cont. Filter'
 * '<S102>' : 'AutoPIDTuner/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S103>' : 'AutoPIDTuner/PID Controller/I Gain/Internal Parameters'
 * '<S104>' : 'AutoPIDTuner/PID Controller/Ideal P Gain/Passthrough'
 * '<S105>' : 'AutoPIDTuner/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S106>' : 'AutoPIDTuner/PID Controller/Integrator/Continuous'
 * '<S107>' : 'AutoPIDTuner/PID Controller/Integrator ICs/Internal IC'
 * '<S108>' : 'AutoPIDTuner/PID Controller/N Copy/Disabled'
 * '<S109>' : 'AutoPIDTuner/PID Controller/N Gain/Internal Parameters'
 * '<S110>' : 'AutoPIDTuner/PID Controller/P Copy/Disabled'
 * '<S111>' : 'AutoPIDTuner/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S112>' : 'AutoPIDTuner/PID Controller/Reset Signal/Disabled'
 * '<S113>' : 'AutoPIDTuner/PID Controller/Saturation/Passthrough'
 * '<S114>' : 'AutoPIDTuner/PID Controller/Saturation Fdbk/Disabled'
 * '<S115>' : 'AutoPIDTuner/PID Controller/Sum/Sum_PID'
 * '<S116>' : 'AutoPIDTuner/PID Controller/Sum Fdbk/Disabled'
 * '<S117>' : 'AutoPIDTuner/PID Controller/Tracking Mode/Disabled'
 * '<S118>' : 'AutoPIDTuner/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S119>' : 'AutoPIDTuner/PID Controller/Tsamp - Integral/Passthrough'
 * '<S120>' : 'AutoPIDTuner/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S121>' : 'AutoPIDTuner/PID Controller/postSat Signal/Forward_Path'
 * '<S122>' : 'AutoPIDTuner/PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_AutoPIDTuner_h_ */
