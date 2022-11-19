/*
 * AutoPIDTuner.cpp
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
#include "rtwtypes.h"
#include <cstring>
#include <cmath>
#include "AutoPIDTuner_private.h"
#include <emmintrin.h>
#include "rt_defines.h"
#include "zero_crossing_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
void AutoPIDTuner::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3]{
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3]{
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t { rtsiGetT(si) };

  time_T tnew { rtsiGetSolverStopTime(si) };

  time_T h { rtsiGetStepSize(si) };

  real_T *x { rtsiGetContStates(si) };

  ODE3_IntgData *id { static_cast<ODE3_IntgData *>(rtsiGetSolverData(si)) };

  real_T *y { id->y };

  real_T *f0 { id->f[0] };

  real_T *f1 { id->f[1] };

  real_T *f2 { id->f[2] };

  real_T hB[3];
  int_T i;
  int_T nXc { 2 };

  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) std::memcpy(y, x,
                     static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  AutoPIDTuner_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  this->step();
  AutoPIDTuner_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  this->step();
  AutoPIDTuner_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Function for MATLAB Function: '<S19>/RLS' */
real_T AutoPIDTuner::AutoPIDTuner_xnrm2(int32_T n, const real_T x[12], int32_T
  ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd_snf(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0) * a;
  } else if (std::isnan(b)) {
    y = (rtNaN);
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S19>/RLS' */
real_T AutoPIDTuner::AutoPIDTuner_qrFactor(const real_T A[11], const real_T S
  [121], real_T Ns)
{
  real_T b_A[12];
  real_T atmp;
  real_T b_S;
  real_T s;
  int32_T aoffset;
  for (int32_T i{0}; i < 11; i++) {
    aoffset = i * 11;
    s = 0.0;
    for (int32_T k{0}; k < 11; k++) {
      s += S[aoffset + k] * A[k];
    }

    b_A[i] = s;
  }

  b_A[11] = Ns;
  atmp = b_A[0];
  s = AutoPIDTuner_xnrm2(11, b_A, 2);
  if (s != 0.0) {
    s = rt_hypotd_snf(b_A[0], s);
    if (b_A[0] >= 0.0) {
      s = -s;
    }

    if (std::abs(s) < 1.0020841800044864E-292) {
      aoffset = 0;
      do {
        aoffset++;
        for (int32_T k{0}; k <= 8; k += 2) {
          __m128d tmp;
          tmp = _mm_loadu_pd(&b_A[k + 1]);
          tmp = _mm_mul_pd(tmp, _mm_set1_pd(9.9792015476736E+291));
          _mm_storeu_pd(&b_A[k + 1], tmp);
        }

        for (int32_T k{10}; k < 11; k++) {
          b_A[k + 1] *= 9.9792015476736E+291;
        }

        s *= 9.9792015476736E+291;
        atmp *= 9.9792015476736E+291;
      } while ((std::abs(s) < 1.0020841800044864E-292) && (aoffset < 20));

      s = rt_hypotd_snf(atmp, AutoPIDTuner_xnrm2(11, b_A, 2));
      if (atmp >= 0.0) {
        s = -s;
      }

      for (int32_T k{0}; k < aoffset; k++) {
        s *= 1.0020841800044864E-292;
      }

      atmp = s;
    } else {
      atmp = s;
    }
  }

  b_A[0] = atmp;
  b_S = b_A[0];
  return b_S;
}

/* Function for MATLAB Function: '<S19>/RLS' */
void AutoPIDTuner::AutoPIDTuner_trisolve(real_T A, real_T B[11])
{
  for (int32_T j{0}; j < 11; j++) {
    if (B[j] != 0.0) {
      B[j] /= A;
    }
  }
}

/* Function for MATLAB Function: '<S19>/RLS' */
real_T AutoPIDTuner::AutoPIDTuner_xnrm2_o(int32_T n, const real_T x[132],
  int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S19>/RLS' */
void AutoPIDTuner::AutoPIDTuner_xgemv(int32_T m, int32_T n, const real_T A[132],
  int32_T ia0, const real_T x[132], int32_T ix0, real_T y[11])
{
  if ((m != 0) && (n != 0)) {
    int32_T b;
    int32_T iyend;
    iyend = n;
    if (iyend - 1 >= 0) {
      std::memset(&y[0], 0, static_cast<uint32_T>(iyend) * sizeof(real_T));
    }

    b = (n - 1) * 12 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= b; b_iy += 12) {
      real_T c;
      int32_T d;
      int32_T ix;
      ix = ix0 - 1;
      c = 0.0;
      d = b_iy + m;
      for (iyend = b_iy; iyend < d; iyend++) {
        c += x[(ix + iyend) - b_iy] * A[iyend - 1];
      }

      y[div_nde_s32_floor(b_iy - ia0, 12)] = y[div_nde_s32_floor(b_iy - ia0, 12)]
        + c;
    }
  }
}

/* Function for MATLAB Function: '<S19>/RLS' */
void AutoPIDTuner::AutoPIDTuner_xgerc(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[11], real_T A[132], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T jA;
    jA = ia0;
    for (int32_T j{0}; j < n; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T b;
        int32_T ix;
        temp *= alpha1;
        ix = ix0 - 1;
        b = m + jA;
        for (int32_T ijA{jA}; ijA < b; ijA++) {
          A[ijA - 1] += A[(ix + ijA) - jA] * temp;
        }
      }

      jA += 12;
    }
  }
}

/* Function for MATLAB Function: '<S19>/RLS' */
void AutoPIDTuner::AutoPIDTu_sqrtMeasurementUpdate(real_T L[121], const real_T
  H[11], real_T a0, real_T K[11])
{
  __m128d tmp;
  real_T c_A[132];
  real_T A[121];
  real_T y[121];
  real_T B[11];
  real_T Pxy[11];
  real_T prodVal;
  real_T s;
  int32_T aoffset;
  int32_T coffset;
  int32_T ii;
  int32_T lastv;
  for (int32_T b_i{0}; b_i < 11; b_i++) {
    Pxy[b_i] = 0.0;
    for (ii = 0; ii < 11; ii++) {
      s = Pxy[b_i];
      A[b_i + 11 * ii] = 0.0;
      for (coffset = 0; coffset < 11; coffset++) {
        prodVal = A[11 * ii + b_i];
        prodVal += L[11 * coffset + b_i] * L[11 * coffset + ii];
        A[b_i + 11 * ii] = prodVal;
      }

      if (std::isinf(A[11 * ii + b_i])) {
        prodVal = A[11 * ii + b_i];
        if (std::isnan(prodVal)) {
          prodVal = (rtNaN);
        } else if (prodVal < 0.0) {
          prodVal = -1.0;
        } else {
          prodVal = (prodVal > 0.0);
        }

        A[b_i + 11 * ii] = prodVal * 1.7976931348623157E+308;
      }

      prodVal = A[11 * ii + b_i] * H[ii];
      if (std::isinf(prodVal)) {
        if (std::isnan(prodVal)) {
          prodVal = (rtNaN);
        } else if (prodVal < 0.0) {
          prodVal = -1.0;
        } else {
          prodVal = (prodVal > 0.0);
        }

        prodVal *= 1.7976931348623157E+308;
      }

      prodVal += s;
      if (std::isinf(prodVal)) {
        if (std::isnan(prodVal)) {
          prodVal = (rtNaN);
        } else if (prodVal < 0.0) {
          prodVal = -1.0;
        } else {
          prodVal = (prodVal > 0.0);
        }

        prodVal *= 1.7976931348623157E+308;
      }

      s = prodVal;
      Pxy[b_i] = s;
    }
  }

  prodVal = AutoPIDTuner_qrFactor(H, L, std::sqrt(a0));
  std::memcpy(&B[0], &Pxy[0], 11U * sizeof(real_T));
  AutoPIDTuner_trisolve(prodVal, B);
  std::memcpy(&K[0], &B[0], 11U * sizeof(real_T));
  AutoPIDTuner_trisolve(prodVal, K);
  for (int32_T b_i{0}; b_i < 11; b_i++) {
    s = K[b_i];
    if (std::isinf(s)) {
      if (std::isnan(s)) {
        prodVal = (rtNaN);
      } else if (s < 0.0) {
        prodVal = -1.0;
      } else {
        prodVal = (s > 0.0);
      }

      s = prodVal * 1.7976931348623157E+308;
    }

    Pxy[b_i] = -s;
    K[b_i] = s;
  }

  for (coffset = 0; coffset < 11; coffset++) {
    for (int32_T b_i{0}; b_i <= 8; b_i += 2) {
      tmp = _mm_loadu_pd(&Pxy[b_i]);
      tmp = _mm_mul_pd(tmp, _mm_set1_pd(H[coffset]));
      _mm_storeu_pd(&A[b_i + 11 * coffset], tmp);
    }

    for (int32_T b_i{10}; b_i < 11; b_i++) {
      A[b_i + 11 * coffset] = Pxy[b_i] * H[coffset];
    }
  }

  for (int32_T b_i{0}; b_i < 11; b_i++) {
    prodVal = A[11 * b_i + b_i];
    prodVal++;
    A[b_i + 11 * b_i] = prodVal;
  }

  prodVal = std::sqrt(a0);
  for (int32_T b_i{0}; b_i < 11; b_i++) {
    coffset = b_i * 11;
    for (ii = 0; ii < 11; ii++) {
      aoffset = ii * 11;
      s = 0.0;
      for (lastv = 0; lastv < 11; lastv++) {
        s += A[lastv * 11 + b_i] * L[aoffset + lastv];
      }

      y[coffset + ii] = s;
      c_A[ii + 12 * b_i] = y[11 * b_i + ii];
    }

    c_A[12 * b_i + 11] = K[b_i] * prodVal;
    Pxy[b_i] = 0.0;
  }

  for (int32_T b_i{0}; b_i < 11; b_i++) {
    int32_T d;
    ii = b_i * 12 + b_i;
    s = c_A[ii];
    lastv = ii + 2;
    B[b_i] = 0.0;
    prodVal = AutoPIDTuner_xnrm2_o(11 - b_i, c_A, ii + 2);
    if (prodVal != 0.0) {
      prodVal = rt_hypotd_snf(c_A[ii], prodVal);
      if (c_A[ii] >= 0.0) {
        prodVal = -prodVal;
      }

      if (std::abs(prodVal) < 1.0020841800044864E-292) {
        int32_T scalarLB;
        int32_T vectorUB;
        coffset = 0;
        do {
          coffset++;
          d = (ii - b_i) + 12;
          scalarLB = ((((d - lastv) + 1) / 2) << 1) + lastv;
          vectorUB = scalarLB - 2;
          for (aoffset = lastv; aoffset <= vectorUB; aoffset += 2) {
            tmp = _mm_loadu_pd(&c_A[aoffset - 1]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(9.9792015476736E+291));
            _mm_storeu_pd(&c_A[aoffset - 1], tmp);
          }

          for (aoffset = scalarLB; aoffset <= d; aoffset++) {
            c_A[aoffset - 1] *= 9.9792015476736E+291;
          }

          prodVal *= 9.9792015476736E+291;
          s *= 9.9792015476736E+291;
        } while ((std::abs(prodVal) < 1.0020841800044864E-292) && (coffset < 20));

        prodVal = rt_hypotd_snf(s, AutoPIDTuner_xnrm2_o(11 - b_i, c_A, ii + 2));
        if (s >= 0.0) {
          prodVal = -prodVal;
        }

        B[b_i] = (prodVal - s) / prodVal;
        s = 1.0 / (s - prodVal);
        d = (ii - b_i) + 12;
        scalarLB = ((((d - lastv) + 1) / 2) << 1) + lastv;
        vectorUB = scalarLB - 2;
        for (aoffset = lastv; aoffset <= vectorUB; aoffset += 2) {
          tmp = _mm_loadu_pd(&c_A[aoffset - 1]);
          tmp = _mm_mul_pd(tmp, _mm_set1_pd(s));
          _mm_storeu_pd(&c_A[aoffset - 1], tmp);
        }

        for (aoffset = scalarLB; aoffset <= d; aoffset++) {
          c_A[aoffset - 1] *= s;
        }

        for (lastv = 0; lastv < coffset; lastv++) {
          prodVal *= 1.0020841800044864E-292;
        }

        s = prodVal;
      } else {
        int32_T scalarLB;
        int32_T vectorUB;
        B[b_i] = (prodVal - c_A[ii]) / prodVal;
        s = 1.0 / (c_A[ii] - prodVal);
        aoffset = (ii - b_i) + 12;
        scalarLB = ((((aoffset - lastv) + 1) / 2) << 1) + lastv;
        vectorUB = scalarLB - 2;
        for (coffset = lastv; coffset <= vectorUB; coffset += 2) {
          tmp = _mm_loadu_pd(&c_A[coffset - 1]);
          tmp = _mm_mul_pd(tmp, _mm_set1_pd(s));
          _mm_storeu_pd(&c_A[coffset - 1], tmp);
        }

        for (coffset = scalarLB; coffset <= aoffset; coffset++) {
          c_A[coffset - 1] *= s;
        }

        s = prodVal;
      }
    }

    c_A[ii] = s;
    if (b_i + 1 < 11) {
      prodVal = c_A[ii];
      c_A[ii] = 1.0;
      if (B[b_i] != 0.0) {
        boolean_T exitg2;
        lastv = 12 - b_i;
        coffset = (ii - b_i) + 11;
        while ((lastv > 0) && (c_A[coffset] == 0.0)) {
          lastv--;
          coffset--;
        }

        coffset = 10 - b_i;
        exitg2 = false;
        while ((!exitg2) && (coffset > 0)) {
          int32_T exitg1;
          aoffset = ((coffset - 1) * 12 + ii) + 12;
          d = aoffset;
          do {
            exitg1 = 0;
            if (d + 1 <= aoffset + lastv) {
              if (c_A[d] != 0.0) {
                exitg1 = 1;
              } else {
                d++;
              }
            } else {
              coffset--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        coffset = 0;
      }

      if (lastv > 0) {
        AutoPIDTuner_xgemv(lastv, coffset, c_A, ii + 13, c_A, ii + 1, Pxy);
        AutoPIDTuner_xgerc(lastv, coffset, -B[b_i], ii + 1, Pxy, c_A, ii + 13);
      }

      c_A[ii] = prodVal;
    }
  }

  for (int32_T b_i{0}; b_i < 11; b_i++) {
    for (ii = 0; ii <= b_i; ii++) {
      A[ii + 11 * b_i] = c_A[12 * b_i + ii];
    }

    for (ii = b_i + 2; ii < 12; ii++) {
      A[(ii + 11 * b_i) - 1] = 0.0;
    }
  }

  for (coffset = 0; coffset < 11; coffset++) {
    for (int32_T b_i{0}; b_i < 11; b_i++) {
      L[b_i + 11 * coffset] = A[11 * b_i + coffset];
    }
  }

  for (ii = 0; ii < 11; ii++) {
    for (int32_T b_i{0}; b_i < 11; b_i++) {
      if (std::isinf(L[11 * ii + b_i])) {
        prodVal = L[11 * ii + b_i];
        if (std::isnan(prodVal)) {
          prodVal = (rtNaN);
        } else if (prodVal < 0.0) {
          prodVal = -1.0;
        } else {
          prodVal = (prodVal > 0.0);
        }

        L[b_i + 11 * ii] = prodVal * 1.7976931348623157E+308;
      }
    }
  }
}

/*
 * System reset for atomic system:
 *    '<S19>/RLS'
 *    '<S20>/RLS'
 */
void AutoPIDTuner::AutoPIDTuner_RLS_Reset(DW_RLS_AutoPIDTuner_T *localDW)
{
  localDW->rlsEstimator_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S19>/RLS'
 *    '<S20>/RLS'
 */
void AutoPIDTuner::AutoPIDTuner_RLS(const real_T rtu_H[5], const real_T rtu_H_i
  [5], real_T rtu_H_ip, real_T rtu_y, boolean_T rtu_isEnabled, real_T rtu_adg1,
  real_T rtu_yBuffer, real_T rtu_HBuffer, const real_T rtu_x[11], const real_T
  rtu_L[121], B_RLS_AutoPIDTuner_T *localB, DW_RLS_AutoPIDTuner_T *localDW)
{
  real_T L[121];
  real_T x[11];
  real_T HBuffer;
  real_T yBuffer;
  int32_T i;
  for (i = 0; i < 5; i++) {
    /* SignalConversion generated from: '<S43>/ SFunction ' */
    localB->TmpSignalConversionAtSFunctionI[i] = rtu_H[i];
    localB->TmpSignalConversionAtSFunctionI[i + 5] = rtu_H_i[i];
  }

  /* SignalConversion generated from: '<S43>/ SFunction ' */
  localB->TmpSignalConversionAtSFunctionI[10] = rtu_H_ip;
  std::memcpy(&localB->L[0], &rtu_L[0], 121U * sizeof(real_T));
  std::memcpy(&localB->x[0], &rtu_x[0], 11U * sizeof(real_T));
  localB->HBuffer = rtu_HBuffer;
  localB->yBuffer = rtu_yBuffer;
  std::memcpy(&L[0], &localB->L[0], 121U * sizeof(real_T));
  std::memcpy(&x[0], &localB->x[0], 11U * sizeof(real_T));
  HBuffer = localB->HBuffer;
  yBuffer = localB->yBuffer;
  std::memcpy(&localB->L[0], &L[0], 121U * sizeof(real_T));
  std::memcpy(&localB->x[0], &x[0], 11U * sizeof(real_T));
  localB->HBuffer = HBuffer;
  localB->yBuffer = yBuffer;

  /* :  if isempty(rlsEstimator) */
  if (!localDW->rlsEstimator_not_empty) {
    /* :  rlsEstimator = controllib.internal.blocks.rls.getEstimator(algorithmParams); */
    localDW->rlsEstimator_not_empty = true;
  }

  /* :  if hasTriggeredReset */
  /* :  [yBuffer,HBuffer,x,L,e] = rlsEstimator.estimate(... */
  /* :      yBuffer, HBuffer, x, L,... % states */
  /* :      y, H,... % new IO data */
  /* :      isEnabled, adg1, adg2, algorithmParams); */
  localDW->rlsEstimator.DataIterator.IteratorPosition = 1;
  i = localDW->rlsEstimator.DataIterator.IteratorPosition + 1;
  if (i > 2) {
    i = 2;
  }

  localDW->rlsEstimator.DataIterator.IteratorPosition = i;
  HBuffer = 0.0;
  for (i = 0; i < 11; i++) {
    HBuffer += localB->TmpSignalConversionAtSFunctionI[i] * localB->x[i];
  }

  HBuffer = rtu_y - HBuffer;
  if (rtu_isEnabled) {
    AutoPIDTu_sqrtMeasurementUpdate(localB->L,
      localB->TmpSignalConversionAtSFunctionI, rtu_adg1, x);
    yBuffer = std::sqrt(rtu_adg1);
    for (i = 0; i < 11; i++) {
      for (int32_T b_kk2{0}; b_kk2 <= i; b_kk2++) {
        int32_T kk2;
        kk2 = (b_kk2 - i) + 10;
        localB->L[kk2 + 11 * (10 - i)] /= yBuffer;
      }
    }

    for (i = 0; i < 11; i++) {
      for (int32_T b_kk2{0}; b_kk2 < 11; b_kk2++) {
        if (std::isinf(localB->L[11 * b_kk2 + i])) {
          yBuffer = localB->L[11 * b_kk2 + i];
          if (std::isnan(yBuffer)) {
            yBuffer = (rtNaN);
          } else if (yBuffer < 0.0) {
            yBuffer = -1.0;
          } else {
            yBuffer = (yBuffer > 0.0);
          }

          localB->L[i + 11 * b_kk2] = yBuffer * 1.7976931348623157E+308;
        }
      }

      localB->x[i] += x[i] * HBuffer;
    }
  }

  localB->e = HBuffer;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = (rtNaN);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_generateTargetLoop(const real_T w[3], real_T
  targetPM, creal_T L[3])
{
  real_T a;
  real_T b;
  real_T b_a;
  real_T b_b;
  real_T c_a;
  real_T c_x;
  real_T d_x;
  real_T theta;
  real_T x;
  theta = 1.5707963267948966 - targetPM / 180.0 * 3.1415926535897931;
  a = std::sin(theta) * std::sin(theta);
  x = -std::sin(theta) * w[1] * w[1];
  c_x = std::cos(theta);
  d_x = std::cos(theta);
  b_a = std::sin(theta) * std::sin(theta);
  c_a = w[1] * w[1] * std::cos(theta) * std::cos(theta);
  theta = -std::cos(theta) * w[1] * w[1] * w[1];
  b = w[0] * w[0];
  b_b = rt_powd_snf(w[0], 3.0);
  L[0].re = x / (w[1] * w[1] * c_x * d_x + a * b);
  L[0].im = theta / (b_a * b_b + c_a * w[0]);
  b = w[1] * w[1];
  b_b = rt_powd_snf(w[1], 3.0);
  L[1].re = x / (w[1] * w[1] * c_x * d_x + a * b);
  L[1].im = theta / (b_a * b_b + c_a * w[1]);
  b = w[2] * w[2];
  b_b = rt_powd_snf(w[2], 3.0);
  L[2].re = x / (w[1] * w[1] * c_x * d_x + a * b);
  L[2].im = theta / (b_a * b_b + c_a * w[2]);
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_logspace(real_T d1, real_T d2, real_T y[50])
{
  real_T b_y[50];
  b_y[49] = d2;
  b_y[0] = d1;
  if (d1 == -d2) {
    real_T delta1;
    delta1 = d2 / 49.0;
    for (int32_T c_k{0}; c_k < 48; c_k++) {
      b_y[c_k + 1] = ((static_cast<real_T>(c_k) + 2.0) * 2.0 - 51.0) * delta1;
    }
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((std::abs(d1) >
               8.9884656743115785E+307) || (std::abs(d2) >
               8.9884656743115785E+307))) {
    real_T delta1;
    real_T delta2;
    delta1 = d1 / 49.0;
    delta2 = d2 / 49.0;
    for (int32_T c_k{0}; c_k < 48; c_k++) {
      b_y[c_k + 1] = ((static_cast<real_T>(c_k) + 1.0) * delta2 + d1) - (
        static_cast<real_T>(c_k) + 1.0) * delta1;
    }
  } else {
    real_T delta1;
    delta1 = (d2 - d1) / 49.0;
    for (int32_T c_k{0}; c_k < 48; c_k++) {
      b_y[c_k + 1] = (static_cast<real_T>(c_k) + 1.0) * delta1 + d1;
    }
  }

  for (int32_T c_k{0}; c_k < 50; c_k++) {
    y[c_k] = rt_powd_snf(10.0, b_y[c_k]);
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_pchip(const real_T x[5], const real_T y[5],
  const real_T xx[100], real_T v[100])
{
  real_T pp_coefs[16];
  real_T c_s_idx_1;
  real_T c_s_idx_2;
  real_T c_s_idx_3;
  real_T del_idx_0;
  real_T del_idx_1;
  real_T del_idx_2;
  real_T del_idx_3;
  real_T dzzdx;
  real_T h_idx_0;
  real_T h_idx_1;
  real_T h_idx_2;
  real_T h_idx_3;
  real_T signd1;
  real_T u;
  real_T y_0;
  h_idx_3 = x[1] - x[0];
  del_idx_0 = (y[1] - y[0]) / h_idx_3;
  h_idx_0 = h_idx_3;
  h_idx_3 = x[2] - x[1];
  del_idx_1 = (y[2] - y[1]) / h_idx_3;
  h_idx_1 = h_idx_3;
  h_idx_3 = x[3] - x[2];
  del_idx_2 = (y[3] - y[2]) / h_idx_3;
  h_idx_2 = h_idx_3;
  h_idx_3 = x[4] - x[3];
  del_idx_3 = (y[4] - y[3]) / h_idx_3;
  dzzdx = 2.0 * h_idx_1 + h_idx_0;
  signd1 = 2.0 * h_idx_0 + h_idx_1;
  c_s_idx_1 = 0.0;
  u = del_idx_0 * del_idx_1;
  if (std::isnan(u)) {
    y_0 = (rtNaN);
  } else if (u < 0.0) {
    y_0 = -1.0;
  } else {
    y_0 = (u > 0.0);
  }

  if (y_0 > 0.0) {
    c_s_idx_1 = (dzzdx + signd1) / (dzzdx / del_idx_0 + signd1 / del_idx_1);
  }

  dzzdx = 2.0 * h_idx_2 + h_idx_1;
  signd1 = 2.0 * h_idx_1 + h_idx_2;
  c_s_idx_2 = 0.0;
  u = del_idx_1 * del_idx_2;
  if (std::isnan(u)) {
    y_0 = (rtNaN);
  } else if (u < 0.0) {
    y_0 = -1.0;
  } else {
    y_0 = (u > 0.0);
  }

  if (y_0 > 0.0) {
    c_s_idx_2 = (dzzdx + signd1) / (dzzdx / del_idx_1 + signd1 / del_idx_2);
  }

  dzzdx = 2.0 * h_idx_3 + h_idx_2;
  signd1 = 2.0 * h_idx_2 + h_idx_3;
  c_s_idx_3 = 0.0;
  u = del_idx_2 * del_idx_3;
  if (std::isnan(u)) {
    y_0 = (rtNaN);
  } else if (u < 0.0) {
    y_0 = -1.0;
  } else {
    y_0 = (u > 0.0);
  }

  if (y_0 > 0.0) {
    c_s_idx_3 = (dzzdx + signd1) / (dzzdx / del_idx_2 + signd1 / del_idx_3);
  }

  dzzdx = ((2.0 * h_idx_0 + h_idx_1) * del_idx_0 - h_idx_0 * del_idx_1) /
    (h_idx_0 + h_idx_1);
  if (std::isnan(del_idx_0)) {
    signd1 = (rtNaN);
  } else if (del_idx_0 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_0 > 0.0);
  }

  if (std::isnan(dzzdx)) {
    y_0 = (rtNaN);
  } else if (dzzdx < 0.0) {
    y_0 = -1.0;
  } else {
    y_0 = (dzzdx > 0.0);
  }

  if (y_0 != signd1) {
    dzzdx = 0.0;
  } else {
    if (std::isnan(del_idx_1)) {
      y_0 = (rtNaN);
    } else if (del_idx_1 < 0.0) {
      y_0 = -1.0;
    } else {
      y_0 = (del_idx_1 > 0.0);
    }

    if ((signd1 != y_0) && (std::abs(dzzdx) > std::abs(3.0 * del_idx_0))) {
      dzzdx = 3.0 * del_idx_0;
    }
  }

  u = dzzdx;
  dzzdx = ((2.0 * h_idx_3 + h_idx_2) * del_idx_3 - del_idx_2 * h_idx_3) /
    (h_idx_2 + h_idx_3);
  if (std::isnan(del_idx_3)) {
    signd1 = (rtNaN);
  } else if (del_idx_3 < 0.0) {
    signd1 = -1.0;
  } else {
    signd1 = (del_idx_3 > 0.0);
  }

  if (std::isnan(dzzdx)) {
    y_0 = (rtNaN);
  } else if (dzzdx < 0.0) {
    y_0 = -1.0;
  } else {
    y_0 = (dzzdx > 0.0);
  }

  if (y_0 != signd1) {
    dzzdx = 0.0;
  } else {
    if (std::isnan(del_idx_2)) {
      y_0 = (rtNaN);
    } else if (del_idx_2 < 0.0) {
      y_0 = -1.0;
    } else {
      y_0 = (del_idx_2 > 0.0);
    }

    if ((signd1 != y_0) && (std::abs(dzzdx) > std::abs(3.0 * del_idx_3))) {
      dzzdx = 3.0 * del_idx_3;
    }
  }

  y_0 = dzzdx;
  dzzdx = (del_idx_0 - u) / h_idx_0;
  signd1 = (c_s_idx_1 - del_idx_0) / h_idx_0;
  pp_coefs[0] = (signd1 - dzzdx) / h_idx_0;
  pp_coefs[4] = 2.0 * dzzdx - signd1;
  pp_coefs[8] = u;
  pp_coefs[12] = y[0];
  h_idx_0 = h_idx_1;
  del_idx_0 = del_idx_1;
  dzzdx = (del_idx_0 - c_s_idx_1) / h_idx_0;
  signd1 = (c_s_idx_2 - del_idx_0) / h_idx_0;
  pp_coefs[1] = (signd1 - dzzdx) / h_idx_0;
  pp_coefs[5] = 2.0 * dzzdx - signd1;
  pp_coefs[9] = c_s_idx_1;
  pp_coefs[13] = y[1];
  h_idx_0 = h_idx_2;
  del_idx_0 = del_idx_2;
  dzzdx = (del_idx_0 - c_s_idx_2) / h_idx_0;
  signd1 = (c_s_idx_3 - del_idx_0) / h_idx_0;
  pp_coefs[2] = (signd1 - dzzdx) / h_idx_0;
  pp_coefs[6] = 2.0 * dzzdx - signd1;
  pp_coefs[10] = c_s_idx_2;
  pp_coefs[14] = y[2];
  h_idx_0 = h_idx_3;
  del_idx_0 = del_idx_3;
  dzzdx = (del_idx_0 - c_s_idx_3) / h_idx_0;
  signd1 = (y_0 - del_idx_0) / h_idx_0;
  pp_coefs[3] = (signd1 - dzzdx) / h_idx_0;
  pp_coefs[7] = 2.0 * dzzdx - signd1;
  pp_coefs[11] = c_s_idx_3;
  pp_coefs[15] = y[3];
  for (int32_T k{0}; k < 100; k++) {
    if (std::isnan(xx[k])) {
      signd1 = xx[k];
    } else {
      int32_T high_i;
      int32_T low_i;
      int32_T low_ip1;
      low_i = 0;
      low_ip1 = 2;
      high_i = 5;
      while (high_i > low_ip1) {
        int32_T mid_i;
        mid_i = ((low_i + high_i) + 1) >> 1;
        if (xx[k] >= x[mid_i - 1]) {
          low_i = mid_i - 1;
          low_ip1 = mid_i + 1;
        } else {
          high_i = mid_i;
        }
      }

      dzzdx = xx[k] - x[low_i];
      signd1 = pp_coefs[low_i];
      signd1 = dzzdx * signd1 + pp_coefs[low_i + 4];
      signd1 = dzzdx * signd1 + pp_coefs[low_i + 8];
      signd1 = dzzdx * signd1 + pp_coefs[low_i + 12];
    }

    v[k] = signd1;
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_localGetRealImag(real_T Ts, const real_T w[3],
  uint16_T Formula, real_T *realX, real_T imagX[3])
{
  real_T Ts_0;
  real_T imagX_0;
  real_T w_0;
  if (Formula == 1) {
    *realX = -Ts / 2.0;
  } else if (Formula == 2) {
    *realX = Ts / 2.0;
  } else {
    *realX = 0.0;
  }

  Ts_0 = -Ts / 2.0;
  w_0 = w[0];
  imagX_0 = w_0 * Ts;
  w_0 *= Ts;
  w_0 = std::cos(w_0);
  w_0 = std::fmin(w_0, 0.999999999999999);
  imagX_0 = std::sin(imagX_0);
  imagX_0 = Ts_0 * imagX_0 / (1.0 - w_0);
  imagX[0] = imagX_0;
  w_0 = w[1];
  imagX_0 = w_0 * Ts;
  w_0 *= Ts;
  w_0 = std::cos(w_0);
  w_0 = std::fmin(w_0, 0.999999999999999);
  imagX_0 = std::sin(imagX_0);
  imagX_0 = Ts_0 * imagX_0 / (1.0 - w_0);
  imagX[1] = imagX_0;
  w_0 = w[2];
  imagX_0 = w_0 * Ts;
  w_0 *= Ts;
  w_0 = std::cos(w_0);
  w_0 = std::fmin(w_0, 0.999999999999999);
  imagX_0 = std::sin(imagX_0);
  imagX_0 = Ts_0 * imagX_0 / (1.0 - w_0);
  imagX[2] = imagX_0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_blkdiag(const real_T varargin_1[6], real_T y[14])
{
  std::memset(&y[0], 0, 14U * sizeof(real_T));
  for (int32_T i{0}; i < 6; i++) {
    y[i] = varargin_1[i];
  }

  y[13] = 1.0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_minimum(const real_T x[50])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 51)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 51; k++) {
      if (ex > x[k - 1]) {
        ex = x[k - 1];
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
boolean_T AutoPIDTuner::AutoPIDTuner_vectorAny(const boolean_T x_data[], const
  int32_T *x_size)
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= *x_size - 1)) {
    boolean_T b;
    b = !x_data[k];
    if (!b) {
      y = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_xnrm2_b(int32_T n, const real_T x[49], int32_T
  ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgemv_f(int32_T m, int32_T n, const real_T A[49],
  int32_T ia0, const real_T x[49], int32_T ix0, real_T y[7])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    b = static_cast<uint8_T>(n);
    std::memset(&y[0], 0, static_cast<uint32_T>(b) * sizeof(real_T));
    d = (n - 1) * 7 + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 7) {
      real_T c;
      int32_T e;
      int32_T ix;
      ix = ix0 - 1;
      c = 0.0;
      e = b_iy + m;
      for (b = b_iy; b < e; b++) {
        c += x[(ix + b) - b_iy] * A[b - 1];
      }

      y[div_nde_s32_floor(b_iy - ia0, 7)] = y[div_nde_s32_floor(b_iy - ia0, 7)]
        + c;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgerc_c(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[7], real_T A[49], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        int32_T ix;
        temp *= alpha1;
        ix = ix0 - 1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[(ix + ijA) - jA] * temp;
        }
      }

      jA += 7;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgeqp3(real_T A[49], real_T tau[7], int32_T
  jpvt[7])
{
  real_T vn1[7];
  real_T vn2[7];
  real_T work[7];
  real_T scale;
  real_T smax;
  int32_T ii;
  int32_T pvt;
  for (int32_T k{0}; k < 7; k++) {
    jpvt[k] = k + 1;
    tau[k] = 0.0;
    work[k] = 0.0;
    pvt = k * 7 + 1;
    smax = 0.0;
    scale = 3.3121686421112381E-170;
    for (ii = pvt; ii <= pvt + 6; ii++) {
      real_T absxk;
      absxk = std::abs(A[ii - 1]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        smax = smax * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        smax += t * t;
      }
    }

    smax = scale * std::sqrt(smax);
    vn2[k] = smax;
    vn1[k] = smax;
  }

  for (int32_T k{0}; k < 7; k++) {
    int32_T b_ix;
    int32_T itemp;
    int32_T ix;
    ii = k * 7 + k;
    itemp = 7 - k;
    b_ix = 0;
    if (7 - k > 1) {
      ix = k;
      smax = std::abs(vn1[k]);
      for (pvt = 2; pvt <= itemp; pvt++) {
        ix++;
        scale = std::abs(vn1[ix]);
        if (scale > smax) {
          b_ix = pvt - 1;
          smax = scale;
        }
      }
    }

    pvt = k + b_ix;
    if (pvt != k) {
      b_ix = pvt * 7;
      ix = k * 7;
      for (itemp = 0; itemp < 7; itemp++) {
        smax = A[b_ix];
        A[b_ix] = A[ix];
        A[ix] = smax;
        b_ix++;
        ix++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[k];
      jpvt[k] = itemp;
      vn1[pvt] = vn1[k];
      vn2[pvt] = vn2[k];
    }

    if (k + 1 < 7) {
      scale = A[ii];
      pvt = ii + 2;
      tau[k] = 0.0;
      smax = AutoPIDTuner_xnrm2_b(6 - k, A, ii + 2);
      if (smax != 0.0) {
        smax = rt_hypotd_snf(A[ii], smax);
        if (A[ii] >= 0.0) {
          smax = -smax;
        }

        if (std::abs(smax) < 1.0020841800044864E-292) {
          __m128d tmp;
          int32_T scalarLB;
          int32_T vectorUB;
          itemp = 0;
          do {
            itemp++;
            ix = (ii - k) + 7;
            scalarLB = ((((ix - pvt) + 1) / 2) << 1) + pvt;
            vectorUB = scalarLB - 2;
            for (b_ix = pvt; b_ix <= vectorUB; b_ix += 2) {
              tmp = _mm_loadu_pd(&A[b_ix - 1]);
              tmp = _mm_mul_pd(tmp, _mm_set1_pd(9.9792015476736E+291));
              _mm_storeu_pd(&A[b_ix - 1], tmp);
            }

            for (b_ix = scalarLB; b_ix <= ix; b_ix++) {
              A[b_ix - 1] *= 9.9792015476736E+291;
            }

            smax *= 9.9792015476736E+291;
            scale *= 9.9792015476736E+291;
          } while ((std::abs(smax) < 1.0020841800044864E-292) && (itemp < 20));

          smax = rt_hypotd_snf(scale, AutoPIDTuner_xnrm2_b(6 - k, A, ii + 2));
          if (scale >= 0.0) {
            smax = -smax;
          }

          tau[k] = (smax - scale) / smax;
          scale = 1.0 / (scale - smax);
          ix = (ii - k) + 7;
          scalarLB = ((((ix - pvt) + 1) / 2) << 1) + pvt;
          vectorUB = scalarLB - 2;
          for (b_ix = pvt; b_ix <= vectorUB; b_ix += 2) {
            tmp = _mm_loadu_pd(&A[b_ix - 1]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(scale));
            _mm_storeu_pd(&A[b_ix - 1], tmp);
          }

          for (b_ix = scalarLB; b_ix <= ix; b_ix++) {
            A[b_ix - 1] *= scale;
          }

          for (pvt = 0; pvt < itemp; pvt++) {
            smax *= 1.0020841800044864E-292;
          }

          scale = smax;
        } else {
          int32_T scalarLB;
          int32_T vectorUB;
          tau[k] = (smax - A[ii]) / smax;
          scale = 1.0 / (A[ii] - smax);
          b_ix = (ii - k) + 7;
          scalarLB = ((((b_ix - pvt) + 1) / 2) << 1) + pvt;
          vectorUB = scalarLB - 2;
          for (itemp = pvt; itemp <= vectorUB; itemp += 2) {
            __m128d tmp;
            tmp = _mm_loadu_pd(&A[itemp - 1]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(scale));
            _mm_storeu_pd(&A[itemp - 1], tmp);
          }

          for (itemp = scalarLB; itemp <= b_ix; itemp++) {
            A[itemp - 1] *= scale;
          }

          scale = smax;
        }
      }

      A[ii] = scale;
      smax = A[ii];
      A[ii] = 1.0;
      if (tau[k] != 0.0) {
        boolean_T exitg2;
        pvt = 7 - k;
        itemp = (ii - k) + 6;
        while ((pvt > 0) && (A[itemp] == 0.0)) {
          pvt--;
          itemp--;
        }

        itemp = 6 - k;
        exitg2 = false;
        while ((!exitg2) && (itemp > 0)) {
          int32_T exitg1;
          b_ix = ((itemp - 1) * 7 + ii) + 7;
          ix = b_ix;
          do {
            exitg1 = 0;
            if (ix + 1 <= b_ix + pvt) {
              if (A[ix] != 0.0) {
                exitg1 = 1;
              } else {
                ix++;
              }
            } else {
              itemp--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        pvt = 0;
        itemp = 0;
      }

      if (pvt > 0) {
        AutoPIDTuner_xgemv_f(pvt, itemp, A, ii + 8, A, ii + 1, work);
        AutoPIDTuner_xgerc_c(pvt, itemp, -tau[k], ii + 1, work, A, ii + 8);
      }

      A[ii] = smax;
    } else {
      tau[6] = 0.0;
    }

    for (ii = k + 2; ii < 8; ii++) {
      pvt = (ii - 1) * 7 + k;
      if (vn1[ii - 1] != 0.0) {
        smax = std::abs(A[pvt]) / vn1[ii - 1];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        scale = vn1[ii - 1] / vn2[ii - 1];
        scale = scale * scale * smax;
        if (scale <= 1.4901161193847656E-8) {
          if (k + 1 < 7) {
            vn1[ii - 1] = AutoPIDTuner_xnrm2_b(6 - k, A, pvt + 2);
            vn2[ii - 1] = vn1[ii - 1];
          } else {
            vn1[ii - 1] = 0.0;
            vn2[ii - 1] = 0.0;
          }
        } else {
          vn1[ii - 1] *= std::sqrt(smax);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xorgqr(int32_T k, real_T A[49], const real_T
  tau[7])
{
  real_T work[7];
  int32_T ia;
  int32_T itau;
  for (itau = k; itau < 7; itau++) {
    ia = itau * 7;
    for (int32_T i{0}; i < 7; i++) {
      A[ia + i] = 0.0;
    }

    A[ia + itau] = 1.0;
  }

  itau = k - 1;
  for (int32_T i{0}; i < 7; i++) {
    work[i] = 0.0;
  }

  for (int32_T i{k}; i >= 1; i--) {
    int32_T b_ia;
    int32_T coltop;
    int32_T lastc;
    int32_T lastv;
    ia = ((i - 1) * 7 + i) + 7;
    A[ia - 8] = 1.0;
    if (tau[itau] != 0.0) {
      boolean_T exitg2;
      lastv = 8 - i;
      lastc = ia - i;
      while ((lastv > 0) && (A[lastc - 1] == 0.0)) {
        lastv--;
        lastc--;
      }

      lastc = 7 - i;
      exitg2 = false;
      while ((!exitg2) && (lastc > 0)) {
        int32_T exitg1;
        coltop = (lastc - 1) * 7 + ia;
        b_ia = coltop;
        do {
          exitg1 = 0;
          if (b_ia <= (coltop + lastv) - 1) {
            if (A[b_ia - 1] != 0.0) {
              exitg1 = 1;
            } else {
              b_ia++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = 0;
    }

    if (lastv > 0) {
      AutoPIDTuner_xgemv_f(lastv, lastc, A, ia, A, ia - 7, work);
      AutoPIDTuner_xgerc_c(lastv, lastc, -tau[itau], ia - 7, work, A, ia);
    }

    lastc = ia - i;
    coltop = (((((lastc - ia) + 7) / 2) << 1) + ia) - 6;
    b_ia = coltop - 2;
    for (lastv = ia - 6; lastv <= b_ia; lastv += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&A[lastv - 1]);
      tmp = _mm_mul_pd(tmp, _mm_set1_pd(-tau[itau]));
      _mm_storeu_pd(&A[lastv - 1], tmp);
    }

    for (lastv = coltop; lastv <= lastc; lastv++) {
      A[lastv - 1] *= -tau[itau];
    }

    A[ia - 8] = 1.0 - tau[itau];
    lastc = static_cast<uint8_T>(i - 1);
    for (lastv = 0; lastv < lastc; lastv++) {
      A[(ia - lastv) - 9] = 0.0;
    }

    itau--;
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_computeZ(real_T Z[2], const real_T C[14], const
  real_T d[7], const boolean_T p[2])
{
  real_T Q[49];
  real_T A[14];
  real_T qtd[7];
  real_T z[2];
  int32_T jpvt1[7];
  int32_T jpvt[2];
  int32_T ct;
  int32_T ncols;
  std::memset(&A[0], 0, 14U * sizeof(real_T));
  ncols = -1;
  for (ct = 0; ct < 2; ct++) {
    if (p[ct]) {
      ncols++;
      for (int32_T c_i{0}; c_i < 7; c_i++) {
        A[c_i + 7 * ncols] = C[7 * ct + c_i];
      }
    }
  }

  std::memcpy(&Q[0], &A[0], 14U * sizeof(real_T));
  std::memset(&Q[14], 0, 35U * sizeof(real_T));
  AutoPIDTuner_xgeqp3(Q, qtd, jpvt1);
  for (ct = 0; ct < 2; ct++) {
    jpvt[ct] = jpvt1[ct];
    for (int32_T c_i{0}; c_i <= ct; c_i++) {
      A[c_i + 7 * ct] = Q[7 * ct + c_i];
    }

    for (int32_T c_i{ct + 2}; c_i < 8; c_i++) {
      A[(c_i + 7 * ct) - 1] = 0.0;
    }
  }

  AutoPIDTuner_xorgqr(2, Q, qtd);
  for (int32_T c_i{0}; c_i < 7; c_i++) {
    qtd[c_i] = 0.0;
    for (ct = 0; ct < 7; ct++) {
      real_T qtd_0;
      qtd_0 = qtd[c_i];
      qtd_0 += Q[7 * c_i + ct] * d[ct];
      qtd[c_i] = qtd_0;
    }
  }

  z[0] = 0.0;
  z[1] = 0.0;
  for (ct = 0; ct <= ncols; ct++) {
    z[jpvt[ct] - 1] = qtd[ct];
  }

  for (ct = 0; ct <= ncols; ct++) {
    int32_T j;
    j = ncols - ct;
    z[jpvt[j] - 1] /= A[7 * j + j];
    for (int32_T c_i{0}; c_i < j; c_i++) {
      z[jpvt[0] - 1] -= z[jpvt[j] - 1] * A[7 * j];
    }
  }

  ct = 0;
  if (p[0]) {
    Z[0] = z[0];
    ct = 1;
  }

  if (p[1]) {
    Z[1] = z[ct];
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTun_utilLSQFixedSizeData(const real_T C[14], const
  real_T d[7], real_T x[2])
{
  static const int32_T tmp_1{ 2 };

  __m128d tmp;
  __m128d tmp_0;
  real_T d_0[7];
  real_T w[2];
  real_T z[2];
  real_T s;
  real_T tol;
  real_T w_0;
  real_T wz_idx_0;
  real_T wz_idx_1;
  int32_T j;
  int16_T iter;
  boolean_T P[2];
  boolean_T Z[2];
  boolean_T exitg1;
  wz_idx_0 = 0.0;
  wz_idx_1 = 0.0;
  tol = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 2)) {
    s = 0.0;
    for (int32_T i{0}; i < 7; i++) {
      s += std::abs(C[7 * j + i]);
    }

    if (std::isnan(s)) {
      tol = (rtNaN);
      exitg1 = true;
    } else {
      if (s > tol) {
        tol = s;
      }

      j++;
    }
  }

  tol = 2.2204460492503131E-15 * tol * 7.0;
  P[0] = false;
  Z[0] = true;
  x[0] = 0.0;
  P[1] = false;
  Z[1] = true;
  x[1] = 0.0;
  for (j = 0; j <= 4; j += 2) {
    tmp = _mm_loadu_pd(&C[j]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(0.0));
    tmp_0 = _mm_loadu_pd(&C[j + 7]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&d[j]);
    tmp = _mm_sub_pd(tmp_0, tmp);
    _mm_storeu_pd(&d_0[j], tmp);
  }

  for (j = 6; j < 7; j++) {
    s = C[j] * 0.0;
    s += C[j + 7] * 0.0;
    d_0[j] = d[j] - s;
  }

  for (j = 0; j < 2; j++) {
    w[j] = 0.0;
    for (int32_T i{0}; i < 7; i++) {
      w_0 = w[j];
      w_0 += C[7 * j + i] * d_0[i];
      w[j] = w_0;
    }
  }

  iter = 0;
  exitg1 = false;
  while ((!exitg1) && AutoPIDTuner_vectorAny(Z, &tmp_1)) {
    boolean_T exitg3;
    boolean_T found;
    found = false;
    j = 1;
    exitg3 = false;
    while ((!exitg3) && (j - 1 < 2)) {
      if (Z[static_cast<int16_T>(j) - 1] && (w[static_cast<int16_T>(j) - 1] >
           tol)) {
        found = true;
        exitg3 = true;
      } else {
        j++;
      }
    }

    if (found) {
      int32_T exitg2;
      if (P[0]) {
        wz_idx_0 = -1.7976931348623157E+308;
      }

      if (Z[0]) {
        wz_idx_0 = w[0];
      }

      if (P[1]) {
        wz_idx_1 = -1.7976931348623157E+308;
      }

      if (Z[1]) {
        wz_idx_1 = w[1];
      }

      if ((wz_idx_0 < wz_idx_1) || (std::isnan(wz_idx_0) && (!std::isnan
            (wz_idx_1)))) {
        j = 1;
      } else {
        j = 0;
      }

      P[j] = true;
      Z[j] = false;
      z[0] = 0.0;
      z[1] = 0.0;
      AutoPIDTuner_computeZ(z, C, d, P);
      do {
        exitg2 = 0;
        found = false;
        j = 1;
        exitg3 = false;
        while ((!exitg3) && (j - 1 < 2)) {
          if (P[static_cast<int16_T>(j) - 1] && (z[static_cast<int16_T>(j) - 1] <=
               0.0)) {
            found = true;
            exitg3 = true;
          } else {
            j++;
          }
        }

        if (found) {
          iter = static_cast<int16_T>(iter + 1);
          if (iter > 6) {
            x[0] = z[0];
            x[1] = z[1];
            exitg2 = 1;
          } else {
            real_T x_0;
            boolean_T P_0;
            w[0] = 1.7976931348623157E+308;
            w[1] = 1.7976931348623157E+308;
            if ((z[0] <= 0.0) && P[0]) {
              w[0] = x[0] / (x[0] - z[0]);
            }

            if ((z[1] <= 0.0) && P[1]) {
              w[1] = x[1] / (x[1] - z[1]);
            }

            if ((w[0] > w[1]) || (std::isnan(w[0]) && (!std::isnan(w[1])))) {
              s = w[1];
            } else {
              s = w[0];
            }

            found = Z[0];
            P_0 = P[0];
            w_0 = z[0];
            x_0 = x[0];
            x_0 += (w_0 - x_0) * s;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[0] = x_0;
            z[0] = 0.0;
            P[0] = P_0;
            Z[0] = found;
            found = Z[1];
            P_0 = P[1];
            w_0 = z[1];
            x_0 = x[1];
            x_0 += (w_0 - x_0) * s;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[1] = x_0;
            z[1] = 0.0;
            P[1] = P_0;
            Z[1] = found;
            AutoPIDTuner_computeZ(z, C, d, P);
          }
        } else {
          x[0] = z[0];
          x[1] = z[1];
          for (j = 0; j <= 4; j += 2) {
            tmp = _mm_loadu_pd(&C[j]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(z[0]));
            tmp_0 = _mm_loadu_pd(&C[j + 7]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[1]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&d[j]);
            tmp = _mm_sub_pd(tmp_0, tmp);
            _mm_storeu_pd(&d_0[j], tmp);
          }

          for (j = 6; j < 7; j++) {
            s = C[j] * z[0];
            s += C[j + 7] * z[1];
            d_0[j] = d[j] - s;
          }

          for (j = 0; j < 2; j++) {
            w[j] = 0.0;
            for (int32_T i{0}; i < 7; i++) {
              w_0 = w[j];
              w_0 += C[7 * j + i] * d_0[i];
              w[j] = w_0;
            }
          }

          exitg2 = 2;
        }
      } while (exitg2 == 0);

      if (exitg2 == 1) {
        exitg1 = true;
      }
    } else {
      exitg1 = true;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_blkdiag_c(const real_T varargin_1[12], real_T y
  [21])
{
  std::memset(&y[0], 0, 21U * sizeof(real_T));
  for (int32_T i{0}; i < 6; i++) {
    y[i] = varargin_1[i];
    y[i + 7] = varargin_1[i + 6];
  }

  y[20] = 1.0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_norm(const real_T x[21])
{
  real_T y;
  int32_T j;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 3)) {
    real_T s;
    s = 0.0;
    for (int32_T i{0}; i < 7; i++) {
      s += std::abs(x[7 * j + i]);
    }

    if (std::isnan(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_maximum(const real_T x[3], real_T *ex, int32_T
  *idx)
{
  int32_T b_idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    b_idx = 1;
  } else {
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!std::isnan(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[b_idx - 1];
    *idx = b_idx;
    for (k = b_idx + 1; k < 4; k++) {
      if (*ex < x[k - 1]) {
        *ex = x[k - 1];
        *idx = k;
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_computeZ_j(real_T Z[3], const real_T C[21],
  const real_T d[7], const boolean_T p[3])
{
  real_T Q[49];
  real_T A[21];
  real_T qtd[7];
  real_T z[3];
  int32_T jpvt1[7];
  int32_T jpvt[3];
  int32_T ct;
  int32_T ncols;
  std::memset(&A[0], 0, 21U * sizeof(real_T));
  ncols = -1;
  for (ct = 0; ct < 3; ct++) {
    if (p[ct]) {
      ncols++;
      for (int32_T c_i{0}; c_i < 7; c_i++) {
        A[c_i + 7 * ncols] = C[7 * ct + c_i];
      }
    }
  }

  std::memcpy(&Q[0], &A[0], 21U * sizeof(real_T));
  std::memset(&Q[21], 0, 28U * sizeof(real_T));
  AutoPIDTuner_xgeqp3(Q, qtd, jpvt1);
  for (ct = 0; ct < 3; ct++) {
    jpvt[ct] = jpvt1[ct];
    for (int32_T c_i{0}; c_i <= ct; c_i++) {
      A[c_i + 7 * ct] = Q[7 * ct + c_i];
    }

    for (int32_T c_i{ct + 2}; c_i < 8; c_i++) {
      A[(c_i + 7 * ct) - 1] = 0.0;
    }
  }

  AutoPIDTuner_xorgqr(3, Q, qtd);
  for (int32_T c_i{0}; c_i < 7; c_i++) {
    qtd[c_i] = 0.0;
    for (ct = 0; ct < 7; ct++) {
      real_T qtd_0;
      qtd_0 = qtd[c_i];
      qtd_0 += Q[7 * c_i + ct] * d[ct];
      qtd[c_i] = qtd_0;
    }
  }

  z[0] = 0.0;
  z[1] = 0.0;
  z[2] = 0.0;
  for (ct = 0; ct <= ncols; ct++) {
    z[jpvt[ct] - 1] = qtd[ct];
  }

  for (ct = 0; ct <= ncols; ct++) {
    int32_T j;
    j = ncols - ct;
    z[jpvt[j] - 1] /= A[7 * j + j];
    for (int32_T c_i{0}; c_i < j; c_i++) {
      z[jpvt[c_i] - 1] -= A[7 * j + c_i] * z[jpvt[j] - 1];
    }
  }

  ct = 0;
  if (p[0]) {
    Z[0] = z[0];
    ct = 1;
  }

  if (p[1]) {
    Z[1] = z[ct];
    ct++;
  }

  if (p[2]) {
    Z[2] = z[ct];
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_computeAlpha(const real_T x[3], const real_T
  z[3], const boolean_T Q[3])
{
  real_T b_x[3];
  real_T result;
  int32_T idx;
  int32_T k;
  b_x[0] = 1.7976931348623157E+308;
  b_x[1] = 1.7976931348623157E+308;
  b_x[2] = 1.7976931348623157E+308;
  if (Q[0]) {
    b_x[0] = x[0] / (x[0] - z[0]);
  }

  if (Q[1]) {
    b_x[1] = x[1] / (x[1] - z[1]);
  }

  if (Q[2]) {
    b_x[2] = x[2] / (x[2] - z[2]);
  }

  if (!std::isnan(b_x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 3)) {
      if (!std::isnan(b_x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    result = b_x[0];
  } else {
    result = b_x[idx - 1];
    for (k = idx + 1; k < 4; k++) {
      if (result > b_x[k - 1]) {
        result = b_x[k - 1];
      }
    }
  }

  return result;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDT_utilLSQFixedSizeData_j(const real_T C[21], const
  real_T d[7], real_T x[3])
{
  static const int32_T tmp_1{ 3 };

  __m128d tmp;
  __m128d tmp_0;
  real_T d_0[7];
  real_T w[3];
  real_T wz[3];
  real_T z[3];
  real_T a__4;
  real_T tol;
  real_T w_0;
  int32_T b_ct;
  int16_T iter;
  boolean_T P[3];
  boolean_T Z[3];
  boolean_T z_0[3];
  boolean_T exitg2;
  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm(C) * 7.0;
  wz[0] = 0.0;
  P[0] = false;
  Z[0] = true;
  x[0] = 0.0;
  wz[1] = 0.0;
  P[1] = false;
  Z[1] = true;
  x[1] = 0.0;
  wz[2] = 0.0;
  P[2] = false;
  Z[2] = true;
  x[2] = 0.0;
  for (b_ct = 0; b_ct <= 4; b_ct += 2) {
    tmp = _mm_loadu_pd(&C[b_ct]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(0.0));
    tmp_0 = _mm_loadu_pd(&C[b_ct + 7]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 14]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&d[b_ct]);
    tmp = _mm_sub_pd(tmp_0, tmp);
    _mm_storeu_pd(&d_0[b_ct], tmp);
  }

  for (b_ct = 6; b_ct < 7; b_ct++) {
    a__4 = C[b_ct] * 0.0;
    a__4 += C[b_ct + 7] * 0.0;
    a__4 += C[b_ct + 14] * 0.0;
    d_0[b_ct] = d[b_ct] - a__4;
  }

  for (b_ct = 0; b_ct < 3; b_ct++) {
    w[b_ct] = 0.0;
    for (int32_T i{0}; i < 7; i++) {
      w_0 = w[b_ct];
      w_0 += C[7 * b_ct + i] * d_0[i];
      w[b_ct] = w_0;
    }
  }

  iter = 0;
  exitg2 = false;
  while ((!exitg2) && AutoPIDTuner_vectorAny(Z, &tmp_1)) {
    boolean_T exitg3;
    boolean_T found;
    found = false;
    b_ct = 1;
    exitg3 = false;
    while ((!exitg3) && (b_ct - 1 < 3)) {
      if (Z[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>(b_ct) - 1]
           > tol)) {
        found = true;
        exitg3 = true;
      } else {
        b_ct++;
      }
    }

    if (found) {
      int32_T exitg1;
      if (P[0]) {
        wz[0] = -1.7976931348623157E+308;
      }

      if (Z[0]) {
        wz[0] = w[0];
      }

      z[0] = 0.0;
      if (P[1]) {
        wz[1] = -1.7976931348623157E+308;
      }

      if (Z[1]) {
        wz[1] = w[1];
      }

      z[1] = 0.0;
      if (P[2]) {
        wz[2] = -1.7976931348623157E+308;
      }

      if (Z[2]) {
        wz[2] = w[2];
      }

      z[2] = 0.0;
      AutoPIDTuner_maximum(wz, &a__4, &b_ct);
      P[b_ct - 1] = true;
      Z[b_ct - 1] = false;
      AutoPIDTuner_computeZ_j(z, C, d, P);
      do {
        exitg1 = 0;
        found = false;
        b_ct = 1;
        exitg3 = false;
        while ((!exitg3) && (b_ct - 1 < 3)) {
          if (P[static_cast<int16_T>(b_ct) - 1] && (z[static_cast<int16_T>(b_ct)
               - 1] <= 0.0)) {
            found = true;
            exitg3 = true;
          } else {
            b_ct++;
          }
        }

        if (found) {
          iter = static_cast<int16_T>(iter + 1);
          if (iter > 9) {
            x[0] = z[0];
            x[1] = z[1];
            x[2] = z[2];
            exitg1 = 1;
          } else {
            real_T x_0;
            boolean_T P_0;
            z_0[0] = ((z[0] <= 0.0) && P[0]);
            z_0[1] = ((z[1] <= 0.0) && P[1]);
            z_0[2] = ((z[2] <= 0.0) && P[2]);
            a__4 = AutoPIDTuner_computeAlpha(x, z, z_0);
            found = Z[0];
            P_0 = P[0];
            w_0 = z[0];
            x_0 = x[0];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[0] = x_0;
            z[0] = 0.0;
            P[0] = P_0;
            Z[0] = found;
            found = Z[1];
            P_0 = P[1];
            w_0 = z[1];
            x_0 = x[1];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[1] = x_0;
            z[1] = 0.0;
            P[1] = P_0;
            Z[1] = found;
            found = Z[2];
            P_0 = P[2];
            w_0 = z[2];
            x_0 = x[2];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[2] = x_0;
            z[2] = 0.0;
            P[2] = P_0;
            Z[2] = found;
            AutoPIDTuner_computeZ_j(z, C, d, P);
          }
        } else {
          x[0] = z[0];
          x[1] = z[1];
          x[2] = z[2];
          for (b_ct = 0; b_ct <= 4; b_ct += 2) {
            tmp = _mm_loadu_pd(&C[b_ct]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(z[0]));
            tmp_0 = _mm_loadu_pd(&C[b_ct + 7]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[1]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&C[b_ct + 14]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[2]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&d[b_ct]);
            tmp = _mm_sub_pd(tmp_0, tmp);
            _mm_storeu_pd(&d_0[b_ct], tmp);
          }

          for (b_ct = 6; b_ct < 7; b_ct++) {
            a__4 = C[b_ct] * z[0];
            a__4 += C[b_ct + 7] * z[1];
            a__4 += C[b_ct + 14] * z[2];
            d_0[b_ct] = d[b_ct] - a__4;
          }

          for (b_ct = 0; b_ct < 3; b_ct++) {
            w[b_ct] = 0.0;
            for (int32_T i{0}; i < 7; i++) {
              w_0 = w[b_ct];
              w_0 += C[7 * b_ct + i] * d_0[i];
              w[b_ct] = w_0;
            }
          }

          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    } else {
      exitg2 = true;
    }
  }
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaN);
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u1 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u0 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = std::atan2(static_cast<real_T>(tmp_0), static_cast<real_T>(tmp));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_computePM(const creal_T L)
{
  real_T PM;
  real_T r;
  real_T x;
  x = rt_atan2d_snf(L.im, L.re) * 180.0 / 3.1415926535897931;
  if (std::isnan(x) || std::isinf(x)) {
    r = (rtNaN);
  } else if (x == 0.0) {
    r = 0.0;
  } else {
    r = std::fmod(x, 360.0);
    if (r == 0.0) {
      r = 0.0;
    } else if (x < 0.0) {
      r += 360.0;
    }
  }

  PM = r - 180.0;
  return PM;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_maximum_f(const real_T x[50])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 51)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 51; k++) {
      if (ex < x[k - 1]) {
        ex = x[k - 1];
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_blkdiag_ch(const real_T varargin_1[12], real_T
  y[32])
{
  std::memset(&y[0], 0, sizeof(real_T) << 5U);
  for (int32_T i{0}; i < 6; i++) {
    y[i] = varargin_1[i];
    y[i + 8] = varargin_1[i + 6];
  }

  y[22] = 1.0;
  y[31] = 1.0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_norm_p(const real_T x[32])
{
  real_T y;
  int32_T j;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 4)) {
    real_T s;
    s = 0.0;
    for (int32_T i{0}; i < 8; i++) {
      s += std::abs(x[(j << 3) + i]);
    }

    if (std::isnan(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_maximum_f5(const real_T x[4], real_T *ex,
  int32_T *idx)
{
  int32_T b_idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    b_idx = 1;
  } else {
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!std::isnan(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[b_idx - 1];
    *idx = b_idx;
    for (k = b_idx + 1; k < 5; k++) {
      if (*ex < x[k - 1]) {
        *ex = x[k - 1];
        *idx = k;
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_xnrm2_bx(int32_T n, const real_T x[64],
  int32_T ix0)
{
  real_T y;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      real_T scale;
      int32_T kend;
      scale = 3.3121686421112381E-170;
      kend = (ix0 + n) - 1;
      for (int32_T k{ix0}; k <= kend; k++) {
        real_T absxk;
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          real_T t;
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          real_T t;
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgemv_f2(int32_T m, int32_T n, const real_T A[64],
  int32_T ia0, const real_T x[64], int32_T ix0, real_T y[8])
{
  if (n != 0) {
    int32_T b;
    int32_T d;
    b = static_cast<uint8_T>(n);
    std::memset(&y[0], 0, static_cast<uint32_T>(b) * sizeof(real_T));
    d = ((n - 1) << 3) + ia0;
    for (int32_T b_iy{ia0}; b_iy <= d; b_iy += 8) {
      real_T c;
      int32_T e;
      int32_T ix;
      ix = ix0 - 1;
      c = 0.0;
      e = b_iy + m;
      for (b = b_iy; b < e; b++) {
        c += x[(ix + b) - b_iy] * A[b - 1];
      }

      y[(b_iy - ia0) >> 3] += c;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgerc_c4(int32_T m, int32_T n, real_T alpha1,
  int32_T ix0, const real_T y[8], real_T A[64], int32_T ia0)
{
  if (!(alpha1 == 0.0)) {
    int32_T b;
    int32_T jA;
    jA = ia0;
    b = static_cast<uint8_T>(n);
    for (int32_T j{0}; j < b; j++) {
      real_T temp;
      temp = y[j];
      if (temp != 0.0) {
        int32_T c;
        int32_T ix;
        temp *= alpha1;
        ix = ix0 - 1;
        c = m + jA;
        for (int32_T ijA{jA}; ijA < c; ijA++) {
          A[ijA - 1] += A[(ix + ijA) - jA] * temp;
        }
      }

      jA += 8;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xgeqp3_k(real_T A[64], real_T tau[8], int32_T
  jpvt[8])
{
  real_T vn1[8];
  real_T vn2[8];
  real_T work[8];
  real_T scale;
  real_T smax;
  int32_T ii;
  int32_T pvt;
  for (int32_T k{0}; k < 8; k++) {
    jpvt[k] = k + 1;
    tau[k] = 0.0;
    work[k] = 0.0;
    pvt = (k << 3) + 1;
    smax = 0.0;
    scale = 3.3121686421112381E-170;
    for (ii = pvt; ii <= pvt + 7; ii++) {
      real_T absxk;
      absxk = std::abs(A[ii - 1]);
      if (absxk > scale) {
        real_T t;
        t = scale / absxk;
        smax = smax * t * t + 1.0;
        scale = absxk;
      } else {
        real_T t;
        t = absxk / scale;
        smax += t * t;
      }
    }

    smax = scale * std::sqrt(smax);
    vn2[k] = smax;
    vn1[k] = smax;
  }

  for (int32_T k{0}; k < 8; k++) {
    int32_T b_ix;
    int32_T itemp;
    int32_T ix;
    ii = (k << 3) + k;
    itemp = 8 - k;
    b_ix = 0;
    if (8 - k > 1) {
      ix = k;
      smax = std::abs(vn1[k]);
      for (pvt = 2; pvt <= itemp; pvt++) {
        ix++;
        scale = std::abs(vn1[ix]);
        if (scale > smax) {
          b_ix = pvt - 1;
          smax = scale;
        }
      }
    }

    pvt = k + b_ix;
    if (pvt != k) {
      b_ix = pvt << 3;
      ix = k << 3;
      for (itemp = 0; itemp < 8; itemp++) {
        smax = A[b_ix];
        A[b_ix] = A[ix];
        A[ix] = smax;
        b_ix++;
        ix++;
      }

      itemp = jpvt[pvt];
      jpvt[pvt] = jpvt[k];
      jpvt[k] = itemp;
      vn1[pvt] = vn1[k];
      vn2[pvt] = vn2[k];
    }

    if (k + 1 < 8) {
      scale = A[ii];
      pvt = ii + 2;
      tau[k] = 0.0;
      smax = AutoPIDTuner_xnrm2_bx(7 - k, A, ii + 2);
      if (smax != 0.0) {
        smax = rt_hypotd_snf(A[ii], smax);
        if (A[ii] >= 0.0) {
          smax = -smax;
        }

        if (std::abs(smax) < 1.0020841800044864E-292) {
          __m128d tmp;
          int32_T scalarLB;
          int32_T vectorUB;
          itemp = 0;
          do {
            itemp++;
            ix = (ii - k) + 8;
            scalarLB = ((((ix - pvt) + 1) / 2) << 1) + pvt;
            vectorUB = scalarLB - 2;
            for (b_ix = pvt; b_ix <= vectorUB; b_ix += 2) {
              tmp = _mm_loadu_pd(&A[b_ix - 1]);
              tmp = _mm_mul_pd(tmp, _mm_set1_pd(9.9792015476736E+291));
              _mm_storeu_pd(&A[b_ix - 1], tmp);
            }

            for (b_ix = scalarLB; b_ix <= ix; b_ix++) {
              A[b_ix - 1] *= 9.9792015476736E+291;
            }

            smax *= 9.9792015476736E+291;
            scale *= 9.9792015476736E+291;
          } while ((std::abs(smax) < 1.0020841800044864E-292) && (itemp < 20));

          smax = rt_hypotd_snf(scale, AutoPIDTuner_xnrm2_bx(7 - k, A, ii + 2));
          if (scale >= 0.0) {
            smax = -smax;
          }

          tau[k] = (smax - scale) / smax;
          scale = 1.0 / (scale - smax);
          ix = (ii - k) + 8;
          scalarLB = ((((ix - pvt) + 1) / 2) << 1) + pvt;
          vectorUB = scalarLB - 2;
          for (b_ix = pvt; b_ix <= vectorUB; b_ix += 2) {
            tmp = _mm_loadu_pd(&A[b_ix - 1]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(scale));
            _mm_storeu_pd(&A[b_ix - 1], tmp);
          }

          for (b_ix = scalarLB; b_ix <= ix; b_ix++) {
            A[b_ix - 1] *= scale;
          }

          for (pvt = 0; pvt < itemp; pvt++) {
            smax *= 1.0020841800044864E-292;
          }

          scale = smax;
        } else {
          int32_T scalarLB;
          int32_T vectorUB;
          tau[k] = (smax - A[ii]) / smax;
          scale = 1.0 / (A[ii] - smax);
          b_ix = (ii - k) + 8;
          scalarLB = ((((b_ix - pvt) + 1) / 2) << 1) + pvt;
          vectorUB = scalarLB - 2;
          for (itemp = pvt; itemp <= vectorUB; itemp += 2) {
            __m128d tmp;
            tmp = _mm_loadu_pd(&A[itemp - 1]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(scale));
            _mm_storeu_pd(&A[itemp - 1], tmp);
          }

          for (itemp = scalarLB; itemp <= b_ix; itemp++) {
            A[itemp - 1] *= scale;
          }

          scale = smax;
        }
      }

      A[ii] = scale;
      smax = A[ii];
      A[ii] = 1.0;
      if (tau[k] != 0.0) {
        boolean_T exitg2;
        pvt = 8 - k;
        itemp = (ii - k) + 7;
        while ((pvt > 0) && (A[itemp] == 0.0)) {
          pvt--;
          itemp--;
        }

        itemp = 7 - k;
        exitg2 = false;
        while ((!exitg2) && (itemp > 0)) {
          int32_T exitg1;
          b_ix = (((itemp - 1) << 3) + ii) + 8;
          ix = b_ix;
          do {
            exitg1 = 0;
            if (ix + 1 <= b_ix + pvt) {
              if (A[ix] != 0.0) {
                exitg1 = 1;
              } else {
                ix++;
              }
            } else {
              itemp--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        pvt = 0;
        itemp = 0;
      }

      if (pvt > 0) {
        AutoPIDTuner_xgemv_f2(pvt, itemp, A, ii + 9, A, ii + 1, work);
        AutoPIDTuner_xgerc_c4(pvt, itemp, -tau[k], ii + 1, work, A, ii + 9);
      }

      A[ii] = smax;
    } else {
      tau[7] = 0.0;
    }

    for (ii = k + 2; ii < 9; ii++) {
      pvt = ((ii - 1) << 3) + k;
      if (vn1[ii - 1] != 0.0) {
        smax = std::abs(A[pvt]) / vn1[ii - 1];
        smax = 1.0 - smax * smax;
        if (smax < 0.0) {
          smax = 0.0;
        }

        scale = vn1[ii - 1] / vn2[ii - 1];
        scale = scale * scale * smax;
        if (scale <= 1.4901161193847656E-8) {
          if (k + 1 < 8) {
            vn1[ii - 1] = AutoPIDTuner_xnrm2_bx(7 - k, A, pvt + 2);
            vn2[ii - 1] = vn1[ii - 1];
          } else {
            vn1[ii - 1] = 0.0;
            vn2[ii - 1] = 0.0;
          }
        } else {
          vn1[ii - 1] *= std::sqrt(smax);
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_xorgqr_o(int32_T k, real_T A[64], const real_T
  tau[8])
{
  real_T work[8];
  int32_T ia;
  int32_T itau;
  for (itau = k; itau < 8; itau++) {
    ia = itau << 3;
    std::memset(&A[ia], 0, sizeof(real_T) << 3U);
    A[ia + itau] = 1.0;
  }

  itau = k - 1;
  std::memset(&work[0], 0, sizeof(real_T) << 3U);
  for (int32_T i{k}; i >= 1; i--) {
    int32_T b_ia;
    int32_T coltop;
    int32_T lastc;
    int32_T lastv;
    ia = (((i - 1) << 3) + i) + 8;
    A[ia - 9] = 1.0;
    if (tau[itau] != 0.0) {
      boolean_T exitg2;
      lastv = 9 - i;
      lastc = ia - i;
      while ((lastv > 0) && (A[lastc - 1] == 0.0)) {
        lastv--;
        lastc--;
      }

      lastc = 8 - i;
      exitg2 = false;
      while ((!exitg2) && (lastc > 0)) {
        int32_T exitg1;
        coltop = ((lastc - 1) << 3) + ia;
        b_ia = coltop;
        do {
          exitg1 = 0;
          if (b_ia <= (coltop + lastv) - 1) {
            if (A[b_ia - 1] != 0.0) {
              exitg1 = 1;
            } else {
              b_ia++;
            }
          } else {
            lastc--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      lastv = 0;
      lastc = 0;
    }

    if (lastv > 0) {
      AutoPIDTuner_xgemv_f2(lastv, lastc, A, ia, A, ia - 8, work);
      AutoPIDTuner_xgerc_c4(lastv, lastc, -tau[itau], ia - 8, work, A, ia);
    }

    lastc = ia - i;
    coltop = (((((lastc - ia) + 8) / 2) << 1) + ia) - 7;
    b_ia = coltop - 2;
    for (lastv = ia - 7; lastv <= b_ia; lastv += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&A[lastv - 1]);
      tmp = _mm_mul_pd(tmp, _mm_set1_pd(-tau[itau]));
      _mm_storeu_pd(&A[lastv - 1], tmp);
    }

    for (lastv = coltop; lastv <= lastc; lastv++) {
      A[lastv - 1] *= -tau[itau];
    }

    A[ia - 9] = 1.0 - tau[itau];
    lastc = static_cast<uint8_T>(i - 1);
    for (lastv = 0; lastv < lastc; lastv++) {
      A[(ia - lastv) - 10] = 0.0;
    }

    itau--;
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_computeZ_j3(real_T Z[4], const real_T C[32],
  const real_T d[8], const boolean_T p[4])
{
  real_T Q[64];
  real_T A[32];
  real_T qtd[8];
  real_T z[4];
  int32_T jpvt1[8];
  int32_T jpvt[4];
  int32_T ct;
  int32_T ncols;
  std::memset(&A[0], 0, sizeof(real_T) << 5U);
  ncols = -1;
  for (ct = 0; ct < 4; ct++) {
    if (p[ct]) {
      ncols++;
      std::memcpy(&A[ncols << 3], &C[ct << 3], sizeof(real_T) << 3U);
    }
  }

  for (int32_T c_i{0}; c_i < 8; c_i++) {
    Q[c_i] = A[c_i];
    Q[c_i + 32] = 0.0;
    Q[c_i + 8] = A[c_i + 8];
    Q[c_i + 40] = 0.0;
    Q[c_i + 16] = A[c_i + 16];
    Q[c_i + 48] = 0.0;
    Q[c_i + 24] = A[c_i + 24];
    Q[c_i + 56] = 0.0;
  }

  AutoPIDTuner_xgeqp3_k(Q, qtd, jpvt1);
  for (ct = 0; ct < 4; ct++) {
    jpvt[ct] = jpvt1[ct];
    for (int32_T c_i{0}; c_i <= ct; c_i++) {
      A[c_i + (ct << 3)] = Q[(ct << 3) + c_i];
    }

    for (int32_T c_i{ct + 2}; c_i < 9; c_i++) {
      A[(c_i + (ct << 3)) - 1] = 0.0;
    }
  }

  AutoPIDTuner_xorgqr_o(4, Q, qtd);
  for (int32_T c_i{0}; c_i < 8; c_i++) {
    qtd[c_i] = 0.0;
    for (ct = 0; ct < 8; ct++) {
      real_T qtd_0;
      qtd_0 = qtd[c_i];
      qtd_0 += Q[(c_i << 3) + ct] * d[ct];
      qtd[c_i] = qtd_0;
    }
  }

  z[0] = 0.0;
  z[1] = 0.0;
  z[2] = 0.0;
  z[3] = 0.0;
  for (ct = 0; ct <= ncols; ct++) {
    z[jpvt[ct] - 1] = qtd[ct];
  }

  for (ct = 0; ct <= ncols; ct++) {
    int32_T j;
    j = ncols - ct;
    z[jpvt[j] - 1] /= A[(j << 3) + j];
    for (int32_T c_i{0}; c_i < j; c_i++) {
      z[jpvt[c_i] - 1] -= A[(j << 3) + c_i] * z[jpvt[j] - 1];
    }
  }

  ct = 0;
  if (p[0]) {
    Z[0] = z[0];
    ct = 1;
  }

  if (p[1]) {
    Z[1] = z[ct];
    ct++;
  }

  if (p[2]) {
    Z[2] = z[ct];
    ct++;
  }

  if (p[3]) {
    Z[3] = z[ct];
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_computeAlpha_f(const real_T x[4], const real_T
  z[4], const boolean_T Q[4])
{
  real_T b_x[4];
  real_T result;
  int32_T idx;
  int32_T k;
  b_x[0] = 1.7976931348623157E+308;
  b_x[1] = 1.7976931348623157E+308;
  b_x[2] = 1.7976931348623157E+308;
  b_x[3] = 1.7976931348623157E+308;
  if (Q[0]) {
    b_x[0] = x[0] / (x[0] - z[0]);
  }

  if (Q[1]) {
    b_x[1] = x[1] / (x[1] - z[1]);
  }

  if (Q[2]) {
    b_x[2] = x[2] / (x[2] - z[2]);
  }

  if (Q[3]) {
    b_x[3] = x[3] / (x[3] - z[3]);
  }

  if (!std::isnan(b_x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 4)) {
      if (!std::isnan(b_x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    result = b_x[0];
  } else {
    result = b_x[idx - 1];
    for (k = idx + 1; k < 5; k++) {
      if (result > b_x[k - 1]) {
        result = b_x[k - 1];
      }
    }
  }

  return result;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPID_utilLSQFixedSizeData_j2(const real_T C[32], const
  real_T d[8], real_T x[4])
{
  static const int32_T tmp_1{ 4 };

  __m128d tmp;
  __m128d tmp_0;
  real_T d_0[8];
  real_T w[4];
  real_T wz[4];
  real_T z[4];
  real_T a__4;
  real_T tol;
  real_T w_0;
  int32_T b_ct;
  int16_T iter;
  boolean_T P[4];
  boolean_T Z[4];
  boolean_T z_0[4];
  boolean_T exitg2;
  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm_p(C) * 8.0;
  wz[0] = 0.0;
  P[0] = false;
  Z[0] = true;
  x[0] = 0.0;
  wz[1] = 0.0;
  P[1] = false;
  Z[1] = true;
  x[1] = 0.0;
  wz[2] = 0.0;
  P[2] = false;
  Z[2] = true;
  x[2] = 0.0;
  wz[3] = 0.0;
  P[3] = false;
  Z[3] = true;
  x[3] = 0.0;
  for (b_ct = 0; b_ct <= 6; b_ct += 2) {
    tmp = _mm_loadu_pd(&C[b_ct]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(0.0));
    tmp_0 = _mm_loadu_pd(&C[b_ct + 8]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 16]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 24]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&d[b_ct]);
    tmp = _mm_sub_pd(tmp_0, tmp);
    _mm_storeu_pd(&d_0[b_ct], tmp);
  }

  for (b_ct = 0; b_ct < 4; b_ct++) {
    w[b_ct] = 0.0;
    for (int32_T i{0}; i < 8; i++) {
      w_0 = w[b_ct];
      w_0 += C[(b_ct << 3) + i] * d_0[i];
      w[b_ct] = w_0;
    }
  }

  iter = 0;
  exitg2 = false;
  while ((!exitg2) && AutoPIDTuner_vectorAny(Z, &tmp_1)) {
    boolean_T exitg3;
    boolean_T found;
    found = false;
    b_ct = 1;
    exitg3 = false;
    while ((!exitg3) && (b_ct - 1 < 4)) {
      if (Z[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>(b_ct) - 1]
           > tol)) {
        found = true;
        exitg3 = true;
      } else {
        b_ct++;
      }
    }

    if (found) {
      int32_T exitg1;
      if (P[0]) {
        wz[0] = -1.7976931348623157E+308;
      }

      if (Z[0]) {
        wz[0] = w[0];
      }

      z[0] = 0.0;
      if (P[1]) {
        wz[1] = -1.7976931348623157E+308;
      }

      if (Z[1]) {
        wz[1] = w[1];
      }

      z[1] = 0.0;
      if (P[2]) {
        wz[2] = -1.7976931348623157E+308;
      }

      if (Z[2]) {
        wz[2] = w[2];
      }

      z[2] = 0.0;
      if (P[3]) {
        wz[3] = -1.7976931348623157E+308;
      }

      if (Z[3]) {
        wz[3] = w[3];
      }

      z[3] = 0.0;
      AutoPIDTuner_maximum_f5(wz, &a__4, &b_ct);
      P[b_ct - 1] = true;
      Z[b_ct - 1] = false;
      AutoPIDTuner_computeZ_j3(z, C, d, P);
      do {
        exitg1 = 0;
        found = false;
        b_ct = 1;
        exitg3 = false;
        while ((!exitg3) && (b_ct - 1 < 4)) {
          if (P[static_cast<int16_T>(b_ct) - 1] && (z[static_cast<int16_T>(b_ct)
               - 1] <= 0.0)) {
            found = true;
            exitg3 = true;
          } else {
            b_ct++;
          }
        }

        if (found) {
          iter = static_cast<int16_T>(iter + 1);
          if (iter > 12) {
            x[0] = z[0];
            x[1] = z[1];
            x[2] = z[2];
            x[3] = z[3];
            exitg1 = 1;
          } else {
            real_T x_0;
            boolean_T P_0;
            z_0[0] = ((z[0] <= 0.0) && P[0]);
            z_0[1] = ((z[1] <= 0.0) && P[1]);
            z_0[2] = ((z[2] <= 0.0) && P[2]);
            z_0[3] = ((z[3] <= 0.0) && P[3]);
            a__4 = AutoPIDTuner_computeAlpha_f(x, z, z_0);
            found = Z[0];
            P_0 = P[0];
            w_0 = z[0];
            x_0 = x[0];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[0] = x_0;
            z[0] = 0.0;
            P[0] = P_0;
            Z[0] = found;
            found = Z[1];
            P_0 = P[1];
            w_0 = z[1];
            x_0 = x[1];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[1] = x_0;
            z[1] = 0.0;
            P[1] = P_0;
            Z[1] = found;
            found = Z[2];
            P_0 = P[2];
            w_0 = z[2];
            x_0 = x[2];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[2] = x_0;
            z[2] = 0.0;
            P[2] = P_0;
            Z[2] = found;
            found = Z[3];
            P_0 = P[3];
            w_0 = z[3];
            x_0 = x[3];
            x_0 += (w_0 - x_0) * a__4;
            w_0 = std::abs(x_0);
            found = (((w_0 < tol) && P_0) || found);
            P_0 = !found;
            x[3] = x_0;
            z[3] = 0.0;
            P[3] = P_0;
            Z[3] = found;
            AutoPIDTuner_computeZ_j3(z, C, d, P);
          }
        } else {
          x[0] = z[0];
          x[1] = z[1];
          x[2] = z[2];
          x[3] = z[3];
          for (b_ct = 0; b_ct <= 6; b_ct += 2) {
            tmp = _mm_loadu_pd(&C[b_ct]);
            tmp = _mm_mul_pd(tmp, _mm_set1_pd(z[0]));
            tmp_0 = _mm_loadu_pd(&C[b_ct + 8]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[1]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&C[b_ct + 16]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[2]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&C[b_ct + 24]);
            tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(z[3]));
            tmp = _mm_add_pd(tmp_0, tmp);
            tmp_0 = _mm_loadu_pd(&d[b_ct]);
            tmp = _mm_sub_pd(tmp_0, tmp);
            _mm_storeu_pd(&d_0[b_ct], tmp);
          }

          for (b_ct = 0; b_ct < 4; b_ct++) {
            w[b_ct] = 0.0;
            for (int32_T i{0}; i < 8; i++) {
              w_0 = w[b_ct];
              w_0 += C[(b_ct << 3) + i] * d_0[i];
              w[b_ct] = w_0;
            }
          }

          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    } else {
      exitg2 = true;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_logspace_l(real_T d1, real_T d2, real_T y[20])
{
  real_T b_y[20];
  b_y[19] = d2;
  b_y[0] = d1;
  if (d1 == -d2) {
    real_T delta1;
    delta1 = d2 / 19.0;
    for (int32_T c_k{0}; c_k < 18; c_k++) {
      b_y[c_k + 1] = ((static_cast<real_T>(c_k) + 2.0) * 2.0 - 21.0) * delta1;
    }
  } else if (((d1 < 0.0) != (d2 < 0.0)) && ((std::abs(d1) >
               8.9884656743115785E+307) || (std::abs(d2) >
               8.9884656743115785E+307))) {
    real_T delta1;
    real_T delta2;
    delta1 = d1 / 19.0;
    delta2 = d2 / 19.0;
    for (int32_T c_k{0}; c_k < 18; c_k++) {
      b_y[c_k + 1] = ((static_cast<real_T>(c_k) + 1.0) * delta2 + d1) - (
        static_cast<real_T>(c_k) + 1.0) * delta1;
    }
  } else {
    real_T delta1;
    delta1 = (d2 - d1) / 19.0;
    for (int32_T c_k{0}; c_k < 18; c_k++) {
      b_y[c_k + 1] = (static_cast<real_T>(c_k) + 1.0) * delta1 + d1;
    }
  }

  for (int32_T c_k{0}; c_k < 20; c_k++) {
    y[c_k] = rt_powd_snf(10.0, b_y[c_k]);
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_computeTAU(boolean_T IsDiscrete, const real_T
  w3[3], real_T Ts, uint16_T DF, real_T tau[20])
{
  if (IsDiscrete && (DF == 1)) {
    AutoPIDTuner_logspace_l(std::log10(std::fmin(w3[2], 1.99 / Ts)), std::log10
      (std::fmin(w3[2] * 10.0, 1.99 / Ts)), tau);
    for (int32_T i{0}; i <= 18; i += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&tau[i]);
      tmp = _mm_div_pd(_mm_set1_pd(1.0), tmp);
      _mm_storeu_pd(&tau[i], tmp);
    }
  } else {
    AutoPIDTuner_logspace_l(std::log10(w3[2]), std::log10(w3[2] * 10.0), tau);
    for (int32_T i{0}; i <= 18; i += 2) {
      __m128d tmp;
      tmp = _mm_loadu_pd(&tau[i]);
      tmp = _mm_div_pd(_mm_set1_pd(1.0), tmp);
      _mm_storeu_pd(&tau[i], tmp);
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoP_utilLSQFixedSizeData_j2zd(const real_T C[21], const
  real_T d[7], real_T x[3], real_T *resnorm)
{
  static const int32_T tmp_1{ 3 };

  __m128d tmp;
  __m128d tmp_0;
  real_T resid[7];
  real_T w[3];
  real_T wz[3];
  real_T a__4;
  real_T tol;
  real_T w_1;
  int32_T b_ct;
  int16_T iter;
  boolean_T P[3];
  boolean_T Z[3];
  boolean_T w_0[3];
  boolean_T guard1{ false };

  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm(C) * 7.0;
  wz[0] = 0.0;
  P[0] = false;
  Z[0] = true;
  x[0] = 0.0;
  wz[1] = 0.0;
  P[1] = false;
  Z[1] = true;
  x[1] = 0.0;
  wz[2] = 0.0;
  P[2] = false;
  Z[2] = true;
  x[2] = 0.0;
  for (b_ct = 0; b_ct <= 4; b_ct += 2) {
    tmp = _mm_loadu_pd(&C[b_ct]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(0.0));
    tmp_0 = _mm_loadu_pd(&C[b_ct + 7]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 14]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&d[b_ct]);
    tmp = _mm_sub_pd(tmp_0, tmp);
    _mm_storeu_pd(&resid[b_ct], tmp);
  }

  for (b_ct = 6; b_ct < 7; b_ct++) {
    w_1 = C[b_ct] * 0.0;
    w_1 += C[b_ct + 7] * 0.0;
    w_1 += C[b_ct + 14] * 0.0;
    resid[b_ct] = d[b_ct] - w_1;
  }

  for (b_ct = 0; b_ct < 3; b_ct++) {
    w[b_ct] = 0.0;
    for (int32_T i{0}; i < 7; i++) {
      w_1 = w[b_ct];
      w_1 += C[7 * b_ct + i] * resid[i];
      w[b_ct] = w_1;
    }
  }

  iter = 0;
  guard1 = false;
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (AutoPIDTuner_vectorAny(Z, &tmp_1)) {
      boolean_T exitg3;
      boolean_T found;
      found = false;
      b_ct = 1;
      exitg3 = false;
      while ((!exitg3) && (b_ct - 1 < 3)) {
        if (Z[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>(b_ct) -
             1] > tol)) {
          found = true;
          exitg3 = true;
        } else {
          b_ct++;
        }
      }

      if (found) {
        int32_T exitg1;
        if (P[0]) {
          wz[0] = -1.7976931348623157E+308;
        }

        if (Z[0]) {
          wz[0] = w[0];
        }

        if (P[1]) {
          wz[1] = -1.7976931348623157E+308;
        }

        if (Z[1]) {
          wz[1] = w[1];
        }

        if (P[2]) {
          wz[2] = -1.7976931348623157E+308;
        }

        if (Z[2]) {
          wz[2] = w[2];
        }

        AutoPIDTuner_maximum(wz, &a__4, &b_ct);
        P[b_ct - 1] = true;
        Z[b_ct - 1] = false;
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 0.0;
        AutoPIDTuner_computeZ_j(w, C, d, P);
        do {
          exitg1 = 0;
          found = false;
          b_ct = 1;
          exitg3 = false;
          while ((!exitg3) && (b_ct - 1 < 3)) {
            if (P[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>
                 (b_ct) - 1] <= 0.0)) {
              found = true;
              exitg3 = true;
            } else {
              b_ct++;
            }
          }

          if (found) {
            iter = static_cast<int16_T>(iter + 1);
            if (iter > 9) {
              for (b_ct = 0; b_ct <= 4; b_ct += 2) {
                tmp = _mm_loadu_pd(&resid[b_ct]);
                tmp = _mm_mul_pd(tmp, tmp);
                _mm_storeu_pd(&resid[b_ct], tmp);
              }

              for (b_ct = 6; b_ct < 7; b_ct++) {
                tol = resid[b_ct];
                tol *= tol;
                resid[b_ct] = tol;
              }

              *resnorm = resid[0];
              for (b_ct = 0; b_ct < 6; b_ct++) {
                *resnorm += resid[b_ct + 1];
              }

              x[0] = w[0];
              x[1] = w[1];
              x[2] = w[2];
              exitg1 = 1;
            } else {
              real_T x_0;
              boolean_T P_0;
              w_0[0] = ((w[0] <= 0.0) && P[0]);
              w_0[1] = ((w[1] <= 0.0) && P[1]);
              w_0[2] = ((w[2] <= 0.0) && P[2]);
              a__4 = AutoPIDTuner_computeAlpha(x, w, w_0);
              found = Z[0];
              P_0 = P[0];
              w_1 = w[0];
              x_0 = x[0];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[0] = x_0;
              w[0] = 0.0;
              P[0] = P_0;
              Z[0] = found;
              found = Z[1];
              P_0 = P[1];
              w_1 = w[1];
              x_0 = x[1];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[1] = x_0;
              w[1] = 0.0;
              P[1] = P_0;
              Z[1] = found;
              found = Z[2];
              P_0 = P[2];
              w_1 = w[2];
              x_0 = x[2];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[2] = x_0;
              w[2] = 0.0;
              P[2] = P_0;
              Z[2] = found;
              AutoPIDTuner_computeZ_j(w, C, d, P);
            }
          } else {
            x[0] = w[0];
            x[1] = w[1];
            x[2] = w[2];
            for (b_ct = 0; b_ct <= 4; b_ct += 2) {
              tmp = _mm_loadu_pd(&C[b_ct]);
              tmp = _mm_mul_pd(tmp, _mm_set1_pd(w[0]));
              tmp_0 = _mm_loadu_pd(&C[b_ct + 7]);
              tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(w[1]));
              tmp = _mm_add_pd(tmp_0, tmp);
              tmp_0 = _mm_loadu_pd(&C[b_ct + 14]);
              tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(w[2]));
              tmp = _mm_add_pd(tmp_0, tmp);
              tmp_0 = _mm_loadu_pd(&d[b_ct]);
              tmp = _mm_sub_pd(tmp_0, tmp);
              _mm_storeu_pd(&resid[b_ct], tmp);
            }

            for (b_ct = 6; b_ct < 7; b_ct++) {
              w_1 = C[b_ct] * w[0];
              w_1 += C[b_ct + 7] * w[1];
              w_1 += C[b_ct + 14] * w[2];
              resid[b_ct] = d[b_ct] - w_1;
            }

            for (b_ct = 0; b_ct < 3; b_ct++) {
              w[b_ct] = 0.0;
              for (int32_T i{0}; i < 7; i++) {
                w_1 = w[b_ct];
                w_1 += C[7 * b_ct + i] * resid[i];
                w[b_ct] = w_1;
              }
            }

            guard1 = false;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = 1;
        }
      } else {
        guard1 = true;
        exitg2 = 1;
      }
    } else {
      guard1 = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (guard1) {
    *resnorm = 0.0;
    for (b_ct = 0; b_ct < 7; b_ct++) {
      tol = resid[b_ct];
      *resnorm += tol * tol;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_maximum_f5j(const real_T x[20])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 20)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 21; k++) {
      if (ex < x[k - 1]) {
        ex = x[k - 1];
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_minimum_g(const real_T x[20])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 20)) {
      if (!std::isnan(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 21; k++) {
      if (ex > x[k - 1]) {
        ex = x[k - 1];
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_mean(const real_T x[20])
{
  real_T b_y;
  b_y = x[0];
  for (int32_T k{0}; k < 19; k++) {
    b_y += x[k + 1];
  }

  return b_y / 20.0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_minimum_g1(const real_T x[20], real_T *ex,
  int32_T *idx)
{
  int32_T b_idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    b_idx = 1;
  } else {
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 21)) {
      if (!std::isnan(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[b_idx - 1];
    *idx = b_idx;
    for (k = b_idx + 1; k < 21; k++) {
      if (*ex > x[k - 1]) {
        *ex = x[k - 1];
        *idx = k;
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPI_utilLSQFixedSizeData_j2z(const real_T C[32], const
  real_T d[8], real_T x[4], real_T *resnorm)
{
  static const int32_T tmp_1{ 4 };

  __m128d tmp;
  __m128d tmp_0;
  real_T resid[8];
  real_T w[4];
  real_T wz[4];
  real_T a__4;
  real_T tol;
  real_T w_1;
  int32_T b_ct;
  int16_T iter;
  boolean_T P[4];
  boolean_T Z[4];
  boolean_T w_0[4];
  boolean_T guard1{ false };

  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm_p(C) * 8.0;
  wz[0] = 0.0;
  P[0] = false;
  Z[0] = true;
  x[0] = 0.0;
  wz[1] = 0.0;
  P[1] = false;
  Z[1] = true;
  x[1] = 0.0;
  wz[2] = 0.0;
  P[2] = false;
  Z[2] = true;
  x[2] = 0.0;
  wz[3] = 0.0;
  P[3] = false;
  Z[3] = true;
  x[3] = 0.0;
  for (b_ct = 0; b_ct <= 6; b_ct += 2) {
    tmp = _mm_loadu_pd(&C[b_ct]);
    tmp = _mm_mul_pd(tmp, _mm_set1_pd(0.0));
    tmp_0 = _mm_loadu_pd(&C[b_ct + 8]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 16]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&C[b_ct + 24]);
    tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(0.0));
    tmp = _mm_add_pd(tmp_0, tmp);
    tmp_0 = _mm_loadu_pd(&d[b_ct]);
    tmp = _mm_sub_pd(tmp_0, tmp);
    _mm_storeu_pd(&resid[b_ct], tmp);
  }

  for (b_ct = 0; b_ct < 4; b_ct++) {
    w[b_ct] = 0.0;
    for (int32_T i{0}; i < 8; i++) {
      w_1 = w[b_ct];
      w_1 += C[(b_ct << 3) + i] * resid[i];
      w[b_ct] = w_1;
    }
  }

  iter = 0;
  guard1 = false;
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (AutoPIDTuner_vectorAny(Z, &tmp_1)) {
      boolean_T exitg3;
      boolean_T found;
      found = false;
      b_ct = 1;
      exitg3 = false;
      while ((!exitg3) && (b_ct - 1 < 4)) {
        if (Z[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>(b_ct) -
             1] > tol)) {
          found = true;
          exitg3 = true;
        } else {
          b_ct++;
        }
      }

      if (found) {
        int32_T exitg1;
        if (P[0]) {
          wz[0] = -1.7976931348623157E+308;
        }

        if (Z[0]) {
          wz[0] = w[0];
        }

        if (P[1]) {
          wz[1] = -1.7976931348623157E+308;
        }

        if (Z[1]) {
          wz[1] = w[1];
        }

        if (P[2]) {
          wz[2] = -1.7976931348623157E+308;
        }

        if (Z[2]) {
          wz[2] = w[2];
        }

        if (P[3]) {
          wz[3] = -1.7976931348623157E+308;
        }

        if (Z[3]) {
          wz[3] = w[3];
        }

        AutoPIDTuner_maximum_f5(wz, &a__4, &b_ct);
        P[b_ct - 1] = true;
        Z[b_ct - 1] = false;
        w[0] = 0.0;
        w[1] = 0.0;
        w[2] = 0.0;
        w[3] = 0.0;
        AutoPIDTuner_computeZ_j3(w, C, d, P);
        do {
          exitg1 = 0;
          found = false;
          b_ct = 1;
          exitg3 = false;
          while ((!exitg3) && (b_ct - 1 < 4)) {
            if (P[static_cast<int16_T>(b_ct) - 1] && (w[static_cast<int16_T>
                 (b_ct) - 1] <= 0.0)) {
              found = true;
              exitg3 = true;
            } else {
              b_ct++;
            }
          }

          if (found) {
            iter = static_cast<int16_T>(iter + 1);
            if (iter > 12) {
              for (b_ct = 0; b_ct <= 6; b_ct += 2) {
                tmp = _mm_loadu_pd(&resid[b_ct]);
                tmp = _mm_mul_pd(tmp, tmp);
                _mm_storeu_pd(&resid[b_ct], tmp);
              }

              *resnorm = resid[0];
              for (b_ct = 0; b_ct < 7; b_ct++) {
                *resnorm += resid[b_ct + 1];
              }

              x[0] = w[0];
              x[1] = w[1];
              x[2] = w[2];
              x[3] = w[3];
              exitg1 = 1;
            } else {
              real_T x_0;
              boolean_T P_0;
              w_0[0] = ((w[0] <= 0.0) && P[0]);
              w_0[1] = ((w[1] <= 0.0) && P[1]);
              w_0[2] = ((w[2] <= 0.0) && P[2]);
              w_0[3] = ((w[3] <= 0.0) && P[3]);
              a__4 = AutoPIDTuner_computeAlpha_f(x, w, w_0);
              found = Z[0];
              P_0 = P[0];
              w_1 = w[0];
              x_0 = x[0];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[0] = x_0;
              w[0] = 0.0;
              P[0] = P_0;
              Z[0] = found;
              found = Z[1];
              P_0 = P[1];
              w_1 = w[1];
              x_0 = x[1];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[1] = x_0;
              w[1] = 0.0;
              P[1] = P_0;
              Z[1] = found;
              found = Z[2];
              P_0 = P[2];
              w_1 = w[2];
              x_0 = x[2];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[2] = x_0;
              w[2] = 0.0;
              P[2] = P_0;
              Z[2] = found;
              found = Z[3];
              P_0 = P[3];
              w_1 = w[3];
              x_0 = x[3];
              x_0 += (w_1 - x_0) * a__4;
              w_1 = std::abs(x_0);
              found = (((w_1 < tol) && P_0) || found);
              P_0 = !found;
              x[3] = x_0;
              w[3] = 0.0;
              P[3] = P_0;
              Z[3] = found;
              AutoPIDTuner_computeZ_j3(w, C, d, P);
            }
          } else {
            x[0] = w[0];
            x[1] = w[1];
            x[2] = w[2];
            x[3] = w[3];
            for (b_ct = 0; b_ct <= 6; b_ct += 2) {
              tmp = _mm_loadu_pd(&C[b_ct]);
              tmp = _mm_mul_pd(tmp, _mm_set1_pd(w[0]));
              tmp_0 = _mm_loadu_pd(&C[b_ct + 8]);
              tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(w[1]));
              tmp = _mm_add_pd(tmp_0, tmp);
              tmp_0 = _mm_loadu_pd(&C[b_ct + 16]);
              tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(w[2]));
              tmp = _mm_add_pd(tmp_0, tmp);
              tmp_0 = _mm_loadu_pd(&C[b_ct + 24]);
              tmp_0 = _mm_mul_pd(tmp_0, _mm_set1_pd(w[3]));
              tmp = _mm_add_pd(tmp_0, tmp);
              tmp_0 = _mm_loadu_pd(&d[b_ct]);
              tmp = _mm_sub_pd(tmp_0, tmp);
              _mm_storeu_pd(&resid[b_ct], tmp);
            }

            for (b_ct = 0; b_ct < 4; b_ct++) {
              w[b_ct] = 0.0;
              for (int32_T i{0}; i < 8; i++) {
                w_1 = w[b_ct];
                w_1 += C[(b_ct << 3) + i] * resid[i];
                w[b_ct] = w_1;
              }
            }

            guard1 = false;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = 1;
        }
      } else {
        guard1 = true;
        exitg2 = 1;
      }
    } else {
      guard1 = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (guard1) {
    *resnorm = 0.0;
    for (b_ct = 0; b_ct < 8; b_ct++) {
      tol = resid[b_ct];
      *resnorm += tol * tol;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_blkdiag_chw(const real_T varargin_1[18], real_T
  y[40])
{
  std::memset(&y[0], 0, 40U * sizeof(real_T));
  for (int32_T i{0}; i < 6; i++) {
    y[i] = varargin_1[i];
    y[i + 8] = varargin_1[i + 6];
    y[i + 16] = varargin_1[i + 12];
  }

  y[30] = 1.0;
  y[39] = 1.0;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_norm_pt(const real_T x[40])
{
  real_T y;
  int32_T j;
  boolean_T exitg1;
  y = 0.0;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 5)) {
    real_T s;
    s = 0.0;
    for (int32_T i{0}; i < 8; i++) {
      s += std::abs(x[(j << 3) + i]);
    }

    if (std::isnan(s)) {
      y = (rtNaN);
      exitg1 = true;
    } else {
      if (s > y) {
        y = s;
      }

      j++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_maximum_f5jx(const real_T x[5], real_T *ex,
  int32_T *idx)
{
  int32_T b_idx;
  int32_T k;
  if (!std::isnan(x[0])) {
    b_idx = 1;
  } else {
    boolean_T exitg1;
    b_idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 6)) {
      if (!std::isnan(x[k - 1])) {
        b_idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (b_idx == 0) {
    *ex = x[0];
    *idx = 1;
  } else {
    *ex = x[b_idx - 1];
    *idx = b_idx;
    for (k = b_idx + 1; k < 6; k++) {
      if (*ex < x[k - 1]) {
        *ex = x[k - 1];
        *idx = k;
      }
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_computeZ_j3j(real_T Z[5], const real_T C[40],
  const real_T d[8], const boolean_T p[5])
{
  real_T Q[64];
  real_T A[40];
  real_T qtd[8];
  real_T z[5];
  int32_T jpvt1[8];
  int32_T jpvt[5];
  int32_T ct;
  int32_T ncols;
  std::memset(&A[0], 0, 40U * sizeof(real_T));
  ncols = -1;
  for (ct = 0; ct < 5; ct++) {
    if (p[ct]) {
      ncols++;
      std::memcpy(&A[ncols << 3], &C[ct << 3], sizeof(real_T) << 3U);
    }
  }

  std::memcpy(&Q[0], &A[0], 40U * sizeof(real_T));
  std::memset(&Q[40], 0, 24U * sizeof(real_T));
  AutoPIDTuner_xgeqp3_k(Q, qtd, jpvt1);
  for (ct = 0; ct < 5; ct++) {
    jpvt[ct] = jpvt1[ct];
    for (int32_T c_i{0}; c_i <= ct; c_i++) {
      A[c_i + (ct << 3)] = Q[(ct << 3) + c_i];
    }

    for (int32_T c_i{ct + 2}; c_i < 9; c_i++) {
      A[(c_i + (ct << 3)) - 1] = 0.0;
    }
  }

  AutoPIDTuner_xorgqr_o(5, Q, qtd);
  for (int32_T c_i{0}; c_i < 8; c_i++) {
    qtd[c_i] = 0.0;
    for (ct = 0; ct < 8; ct++) {
      real_T qtd_0;
      qtd_0 = qtd[c_i];
      qtd_0 += Q[(c_i << 3) + ct] * d[ct];
      qtd[c_i] = qtd_0;
    }
  }

  for (ct = 0; ct < 5; ct++) {
    z[ct] = 0.0;
  }

  for (ct = 0; ct <= ncols; ct++) {
    z[jpvt[ct] - 1] = qtd[ct];
  }

  for (ct = 0; ct <= ncols; ct++) {
    int32_T j;
    j = ncols - ct;
    z[jpvt[j] - 1] /= A[(j << 3) + j];
    for (int32_T c_i{0}; c_i < j; c_i++) {
      z[jpvt[c_i] - 1] -= A[(j << 3) + c_i] * z[jpvt[j] - 1];
    }
  }

  ct = 0;
  for (ncols = 0; ncols < 5; ncols++) {
    if (p[ncols]) {
      Z[ncols] = z[ct];
      ct++;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
real_T AutoPIDTuner::AutoPIDTuner_computeAlpha_fd(const real_T x[5], const
  real_T z[5], const boolean_T Q[5])
{
  real_T b_x[5];
  real_T result;
  int32_T idx;
  int32_T k;
  for (idx = 0; idx < 5; idx++) {
    real_T x_0;
    x_0 = x[idx];
    b_x[idx] = 1.7976931348623157E+308;
    if (Q[idx]) {
      b_x[idx] = x_0 / (x_0 - z[idx]);
    }
  }

  if (!std::isnan(b_x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k <= 5)) {
      if (!std::isnan(b_x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    result = b_x[0];
  } else {
    result = b_x[idx - 1];
    for (k = idx + 1; k < 6; k++) {
      if (result > b_x[k - 1]) {
        result = b_x[k - 1];
      }
    }
  }

  return result;
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::Auto_utilLSQFixedSizeData_j2zds(const real_T C[40], const
  real_T d[8], real_T x[5])
{
  static const int32_T tmp{ 5 };

  real_T d_0[8];
  real_T w[5];
  real_T wz[5];
  real_T z[5];
  real_T a__4;
  real_T tol;
  real_T w_0;
  int32_T i;
  int16_T iter;
  boolean_T P[5];
  boolean_T Z[5];
  boolean_T z_0[5];
  boolean_T exitg2;
  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm_pt(C) * 8.0;
  for (i = 0; i < 5; i++) {
    wz[i] = 0.0;
    P[i] = false;
    Z[i] = true;
    x[i] = 0.0;
  }

  for (i = 0; i < 8; i++) {
    a__4 = 0.0;
    for (int32_T i_0{0}; i_0 < 5; i_0++) {
      a__4 += C[(i_0 << 3) + i] * 0.0;
    }

    d_0[i] = d[i] - a__4;
  }

  for (i = 0; i < 5; i++) {
    w[i] = 0.0;
    for (int32_T i_0{0}; i_0 < 8; i_0++) {
      w_0 = w[i];
      w_0 += C[(i << 3) + i_0] * d_0[i_0];
      w[i] = w_0;
    }
  }

  iter = 0;
  exitg2 = false;
  while ((!exitg2) && AutoPIDTuner_vectorAny(Z, &tmp)) {
    boolean_T exitg3;
    boolean_T found;
    found = false;
    i = 1;
    exitg3 = false;
    while ((!exitg3) && (i - 1 < 5)) {
      if (Z[static_cast<int16_T>(i) - 1] && (w[static_cast<int16_T>(i) - 1] >
           tol)) {
        found = true;
        exitg3 = true;
      } else {
        i++;
      }
    }

    if (found) {
      int32_T exitg1;
      for (i = 0; i < 5; i++) {
        if (P[i]) {
          wz[i] = -1.7976931348623157E+308;
        }

        if (Z[i]) {
          wz[i] = w[i];
        }

        z[i] = 0.0;
      }

      AutoPIDTuner_maximum_f5jx(wz, &a__4, &i);
      P[i - 1] = true;
      Z[i - 1] = false;
      AutoPIDTuner_computeZ_j3j(z, C, d, P);
      do {
        exitg1 = 0;
        found = false;
        i = 1;
        exitg3 = false;
        while ((!exitg3) && (i - 1 < 5)) {
          if (P[static_cast<int16_T>(i) - 1] && (z[static_cast<int16_T>(i) - 1] <=
               0.0)) {
            found = true;
            exitg3 = true;
          } else {
            i++;
          }
        }

        if (found) {
          iter = static_cast<int16_T>(iter + 1);
          if (iter > 15) {
            for (i = 0; i < 5; i++) {
              x[i] = z[i];
            }

            exitg1 = 1;
          } else {
            for (i = 0; i < 5; i++) {
              z_0[i] = ((z[i] <= 0.0) && P[i]);
            }

            a__4 = AutoPIDTuner_computeAlpha_fd(x, z, z_0);
            for (i = 0; i < 5; i++) {
              real_T x_0;
              boolean_T Z_0;
              x_0 = x[i];
              w_0 = z[i];
              found = P[i];
              Z_0 = Z[i];
              x_0 += (w_0 - x_0) * a__4;
              w_0 = std::abs(x_0);
              Z_0 = (((w_0 < tol) && found) || Z_0);
              found = !Z_0;
              Z[i] = Z_0;
              P[i] = found;
              z[i] = 0.0;
              x[i] = x_0;
            }

            AutoPIDTuner_computeZ_j3j(z, C, d, P);
          }
        } else {
          for (i = 0; i < 5; i++) {
            x[i] = z[i];
          }

          for (i = 0; i < 8; i++) {
            a__4 = 0.0;
            for (int32_T i_0{0}; i_0 < 5; i_0++) {
              a__4 += C[(i_0 << 3) + i] * z[i_0];
            }

            d_0[i] = d[i] - a__4;
          }

          for (i = 0; i < 5; i++) {
            w[i] = 0.0;
            for (int32_T i_0{0}; i_0 < 8; i_0++) {
              w_0 = w[i];
              w_0 += C[(i << 3) + i_0] * d_0[i_0];
              w[i] = w_0;
            }
          }

          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    } else {
      exitg2 = true;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::Aut_utilLSQFixedSizeData_j2zdsx(const real_T C[40], const
  real_T d[8], real_T x[5], real_T *resnorm)
{
  static const int32_T tmp_0{ 5 };

  real_T resid[8];
  real_T w[5];
  real_T wz[5];
  real_T a__4;
  real_T tol;
  real_T w_1;
  int32_T i;
  int16_T iter;
  boolean_T P[5];
  boolean_T Z[5];
  boolean_T w_0[5];
  boolean_T guard1{ false };

  tol = 2.2204460492503131E-15 * AutoPIDTuner_norm_pt(C) * 8.0;
  for (i = 0; i < 5; i++) {
    wz[i] = 0.0;
    P[i] = false;
    Z[i] = true;
    x[i] = 0.0;
  }

  for (i = 0; i < 8; i++) {
    w_1 = 0.0;
    for (int32_T i_0{0}; i_0 < 5; i_0++) {
      w_1 += C[(i_0 << 3) + i] * 0.0;
    }

    resid[i] = d[i] - w_1;
  }

  for (i = 0; i < 5; i++) {
    w[i] = 0.0;
    for (int32_T i_0{0}; i_0 < 8; i_0++) {
      w_1 = w[i];
      w_1 += C[(i << 3) + i_0] * resid[i_0];
      w[i] = w_1;
    }
  }

  iter = 0;
  guard1 = false;
  int32_T exitg2;
  do {
    exitg2 = 0;
    if (AutoPIDTuner_vectorAny(Z, &tmp_0)) {
      boolean_T exitg3;
      boolean_T found;
      found = false;
      i = 1;
      exitg3 = false;
      while ((!exitg3) && (i - 1 < 5)) {
        if (Z[static_cast<int16_T>(i) - 1] && (w[static_cast<int16_T>(i) - 1] >
             tol)) {
          found = true;
          exitg3 = true;
        } else {
          i++;
        }
      }

      if (found) {
        int32_T exitg1;
        for (i = 0; i < 5; i++) {
          w_1 = w[i];
          if (P[i]) {
            wz[i] = -1.7976931348623157E+308;
          }

          if (Z[i]) {
            wz[i] = w_1;
          }

          w[i] = 0.0;
        }

        AutoPIDTuner_maximum_f5jx(wz, &a__4, &i);
        P[i - 1] = true;
        Z[i - 1] = false;
        AutoPIDTuner_computeZ_j3j(w, C, d, P);
        do {
          exitg1 = 0;
          found = false;
          i = 1;
          exitg3 = false;
          while ((!exitg3) && (i - 1 < 5)) {
            if (P[static_cast<int16_T>(i) - 1] && (w[static_cast<int16_T>(i) - 1]
                 <= 0.0)) {
              found = true;
              exitg3 = true;
            } else {
              i++;
            }
          }

          if (found) {
            iter = static_cast<int16_T>(iter + 1);
            if (iter > 15) {
              for (i = 0; i <= 6; i += 2) {
                __m128d tmp;
                tmp = _mm_loadu_pd(&resid[i]);
                tmp = _mm_mul_pd(tmp, tmp);
                _mm_storeu_pd(&resid[i], tmp);
              }

              *resnorm = resid[0];
              for (i = 0; i < 7; i++) {
                *resnorm += resid[i + 1];
              }

              for (i = 0; i < 5; i++) {
                x[i] = w[i];
              }

              exitg1 = 1;
            } else {
              for (i = 0; i < 5; i++) {
                w_0[i] = ((w[i] <= 0.0) && P[i]);
              }

              a__4 = AutoPIDTuner_computeAlpha_fd(x, w, w_0);
              for (i = 0; i < 5; i++) {
                real_T x_0;
                boolean_T Z_0;
                x_0 = x[i];
                w_1 = w[i];
                found = P[i];
                Z_0 = Z[i];
                x_0 += (w_1 - x_0) * a__4;
                w_1 = std::abs(x_0);
                Z_0 = (((w_1 < tol) && found) || Z_0);
                found = !Z_0;
                Z[i] = Z_0;
                P[i] = found;
                w[i] = 0.0;
                x[i] = x_0;
              }

              AutoPIDTuner_computeZ_j3j(w, C, d, P);
            }
          } else {
            for (i = 0; i < 5; i++) {
              x[i] = w[i];
            }

            for (i = 0; i < 8; i++) {
              w_1 = 0.0;
              for (int32_T i_0{0}; i_0 < 5; i_0++) {
                w_1 += C[(i_0 << 3) + i] * w[i_0];
              }

              resid[i] = d[i] - w_1;
            }

            for (i = 0; i < 5; i++) {
              w[i] = 0.0;
              for (int32_T i_0{0}; i_0 < 8; i_0++) {
                w_1 = w[i];
                w_1 += C[(i << 3) + i_0] * resid[i_0];
                w[i] = w_1;
              }
            }

            guard1 = false;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = 1;
        }
      } else {
        guard1 = true;
        exitg2 = 1;
      }
    } else {
      guard1 = true;
      exitg2 = 1;
    }
  } while (exitg2 == 0);

  if (guard1) {
    *resnorm = 0.0;
    for (i = 0; i < 8; i++) {
      tol = resid[i];
      *resnorm += tol * tol;
    }
  }
}

/* Function for MATLAB Function: '<S71>/DeployedMode' */
void AutoPIDTuner::AutoPIDTuner_slpidfivepoint(real_T type, uint16_T form, const
  real_T frequencies[5], creal_T responses[5], real_T targetPM, real_T
  HasIntegrator, real_T LoopSign, real_T Ts, uint16_T IF, uint16_T DF, real_T *P,
  real_T *b_I, real_T *D, real_T *N, real_T *achievedPM)
{
  static const int8_T h[40]{ 1, 5, 1, 1, 5, 1, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1, 1,
    5, 1, 1, 5, 1, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1 };

  static const int8_T g[32]{ 1, 5, 1, 1, 5, 1, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1, 1,
    5, 1, 1, 5, 1, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1 };

  static const int8_T e[21]{ 1, 5, 1, 1, 5, 1, 1, 1, 5, 1, 1, 5, 1, 1, 1, 5, 1,
    1, 5, 1, 1 };

  static const int8_T d[14]{ 1, 5, 1, 1, 5, 1, 1, 1, 5, 1, 1, 5, 1, 1 };

  creal_T hL3[3];
  creal_T P_0;
  creal_T P_1;
  creal_T d_x;
  real_T b_y[100];
  real_T c_xtau[100];
  real_T f[100];
  real_T b_xtau[80];
  real_T xtau[60];
  real_T f_0[50];
  real_T f_1[50];
  real_T wHigh[50];
  real_T wLow[50];
  real_T d_A[40];
  real_T c_A[32];
  real_T b_A[21];
  real_T gap[20];
  real_T tau[20];
  real_T responses_2[18];
  real_T A[14];
  real_T responses_0[12];
  real_T b_b[8];
  real_T b[7];
  real_T responses_1[6];
  real_T c_x[5];
  real_T y[5];
  real_T b_x[4];
  real_T rG3[3];
  real_T x[2];
  real_T bim;
  real_T brm;
  real_T gammaHigh;
  real_T hI_idx_0_im;
  real_T hI_idx_0_re;
  real_T hI_idx_1_im;
  real_T hI_idx_1_re;
  real_T hI_idx_2_im;
  real_T hI_idx_2_re;
  real_T hL3_re;
  real_T rLast;
  int32_T imin;
  boolean_T IsDiscrete;
  IsDiscrete = (Ts > 0.0);
  AutoPIDTuner_generateTargetLoop(&frequencies[1], targetPM, hL3);
  gammaHigh = rt_hypotd_snf(hL3[2].re, hL3[2].im);
  AutoPIDTuner_logspace(std::log10(frequencies[0]), std::log10(frequencies[2] /
    5.0), wLow);
  AutoPIDTuner_logspace(std::log10(5.0 * frequencies[2]), std::log10
                        (frequencies[4]), wHigh);
  for (imin = 0; imin < 5; imin++) {
    creal_T responses_3;
    responses_3 = responses[imin];
    hI_idx_0_re = responses_3.re;
    hI_idx_0_im = responses_3.im;
    hI_idx_0_re *= LoopSign;
    hI_idx_0_im *= LoopSign;
    responses_3.re = hI_idx_0_re;
    responses_3.im = hI_idx_0_im;
    hI_idx_0_re = responses_3.re;
    hI_idx_0_im = responses_3.im;
    rLast = frequencies[imin];
    rLast = std::log10(rLast);
    hI_idx_0_re = rt_hypotd_snf(hI_idx_0_re, hI_idx_0_im);
    hI_idx_0_re = std::log10(hI_idx_0_re);
    c_x[imin] = rLast;
    y[imin] = hI_idx_0_re;
    responses[imin] = responses_3;
  }

  rG3[2] = rt_hypotd_snf(responses[3].re, responses[3].im);
  rLast = rt_hypotd_snf(responses[4].re, responses[4].im);
  for (int32_T h_k{0}; h_k < 50; h_k++) {
    c_xtau[h_k] = wLow[h_k];
    c_xtau[h_k + 50] = wHigh[h_k];
  }

  for (imin = 0; imin < 100; imin++) {
    hI_idx_0_re = c_xtau[imin];
    hI_idx_0_re = std::log10(hI_idx_0_re);
    c_xtau[imin] = hI_idx_0_re;
  }

  AutoPIDTuner_pchip(c_x, y, c_xtau, b_y);
  for (imin = 0; imin < 100; imin++) {
    f[imin] = rt_powd_snf(10.0, b_y[imin]);
  }

  if (((type == 6.0) || (type == 7.0)) && (rG3[2] < rLast)) {
    type = 3.0;
  }

  if (((type == 4.0) || (type == 5.0)) && (HasIntegrator != 0.0) && (rG3[2] <
       rLast)) {
    type = 1.0;
  }

  if (IsDiscrete) {
    AutoPIDTuner_localGetRealImag(Ts, &frequencies[1], IF, &rLast, rG3);
    hI_idx_0_re = rLast;
    hI_idx_0_im = rG3[0];
    hI_idx_1_re = rLast;
    hI_idx_1_im = rG3[1];
    hI_idx_2_re = rLast;
    hI_idx_2_im = rG3[2];
  } else {
    hI_idx_0_re = 0.0;
    hI_idx_0_im = -1.0 / frequencies[1];
    hI_idx_1_re = 0.0;
    hI_idx_1_im = -1.0 / frequencies[2];
    hI_idx_2_re = 0.0;
    hI_idx_2_im = -1.0 / frequencies[3];
  }

  AutoPIDTuner_localGetRealImag(Ts, &frequencies[1], DF, &rLast, rG3);
  switch (static_cast<int32_T>(type)) {
   case 1:
    {
      if (HasIntegrator != 0.0) {
        responses_1[0] = responses[1].re;
        responses_1[3] = responses[1].im;
        responses_1[1] = responses[2].re;
        responses_1[4] = responses[2].im;
        responses_1[2] = responses[3].re;
        responses_1[5] = responses[3].im;
        AutoPIDTuner_blkdiag(responses_1, A);
        for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
          __m128d tmp;
          __m128d tmp_0;
          tmp = _mm_loadu_pd(&f[h_k]);
          tmp_0 = _mm_loadu_pd(&wLow[h_k]);
          tmp = _mm_mul_pd(tmp, tmp_0);
          _mm_storeu_pd(&f_0[h_k], tmp);
        }

        A[6] = -AutoPIDTuner_minimum(f_0);
        for (int32_T h_k{0}; h_k < 14; h_k++) {
          rLast = A[h_k];
          rLast *= static_cast<real_T>(d[h_k]);
          A[h_k] = rLast;
        }

        real_T hL3_im;
        hL3_re = hL3[0].re;
        hL3_im = hL3[0].im;
        b[0] = hL3_re;
        b[3] = hL3_im;
        hL3_re = hL3[1].re;
        hL3_im = hL3[1].im;
        b[1] = hL3_re * 5.0;
        b[4] = hL3_im * 5.0;
        hL3_re = hL3[2].re;
        hL3_im = hL3[2].im;
        b[2] = hL3_re;
        b[5] = hL3_im;
        b[6] = -frequencies[1];
        AutoPIDTun_utilLSQFixedSizeData(A, b, x);
        *P = x[0];
        gammaHigh = 0.0;
        rLast = 0.0;
        hI_idx_1_re = 0.0;
      } else {
        *P = 1.0 / rt_hypotd_snf(responses[2].re, responses[2].im);
        gammaHigh = 0.0;
        rLast = 0.0;
        hI_idx_1_re = 0.0;
      }
    }
    break;

   case 2:
    {
      responses_1[0] = responses[1].re * hI_idx_0_re - responses[1].im *
        hI_idx_0_im;
      responses_1[3] = responses[1].re * hI_idx_0_im + responses[1].im *
        hI_idx_0_re;
      hI_idx_0_re = hI_idx_1_re;
      hI_idx_0_im = hI_idx_1_im;
      responses_1[1] = responses[2].re * hI_idx_0_re - responses[2].im *
        hI_idx_0_im;
      responses_1[4] = responses[2].re * hI_idx_0_im + responses[2].im *
        hI_idx_0_re;
      hI_idx_0_re = hI_idx_2_re;
      hI_idx_0_im = hI_idx_2_im;
      responses_1[2] = responses[3].re * hI_idx_0_re - responses[3].im *
        hI_idx_0_im;
      responses_1[5] = responses[3].re * hI_idx_0_im + responses[3].im *
        hI_idx_0_re;
      AutoPIDTuner_blkdiag(responses_1, A);
      A[6] = -AutoPIDTuner_minimum(&f[0]);
      for (int32_T h_k{0}; h_k < 14; h_k++) {
        rLast = A[h_k];
        rLast *= static_cast<real_T>(d[h_k]);
        A[h_k] = rLast;
      }

      real_T hL3_im;
      hL3_re = hL3[0].re;
      hL3_im = hL3[0].im;
      b[0] = hL3_re;
      b[3] = hL3_im;
      hL3_re = hL3[1].re;
      hL3_im = hL3[1].im;
      b[1] = hL3_re * 5.0;
      b[4] = hL3_im * 5.0;
      hL3_re = hL3[2].re;
      hL3_im = hL3[2].im;
      b[2] = hL3_re;
      b[5] = hL3_im;
      b[6] = -frequencies[1];
      AutoPIDTun_utilLSQFixedSizeData(A, b, x);
      *P = 0.0;
      gammaHigh = x[0];
      rLast = 0.0;
      hI_idx_1_re = 0.0;
    }
    break;

   case 3:
    {
      responses_0[0] = responses[1].re;
      responses_0[6] = responses[1].re * hI_idx_0_re - responses[1].im *
        hI_idx_0_im;
      responses_0[3] = responses[1].im;
      responses_0[9] = responses[1].re * hI_idx_0_im + responses[1].im *
        hI_idx_0_re;
      hI_idx_0_re = hI_idx_1_re;
      hI_idx_0_im = hI_idx_1_im;
      responses_0[1] = responses[2].re;
      responses_0[7] = responses[2].re * hI_idx_0_re - responses[2].im *
        hI_idx_0_im;
      responses_0[4] = responses[2].im;
      responses_0[10] = responses[2].re * hI_idx_0_im + responses[2].im *
        hI_idx_0_re;
      hI_idx_0_re = hI_idx_2_re;
      hI_idx_0_im = hI_idx_2_im;
      responses_0[2] = responses[3].re;
      responses_0[8] = responses[3].re * hI_idx_0_re - responses[3].im *
        hI_idx_0_im;
      responses_0[5] = responses[3].im;
      responses_0[11] = responses[3].re * hI_idx_0_im + responses[3].im *
        hI_idx_0_re;
      AutoPIDTuner_blkdiag_c(responses_0, b_A);
      b_A[13] = -AutoPIDTuner_minimum(&f[0]);
      for (int32_T h_k{0}; h_k < 21; h_k++) {
        hI_idx_0_re = b_A[h_k];
        hI_idx_0_re *= static_cast<real_T>(e[h_k]);
        b_A[h_k] = hI_idx_0_re;
      }

      real_T hL3_im;
      hL3_re = hL3[0].re;
      hL3_im = hL3[0].im;
      b[0] = hL3_re;
      b[3] = hL3_im;
      hL3_re = hL3[1].re;
      hL3_im = hL3[1].im;
      b[1] = hL3_re * 5.0;
      b[4] = hL3_im * 5.0;
      hL3_re = hL3[2].re;
      hL3_im = hL3[2].im;
      b[2] = hL3_re;
      b[5] = hL3_im;
      b[6] = -frequencies[1];
      AutoPIDT_utilLSQFixedSizeData_j(b_A, b, rG3);
      *P = rG3[0];
      gammaHigh = rG3[1];
      rLast = 0.0;
      hI_idx_1_re = 0.0;
    }
    break;

   case 4:
    {
      if (HasIntegrator != 0.0) {
        real_T hD_idx_0_im;
        real_T hD_idx_0_re;
        real_T hD_idx_1_im;
        real_T hD_idx_1_re;
        real_T hD_idx_2_im;
        real_T hD_idx_2_re;
        if (IsDiscrete) {
          hL3_re = rG3[0];
          if (hL3_re == 0.0) {
            hD_idx_0_re = 1.0 / rLast;
            hD_idx_0_im = 0.0;
          } else if (rLast == 0.0) {
            hD_idx_0_re = 0.0;
            hD_idx_0_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(rLast);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / rLast;
              brm = bim * hL3_re + rLast;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            } else {
              bim = rLast / hL3_re;
              brm = bim * rLast + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            }
          }

          hL3_re = rG3[1];
          if (hL3_re == 0.0) {
            hD_idx_1_re = 1.0 / rLast;
            hD_idx_1_im = 0.0;
          } else if (rLast == 0.0) {
            hD_idx_1_re = 0.0;
            hD_idx_1_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(rLast);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / rLast;
              brm = bim * hL3_re + rLast;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            } else {
              bim = rLast / hL3_re;
              brm = bim * rLast + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            }
          }

          hL3_re = rG3[2];
          if (hL3_re == 0.0) {
            hD_idx_2_re = 1.0 / rLast;
            hD_idx_2_im = 0.0;
          } else if (rLast == 0.0) {
            hD_idx_2_re = 0.0;
            hD_idx_2_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(rLast);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / rLast;
              brm = bim * hL3_re + rLast;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            } else {
              bim = rLast / hL3_re;
              brm = bim * rLast + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            }
          }
        } else {
          hD_idx_0_re = 0.0;
          hD_idx_0_im = frequencies[1];
          hD_idx_1_re = 0.0;
          hD_idx_1_im = frequencies[2];
          hD_idx_2_re = 0.0;
          hD_idx_2_im = frequencies[3];
        }

        responses_0[0] = responses[1].re;
        responses_0[6] = responses[1].re * hD_idx_0_re - responses[1].im *
          hD_idx_0_im;
        responses_0[3] = responses[1].im;
        responses_0[9] = responses[1].im * hD_idx_0_re + responses[1].re *
          hD_idx_0_im;
        hD_idx_0_re = hD_idx_1_re;
        hD_idx_0_im = hD_idx_1_im;
        responses_0[1] = responses[2].re;
        responses_0[7] = responses[2].re * hD_idx_0_re - responses[2].im *
          hD_idx_0_im;
        responses_0[4] = responses[2].im;
        responses_0[10] = responses[2].im * hD_idx_0_re + responses[2].re *
          hD_idx_0_im;
        hD_idx_0_re = hD_idx_2_re;
        hD_idx_0_im = hD_idx_2_im;
        responses_0[2] = responses[3].re;
        responses_0[8] = responses[3].re * hD_idx_0_re - responses[3].im *
          hD_idx_0_im;
        responses_0[5] = responses[3].im;
        responses_0[11] = responses[3].im * hD_idx_0_re + responses[3].re *
          hD_idx_0_im;
        AutoPIDTuner_blkdiag_ch(responses_0, c_A);
        c_A[7] = AutoPIDTuner_maximum_f(&f[50]);
        for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
          __m128d tmp;
          __m128d tmp_0;
          tmp = _mm_loadu_pd(&f[h_k]);
          tmp_0 = _mm_loadu_pd(&wLow[h_k]);
          tmp = _mm_mul_pd(tmp, tmp_0);
          _mm_storeu_pd(&f_0[h_k], tmp);
          tmp = _mm_loadu_pd(&f[h_k + 50]);
          tmp_0 = _mm_loadu_pd(&wHigh[h_k]);
          tmp = _mm_mul_pd(tmp, tmp_0);
          _mm_storeu_pd(&f_1[h_k], tmp);
        }

        c_A[6] = -AutoPIDTuner_minimum(f_0);
        c_A[15] = AutoPIDTuner_maximum_f(f_1);
        for (int32_T h_k{0}; h_k < 32; h_k++) {
          hI_idx_0_re = c_A[h_k];
          hI_idx_0_re *= static_cast<real_T>(g[h_k]);
          c_A[h_k] = hI_idx_0_re;
        }

        real_T hL3_im;
        hL3_re = hL3[0].re;
        hL3_im = hL3[0].im;
        b_b[0] = hL3_re;
        b_b[3] = hL3_im;
        hL3_re = hL3[1].re;
        hL3_im = hL3[1].im;
        b_b[1] = hL3_re * 5.0;
        b_b[4] = hL3_im * 5.0;
        hL3_re = hL3[2].re;
        hL3_im = hL3[2].im;
        b_b[2] = hL3_re;
        b_b[5] = hL3_im;
        b_b[6] = -frequencies[1];
        b_b[7] = gammaHigh;
        AutoPID_utilLSQFixedSizeData_j2(c_A, b_b, b_x);
        *P = b_x[0];
        gammaHigh = 0.0;
        rLast = b_x[1];
        hI_idx_1_re = 0.0;
      } else {
        *P = 1.0 / rt_hypotd_snf(responses[2].re, responses[2].im);
        P_0.re = *P * responses[2].re;
        P_0.im = *P * responses[2].im;
        if (AutoPIDTuner_computePM(P_0) > targetPM) {
          gammaHigh = 0.0;
          rLast = 0.0;
          hI_idx_1_re = 0.0;
        } else {
          real_T hD_idx_0_im;
          real_T hD_idx_0_re;
          real_T hD_idx_1_im;
          real_T hD_idx_1_re;
          real_T hD_idx_2_im;
          real_T hD_idx_2_re;
          if (IsDiscrete) {
            hL3_re = rG3[0];
            if (hL3_re == 0.0) {
              hD_idx_0_re = 1.0 / rLast;
              hD_idx_0_im = 0.0;
            } else if (rLast == 0.0) {
              hD_idx_0_re = 0.0;
              hD_idx_0_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(rLast);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / rLast;
                brm = bim * hL3_re + rLast;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              } else {
                bim = rLast / hL3_re;
                brm = bim * rLast + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              }
            }

            hL3_re = rG3[1];
            if (hL3_re == 0.0) {
              hD_idx_1_re = 1.0 / rLast;
              hD_idx_1_im = 0.0;
            } else if (rLast == 0.0) {
              hD_idx_1_re = 0.0;
              hD_idx_1_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(rLast);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / rLast;
                brm = bim * hL3_re + rLast;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              } else {
                bim = rLast / hL3_re;
                brm = bim * rLast + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              }
            }

            hL3_re = rG3[2];
            if (hL3_re == 0.0) {
              hD_idx_2_re = 1.0 / rLast;
              hD_idx_2_im = 0.0;
            } else if (rLast == 0.0) {
              hD_idx_2_re = 0.0;
              hD_idx_2_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(rLast);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / rLast;
                brm = bim * hL3_re + rLast;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              } else {
                bim = rLast / hL3_re;
                brm = bim * rLast + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              }
            }
          } else {
            hD_idx_0_re = 0.0;
            hD_idx_0_im = frequencies[1];
            hD_idx_1_re = 0.0;
            hD_idx_1_im = frequencies[2];
            hD_idx_2_re = 0.0;
            hD_idx_2_im = frequencies[3];
          }

          responses_0[0] = responses[1].re;
          responses_0[6] = responses[1].re * hD_idx_0_re - responses[1].im *
            hD_idx_0_im;
          responses_0[3] = responses[1].im;
          responses_0[9] = responses[1].im * hD_idx_0_re + responses[1].re *
            hD_idx_0_im;
          hD_idx_0_re = hD_idx_1_re;
          hD_idx_0_im = hD_idx_1_im;
          responses_0[1] = responses[2].re;
          responses_0[7] = responses[2].re * hD_idx_0_re - responses[2].im *
            hD_idx_0_im;
          responses_0[4] = responses[2].im;
          responses_0[10] = responses[2].im * hD_idx_0_re + responses[2].re *
            hD_idx_0_im;
          hD_idx_0_re = hD_idx_2_re;
          hD_idx_0_im = hD_idx_2_im;
          responses_0[2] = responses[3].re;
          responses_0[8] = responses[3].re * hD_idx_0_re - responses[3].im *
            hD_idx_0_im;
          responses_0[5] = responses[3].im;
          responses_0[11] = responses[3].im * hD_idx_0_re + responses[3].re *
            hD_idx_0_im;
          AutoPIDTuner_blkdiag_c(responses_0, b_A);
          b_A[6] = AutoPIDTuner_maximum_f(&f[50]);
          for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
            __m128d tmp;
            __m128d tmp_0;
            tmp = _mm_loadu_pd(&f[h_k + 50]);
            tmp_0 = _mm_loadu_pd(&wHigh[h_k]);
            tmp = _mm_mul_pd(tmp, tmp_0);
            _mm_storeu_pd(&f_0[h_k], tmp);
          }

          b_A[13] = AutoPIDTuner_maximum_f(f_0);
          for (int32_T h_k{0}; h_k < 21; h_k++) {
            hI_idx_0_re = b_A[h_k];
            hI_idx_0_re *= static_cast<real_T>(e[h_k]);
            b_A[h_k] = hI_idx_0_re;
          }

          real_T hL3_im;
          hL3_re = hL3[0].re;
          hL3_im = hL3[0].im;
          b[0] = hL3_re;
          b[3] = hL3_im;
          hL3_re = hL3[1].re;
          hL3_im = hL3[1].im;
          b[1] = hL3_re * 5.0;
          b[4] = hL3_im * 5.0;
          hL3_re = hL3[2].re;
          hL3_im = hL3[2].im;
          b[2] = hL3_re;
          b[5] = hL3_im;
          b[6] = gammaHigh;
          AutoPIDT_utilLSQFixedSizeData_j(b_A, b, rG3);
          *P = rG3[0];
          gammaHigh = 0.0;
          rLast = rG3[1];
          hI_idx_1_re = 0.0;
        }
      }
    }
    break;

   case 5:
    {
      if (HasIntegrator != 0.0) {
        real_T hL3_im;
        AutoPIDTuner_computeTAU(IsDiscrete, &frequencies[1], Ts, DF, tau);
        hL3_re = hL3[0].re;
        hL3_im = hL3[0].im;
        b_b[0] = hL3_re;
        b_b[3] = hL3_im;
        hL3_re = hL3[1].re;
        hL3_im = hL3[1].im;
        b_b[1] = hL3_re * 5.0;
        b_b[4] = hL3_im * 5.0;
        hL3_re = hL3[2].re;
        hL3_im = hL3[2].im;
        b_b[2] = hL3_re;
        b_b[5] = hL3_im;
        b_b[6] = -frequencies[1];
        b_b[7] = gammaHigh;
        responses_0[0] = responses[1].re;
        responses_0[3] = responses[1].im;
        responses_0[1] = responses[2].re;
        responses_0[4] = responses[2].im;
        responses_0[2] = responses[3].re;
        responses_0[5] = responses[3].im;
        for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
          __m128d tmp;
          __m128d tmp_0;
          tmp = _mm_loadu_pd(&f[h_k]);
          tmp_0 = _mm_loadu_pd(&wLow[h_k]);
          tmp = _mm_mul_pd(tmp, tmp_0);
          _mm_storeu_pd(&f_0[h_k], tmp);
        }

        for (imin = 0; imin < 20; imin++) {
          real_T hD_idx_0_im;
          real_T hD_idx_0_re;
          real_T hD_idx_1_im;
          real_T hD_idx_1_re;
          real_T hD_idx_2_im;
          real_T hD_idx_2_re;
          hL3_im = tau[imin];
          if (IsDiscrete) {
            gammaHigh = hL3_im + rLast;
            hL3_re = rG3[0];
            if (hL3_re == 0.0) {
              hD_idx_0_re = 1.0 / gammaHigh;
              hD_idx_0_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_0_re = 0.0;
              hD_idx_0_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(gammaHigh);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / gammaHigh;
                brm = bim * hL3_re + gammaHigh;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              } else {
                bim = gammaHigh / hL3_re;
                brm = bim * gammaHigh + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_0_re = hL3_re / brm;
                hD_idx_0_im = bim / brm;
              }
            }

            hL3_re = rG3[1];
            if (hL3_re == 0.0) {
              hD_idx_1_re = 1.0 / gammaHigh;
              hD_idx_1_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_1_re = 0.0;
              hD_idx_1_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(gammaHigh);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / gammaHigh;
                brm = bim * hL3_re + gammaHigh;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              } else {
                bim = gammaHigh / hL3_re;
                brm = bim * gammaHigh + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_1_re = hL3_re / brm;
                hD_idx_1_im = bim / brm;
              }
            }

            hL3_re = rG3[2];
            if (hL3_re == 0.0) {
              hD_idx_2_re = 1.0 / gammaHigh;
              hD_idx_2_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_2_re = 0.0;
              hD_idx_2_im = -(1.0 / hL3_re);
            } else {
              brm = std::abs(gammaHigh);
              bim = std::abs(hL3_re);
              if (brm > bim) {
                bim = hL3_re / gammaHigh;
                brm = bim * hL3_re + gammaHigh;
                hL3_re = bim * 0.0 + 1.0;
                bim = 0.0 - bim;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              } else if (bim == brm) {
                hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hL3_re = 0.0 * bim + hD_idx_2_re;
                bim = 0.0 * hD_idx_2_re - bim;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              } else {
                bim = gammaHigh / hL3_re;
                brm = bim * gammaHigh + hL3_re;
                hL3_re = bim;
                bim = bim * 0.0 - 1.0;
                hD_idx_2_re = hL3_re / brm;
                hD_idx_2_im = bim / brm;
              }
            }
          } else {
            gammaHigh = 0.0 * frequencies[1];
            hD_idx_0_im = frequencies[1];
            hL3_re = hL3_im * frequencies[1];
            if (hL3_re == 0.0) {
              if (hD_idx_0_im == 0.0) {
                hD_idx_0_re = gammaHigh;
                hD_idx_0_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_0_re = 0.0;
              } else {
                hD_idx_0_re = (rtNaN);
              }
            } else {
              bim = std::abs(hL3_re);
              if (bim < 1.0) {
                bim = hL3_re;
                brm = bim * hL3_re + 1.0;
                hD_idx_0_re = bim * hD_idx_0_im + gammaHigh;
                hD_idx_0_im -= bim * gammaHigh;
                hD_idx_0_re /= brm;
                hD_idx_0_im /= brm;
              } else if (bim == 1.0) {
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hD_idx_0_re = gammaHigh * 0.5 + hD_idx_0_im * bim;
                hD_idx_0_im = hD_idx_0_im * 0.5 - gammaHigh * bim;
              } else {
                bim = 1.0 / hL3_re;
                brm = hL3_re + bim;
                hD_idx_0_re = bim * gammaHigh + hD_idx_0_im;
                hD_idx_0_im = bim * hD_idx_0_im - gammaHigh;
                hD_idx_0_re /= brm;
                hD_idx_0_im /= brm;
              }
            }

            gammaHigh = 0.0 * frequencies[2];
            hD_idx_1_im = frequencies[2];
            hL3_re = hL3_im * frequencies[2];
            if (hL3_re == 0.0) {
              if (hD_idx_1_im == 0.0) {
                hD_idx_1_re = gammaHigh;
                hD_idx_1_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_1_re = 0.0;
              } else {
                hD_idx_1_re = (rtNaN);
              }
            } else {
              bim = std::abs(hL3_re);
              if (bim < 1.0) {
                bim = hL3_re;
                brm = bim * hL3_re + 1.0;
                hD_idx_1_re = bim * hD_idx_1_im + gammaHigh;
                hD_idx_1_im -= bim * gammaHigh;
                hD_idx_1_re /= brm;
                hD_idx_1_im /= brm;
              } else if (bim == 1.0) {
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hD_idx_1_re = gammaHigh * 0.5 + hD_idx_1_im * bim;
                hD_idx_1_im = hD_idx_1_im * 0.5 - gammaHigh * bim;
              } else {
                bim = 1.0 / hL3_re;
                brm = hL3_re + bim;
                hD_idx_1_re = bim * gammaHigh + hD_idx_1_im;
                hD_idx_1_im = bim * hD_idx_1_im - gammaHigh;
                hD_idx_1_re /= brm;
                hD_idx_1_im /= brm;
              }
            }

            gammaHigh = 0.0 * frequencies[3];
            hD_idx_2_im = frequencies[3];
            hL3_re = hL3_im * frequencies[3];
            if (hL3_re == 0.0) {
              if (hD_idx_2_im == 0.0) {
                hD_idx_2_re = gammaHigh;
                hD_idx_2_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_2_re = 0.0;
              } else {
                hD_idx_2_re = (rtNaN);
              }
            } else {
              bim = std::abs(hL3_re);
              if (bim < 1.0) {
                bim = hL3_re;
                brm = bim * hL3_re + 1.0;
                hD_idx_2_re = bim * hD_idx_2_im + gammaHigh;
                hD_idx_2_im -= bim * gammaHigh;
                hD_idx_2_re /= brm;
                hD_idx_2_im /= brm;
              } else if (bim == 1.0) {
                bim = hL3_re > 0.0 ? 0.5 : -0.5;
                hD_idx_2_re = gammaHigh * 0.5 + hD_idx_2_im * bim;
                hD_idx_2_im = hD_idx_2_im * 0.5 - gammaHigh * bim;
              } else {
                bim = 1.0 / hL3_re;
                brm = hL3_re + bim;
                hD_idx_2_re = bim * gammaHigh + hD_idx_2_im;
                hD_idx_2_im = bim * hD_idx_2_im - gammaHigh;
                hD_idx_2_re /= brm;
                hD_idx_2_im /= brm;
              }
            }
          }

          responses_0[6] = responses[1].re * hD_idx_0_re - responses[1].im *
            hD_idx_0_im;
          responses_0[9] = responses[1].im * hD_idx_0_re + responses[1].re *
            hD_idx_0_im;
          hD_idx_0_re = hD_idx_1_re;
          hD_idx_0_im = hD_idx_1_im;
          responses_0[7] = responses[2].re * hD_idx_0_re - responses[2].im *
            hD_idx_0_im;
          responses_0[10] = responses[2].im * hD_idx_0_re + responses[2].re *
            hD_idx_0_im;
          hD_idx_0_re = hD_idx_2_re;
          hD_idx_0_im = hD_idx_2_im;
          responses_0[8] = responses[3].re * hD_idx_0_re - responses[3].im *
            hD_idx_0_im;
          responses_0[11] = responses[3].im * hD_idx_0_re + responses[3].re *
            hD_idx_0_im;
          AutoPIDTuner_blkdiag_ch(responses_0, c_A);
          c_A[6] = -AutoPIDTuner_minimum(f_0);
          c_A[7] = AutoPIDTuner_maximum_f(&f[50]);
          for (int32_T h_k{0}; h_k < 50; h_k++) {
            gammaHigh = wHigh[h_k];
            d_x.im = gammaHigh * hL3_im;
            hL3_re = rt_hypotd_snf(1.0, d_x.im);
            f_1[h_k] = f[h_k + 50] * gammaHigh / hL3_re;
          }

          c_A[15] = AutoPIDTuner_maximum_f(f_1);
          for (int32_T h_k{0}; h_k < 32; h_k++) {
            hI_idx_0_re = c_A[h_k];
            hI_idx_0_re *= static_cast<real_T>(g[h_k]);
            c_A[h_k] = hI_idx_0_re;
          }

          AutoPI_utilLSQFixedSizeData_j2z(c_A, b_b, &b_xtau[imin << 2],
            &gap[imin]);
        }

        if ((AutoPIDTuner_maximum_f5j(gap) - AutoPIDTuner_minimum_g(gap)) /
            AutoPIDTuner_mean(gap) < 0.1) {
          imin = 0;
        } else {
          AutoPIDTuner_minimum_g1(gap, &gammaHigh, &imin);
          imin--;
        }

        if (b_xtau[(imin << 2) + 1] == 0.0) {
          *P = b_xtau[imin << 2];
          gammaHigh = 0.0;
          rLast = b_xtau[(imin << 2) + 1];
          hI_idx_1_re = 0.0;
        } else {
          *P = b_xtau[imin << 2];
          gammaHigh = 0.0;
          rLast = b_xtau[(imin << 2) + 1];
          hI_idx_1_re = tau[imin];
        }
      } else {
        *P = 1.0 / rt_hypotd_snf(responses[2].re, responses[2].im);
        P_0.re = *P * responses[2].re;
        P_0.im = *P * responses[2].im;
        if (AutoPIDTuner_computePM(P_0) > targetPM) {
          gammaHigh = 0.0;
          rLast = 0.0;
          hI_idx_1_re = 0.0;
        } else {
          real_T hL3_im;
          AutoPIDTuner_computeTAU(IsDiscrete, &frequencies[1], Ts, DF, tau);
          hL3_re = hL3[0].re;
          hL3_im = hL3[0].im;
          b[0] = hL3_re;
          b[3] = hL3_im;
          hL3_re = hL3[1].re;
          hL3_im = hL3[1].im;
          b[1] = hL3_re * 5.0;
          b[4] = hL3_im * 5.0;
          hL3_re = hL3[2].re;
          hL3_im = hL3[2].im;
          b[2] = hL3_re;
          b[5] = hL3_im;
          b[6] = gammaHigh;
          responses_0[0] = responses[1].re;
          responses_0[3] = responses[1].im;
          responses_0[1] = responses[2].re;
          responses_0[4] = responses[2].im;
          responses_0[2] = responses[3].re;
          responses_0[5] = responses[3].im;
          for (imin = 0; imin < 20; imin++) {
            real_T hD_idx_0_im;
            real_T hD_idx_0_re;
            real_T hD_idx_1_im;
            real_T hD_idx_1_re;
            real_T hD_idx_2_im;
            real_T hD_idx_2_re;
            hL3_im = tau[imin];
            if (IsDiscrete) {
              gammaHigh = hL3_im + rLast;
              hL3_re = rG3[0];
              if (hL3_re == 0.0) {
                hD_idx_0_re = 1.0 / gammaHigh;
                hD_idx_0_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_0_re = 0.0;
                hD_idx_0_im = -(1.0 / hL3_re);
              } else {
                brm = std::abs(gammaHigh);
                bim = std::abs(hL3_re);
                if (brm > bim) {
                  bim = hL3_re / gammaHigh;
                  brm = bim * hL3_re + gammaHigh;
                  hL3_re = bim * 0.0 + 1.0;
                  bim = 0.0 - bim;
                  hD_idx_0_re = hL3_re / brm;
                  hD_idx_0_im = bim / brm;
                } else if (bim == brm) {
                  hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hL3_re = 0.0 * bim + hD_idx_2_re;
                  bim = 0.0 * hD_idx_2_re - bim;
                  hD_idx_0_re = hL3_re / brm;
                  hD_idx_0_im = bim / brm;
                } else {
                  bim = gammaHigh / hL3_re;
                  brm = bim * gammaHigh + hL3_re;
                  hL3_re = bim;
                  bim = bim * 0.0 - 1.0;
                  hD_idx_0_re = hL3_re / brm;
                  hD_idx_0_im = bim / brm;
                }
              }

              hL3_re = rG3[1];
              if (hL3_re == 0.0) {
                hD_idx_1_re = 1.0 / gammaHigh;
                hD_idx_1_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_1_re = 0.0;
                hD_idx_1_im = -(1.0 / hL3_re);
              } else {
                brm = std::abs(gammaHigh);
                bim = std::abs(hL3_re);
                if (brm > bim) {
                  bim = hL3_re / gammaHigh;
                  brm = bim * hL3_re + gammaHigh;
                  hL3_re = bim * 0.0 + 1.0;
                  bim = 0.0 - bim;
                  hD_idx_1_re = hL3_re / brm;
                  hD_idx_1_im = bim / brm;
                } else if (bim == brm) {
                  hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hL3_re = 0.0 * bim + hD_idx_2_re;
                  bim = 0.0 * hD_idx_2_re - bim;
                  hD_idx_1_re = hL3_re / brm;
                  hD_idx_1_im = bim / brm;
                } else {
                  bim = gammaHigh / hL3_re;
                  brm = bim * gammaHigh + hL3_re;
                  hL3_re = bim;
                  bim = bim * 0.0 - 1.0;
                  hD_idx_1_re = hL3_re / brm;
                  hD_idx_1_im = bim / brm;
                }
              }

              hL3_re = rG3[2];
              if (hL3_re == 0.0) {
                hD_idx_2_re = 1.0 / gammaHigh;
                hD_idx_2_im = 0.0;
              } else if (gammaHigh == 0.0) {
                hD_idx_2_re = 0.0;
                hD_idx_2_im = -(1.0 / hL3_re);
              } else {
                brm = std::abs(gammaHigh);
                bim = std::abs(hL3_re);
                if (brm > bim) {
                  bim = hL3_re / gammaHigh;
                  brm = bim * hL3_re + gammaHigh;
                  hL3_re = bim * 0.0 + 1.0;
                  bim = 0.0 - bim;
                  hD_idx_2_re = hL3_re / brm;
                  hD_idx_2_im = bim / brm;
                } else if (bim == brm) {
                  hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hL3_re = 0.0 * bim + hD_idx_2_re;
                  bim = 0.0 * hD_idx_2_re - bim;
                  hD_idx_2_re = hL3_re / brm;
                  hD_idx_2_im = bim / brm;
                } else {
                  bim = gammaHigh / hL3_re;
                  brm = bim * gammaHigh + hL3_re;
                  hL3_re = bim;
                  bim = bim * 0.0 - 1.0;
                  hD_idx_2_re = hL3_re / brm;
                  hD_idx_2_im = bim / brm;
                }
              }
            } else {
              gammaHigh = 0.0 * frequencies[1];
              hD_idx_0_im = frequencies[1];
              hL3_re = hL3_im * frequencies[1];
              if (hL3_re == 0.0) {
                if (hD_idx_0_im == 0.0) {
                  hD_idx_0_re = gammaHigh;
                  hD_idx_0_im = 0.0;
                } else if (gammaHigh == 0.0) {
                  hD_idx_0_re = 0.0;
                } else {
                  hD_idx_0_re = (rtNaN);
                }
              } else {
                bim = std::abs(hL3_re);
                if (bim < 1.0) {
                  bim = hL3_re;
                  brm = bim * hL3_re + 1.0;
                  hD_idx_0_re = bim * hD_idx_0_im + gammaHigh;
                  hD_idx_0_im -= bim * gammaHigh;
                  hD_idx_0_re /= brm;
                  hD_idx_0_im /= brm;
                } else if (bim == 1.0) {
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hD_idx_0_re = gammaHigh * 0.5 + hD_idx_0_im * bim;
                  hD_idx_0_im = hD_idx_0_im * 0.5 - gammaHigh * bim;
                } else {
                  bim = 1.0 / hL3_re;
                  brm = hL3_re + bim;
                  hD_idx_0_re = bim * gammaHigh + hD_idx_0_im;
                  hD_idx_0_im = bim * hD_idx_0_im - gammaHigh;
                  hD_idx_0_re /= brm;
                  hD_idx_0_im /= brm;
                }
              }

              gammaHigh = 0.0 * frequencies[2];
              hD_idx_1_im = frequencies[2];
              hL3_re = hL3_im * frequencies[2];
              if (hL3_re == 0.0) {
                if (hD_idx_1_im == 0.0) {
                  hD_idx_1_re = gammaHigh;
                  hD_idx_1_im = 0.0;
                } else if (gammaHigh == 0.0) {
                  hD_idx_1_re = 0.0;
                } else {
                  hD_idx_1_re = (rtNaN);
                }
              } else {
                bim = std::abs(hL3_re);
                if (bim < 1.0) {
                  bim = hL3_re;
                  brm = bim * hL3_re + 1.0;
                  hD_idx_1_re = bim * hD_idx_1_im + gammaHigh;
                  hD_idx_1_im -= bim * gammaHigh;
                  hD_idx_1_re /= brm;
                  hD_idx_1_im /= brm;
                } else if (bim == 1.0) {
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hD_idx_1_re = gammaHigh * 0.5 + hD_idx_1_im * bim;
                  hD_idx_1_im = hD_idx_1_im * 0.5 - gammaHigh * bim;
                } else {
                  bim = 1.0 / hL3_re;
                  brm = hL3_re + bim;
                  hD_idx_1_re = bim * gammaHigh + hD_idx_1_im;
                  hD_idx_1_im = bim * hD_idx_1_im - gammaHigh;
                  hD_idx_1_re /= brm;
                  hD_idx_1_im /= brm;
                }
              }

              gammaHigh = 0.0 * frequencies[3];
              hD_idx_2_im = frequencies[3];
              hL3_re = hL3_im * frequencies[3];
              if (hL3_re == 0.0) {
                if (hD_idx_2_im == 0.0) {
                  hD_idx_2_re = gammaHigh;
                  hD_idx_2_im = 0.0;
                } else if (gammaHigh == 0.0) {
                  hD_idx_2_re = 0.0;
                } else {
                  hD_idx_2_re = (rtNaN);
                }
              } else {
                bim = std::abs(hL3_re);
                if (bim < 1.0) {
                  bim = hL3_re;
                  brm = bim * hL3_re + 1.0;
                  hD_idx_2_re = bim * hD_idx_2_im + gammaHigh;
                  hD_idx_2_im -= bim * gammaHigh;
                  hD_idx_2_re /= brm;
                  hD_idx_2_im /= brm;
                } else if (bim == 1.0) {
                  bim = hL3_re > 0.0 ? 0.5 : -0.5;
                  hD_idx_2_re = gammaHigh * 0.5 + hD_idx_2_im * bim;
                  hD_idx_2_im = hD_idx_2_im * 0.5 - gammaHigh * bim;
                } else {
                  bim = 1.0 / hL3_re;
                  brm = hL3_re + bim;
                  hD_idx_2_re = bim * gammaHigh + hD_idx_2_im;
                  hD_idx_2_im = bim * hD_idx_2_im - gammaHigh;
                  hD_idx_2_re /= brm;
                  hD_idx_2_im /= brm;
                }
              }
            }

            responses_0[6] = responses[1].re * hD_idx_0_re - responses[1].im *
              hD_idx_0_im;
            responses_0[9] = responses[1].im * hD_idx_0_re + responses[1].re *
              hD_idx_0_im;
            hD_idx_0_re = hD_idx_1_re;
            hD_idx_0_im = hD_idx_1_im;
            responses_0[7] = responses[2].re * hD_idx_0_re - responses[2].im *
              hD_idx_0_im;
            responses_0[10] = responses[2].im * hD_idx_0_re + responses[2].re *
              hD_idx_0_im;
            hD_idx_0_re = hD_idx_2_re;
            hD_idx_0_im = hD_idx_2_im;
            responses_0[8] = responses[3].re * hD_idx_0_re - responses[3].im *
              hD_idx_0_im;
            responses_0[11] = responses[3].im * hD_idx_0_re + responses[3].re *
              hD_idx_0_im;
            AutoPIDTuner_blkdiag_c(responses_0, b_A);
            b_A[6] = AutoPIDTuner_maximum_f(&f[50]);
            for (int32_T h_k{0}; h_k < 50; h_k++) {
              gammaHigh = wHigh[h_k];
              d_x.im = gammaHigh * hL3_im;
              hL3_re = rt_hypotd_snf(1.0, d_x.im);
              f_0[h_k] = f[h_k + 50] * gammaHigh / hL3_re;
            }

            b_A[13] = AutoPIDTuner_maximum_f(f_0);
            for (int32_T h_k{0}; h_k < 21; h_k++) {
              hI_idx_0_re = b_A[h_k];
              hI_idx_0_re *= static_cast<real_T>(e[h_k]);
              b_A[h_k] = hI_idx_0_re;
            }

            AutoP_utilLSQFixedSizeData_j2zd(b_A, b, &xtau[3 * imin], &gap[imin]);
          }

          if ((AutoPIDTuner_maximum_f5j(gap) - AutoPIDTuner_minimum_g(gap)) /
              AutoPIDTuner_mean(gap) < 0.1) {
            imin = 0;
          } else {
            AutoPIDTuner_minimum_g1(gap, &gammaHigh, &imin);
            imin--;
          }

          if (xtau[3 * imin + 1] == 0.0) {
            *P = xtau[3 * imin];
            gammaHigh = 0.0;
            rLast = xtau[3 * imin + 1];
            hI_idx_1_re = 0.0;
          } else {
            *P = xtau[3 * imin];
            gammaHigh = 0.0;
            rLast = xtau[3 * imin + 1];
            hI_idx_1_re = tau[imin];
          }
        }
      }
    }
    break;

   case 6:
    {
      real_T hD_idx_0_im;
      real_T hD_idx_0_re;
      real_T hD_idx_1_im;
      real_T hD_idx_1_re;
      real_T hD_idx_2_im;
      real_T hD_idx_2_re;
      if (IsDiscrete) {
        hL3_re = rG3[0];
        if (hL3_re == 0.0) {
          hD_idx_0_re = 1.0 / rLast;
          hD_idx_0_im = 0.0;
        } else if (rLast == 0.0) {
          hD_idx_0_re = 0.0;
          hD_idx_0_im = -(1.0 / hL3_re);
        } else {
          brm = std::abs(rLast);
          bim = std::abs(hL3_re);
          if (brm > bim) {
            bim = hL3_re / rLast;
            brm = bim * hL3_re + rLast;
            hL3_re = bim * 0.0 + 1.0;
            bim = 0.0 - bim;
            hD_idx_0_re = hL3_re / brm;
            hD_idx_0_im = bim / brm;
          } else if (bim == brm) {
            hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
            bim = hL3_re > 0.0 ? 0.5 : -0.5;
            hL3_re = 0.0 * bim + hD_idx_2_re;
            bim = 0.0 * hD_idx_2_re - bim;
            hD_idx_0_re = hL3_re / brm;
            hD_idx_0_im = bim / brm;
          } else {
            bim = rLast / hL3_re;
            brm = bim * rLast + hL3_re;
            hL3_re = bim;
            bim = bim * 0.0 - 1.0;
            hD_idx_0_re = hL3_re / brm;
            hD_idx_0_im = bim / brm;
          }
        }

        hL3_re = rG3[1];
        if (hL3_re == 0.0) {
          hD_idx_1_re = 1.0 / rLast;
          hD_idx_1_im = 0.0;
        } else if (rLast == 0.0) {
          hD_idx_1_re = 0.0;
          hD_idx_1_im = -(1.0 / hL3_re);
        } else {
          brm = std::abs(rLast);
          bim = std::abs(hL3_re);
          if (brm > bim) {
            bim = hL3_re / rLast;
            brm = bim * hL3_re + rLast;
            hL3_re = bim * 0.0 + 1.0;
            bim = 0.0 - bim;
            hD_idx_1_re = hL3_re / brm;
            hD_idx_1_im = bim / brm;
          } else if (bim == brm) {
            hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
            bim = hL3_re > 0.0 ? 0.5 : -0.5;
            hL3_re = 0.0 * bim + hD_idx_2_re;
            bim = 0.0 * hD_idx_2_re - bim;
            hD_idx_1_re = hL3_re / brm;
            hD_idx_1_im = bim / brm;
          } else {
            bim = rLast / hL3_re;
            brm = bim * rLast + hL3_re;
            hL3_re = bim;
            bim = bim * 0.0 - 1.0;
            hD_idx_1_re = hL3_re / brm;
            hD_idx_1_im = bim / brm;
          }
        }

        hL3_re = rG3[2];
        if (hL3_re == 0.0) {
          hD_idx_2_re = 1.0 / rLast;
          hD_idx_2_im = 0.0;
        } else if (rLast == 0.0) {
          hD_idx_2_re = 0.0;
          hD_idx_2_im = -(1.0 / hL3_re);
        } else {
          brm = std::abs(rLast);
          bim = std::abs(hL3_re);
          if (brm > bim) {
            bim = hL3_re / rLast;
            brm = bim * hL3_re + rLast;
            hL3_re = bim * 0.0 + 1.0;
            bim = 0.0 - bim;
            hD_idx_2_re = hL3_re / brm;
            hD_idx_2_im = bim / brm;
          } else if (bim == brm) {
            hD_idx_2_re = rLast > 0.0 ? 0.5 : -0.5;
            bim = hL3_re > 0.0 ? 0.5 : -0.5;
            hL3_re = 0.0 * bim + hD_idx_2_re;
            bim = 0.0 * hD_idx_2_re - bim;
            hD_idx_2_re = hL3_re / brm;
            hD_idx_2_im = bim / brm;
          } else {
            bim = rLast / hL3_re;
            brm = bim * rLast + hL3_re;
            hL3_re = bim;
            bim = bim * 0.0 - 1.0;
            hD_idx_2_re = hL3_re / brm;
            hD_idx_2_im = bim / brm;
          }
        }
      } else {
        hD_idx_0_re = 0.0;
        hD_idx_0_im = frequencies[1];
        hD_idx_1_re = 0.0;
        hD_idx_1_im = frequencies[2];
        hD_idx_2_re = 0.0;
        hD_idx_2_im = frequencies[3];
      }

      responses_2[0] = responses[1].re;
      responses_2[6] = responses[1].re * hI_idx_0_re - responses[1].im *
        hI_idx_0_im;
      responses_2[12] = responses[1].re * hD_idx_0_re - responses[1].im *
        hD_idx_0_im;
      responses_2[3] = responses[1].im;
      responses_2[9] = responses[1].re * hI_idx_0_im + responses[1].im *
        hI_idx_0_re;
      responses_2[15] = responses[1].im * hD_idx_0_re + responses[1].re *
        hD_idx_0_im;
      hD_idx_0_re = hD_idx_1_re;
      hD_idx_0_im = hD_idx_1_im;
      hI_idx_0_re = hI_idx_1_re;
      hI_idx_0_im = hI_idx_1_im;
      responses_2[1] = responses[2].re;
      responses_2[7] = responses[2].re * hI_idx_0_re - responses[2].im *
        hI_idx_0_im;
      responses_2[13] = responses[2].re * hD_idx_0_re - responses[2].im *
        hD_idx_0_im;
      responses_2[4] = responses[2].im;
      responses_2[10] = responses[2].re * hI_idx_0_im + responses[2].im *
        hI_idx_0_re;
      responses_2[16] = responses[2].im * hD_idx_0_re + responses[2].re *
        hD_idx_0_im;
      hD_idx_0_re = hD_idx_2_re;
      hD_idx_0_im = hD_idx_2_im;
      hI_idx_0_re = hI_idx_2_re;
      hI_idx_0_im = hI_idx_2_im;
      responses_2[2] = responses[3].re;
      responses_2[8] = responses[3].re * hI_idx_0_re - responses[3].im *
        hI_idx_0_im;
      responses_2[14] = responses[3].re * hD_idx_0_re - responses[3].im *
        hD_idx_0_im;
      responses_2[5] = responses[3].im;
      responses_2[11] = responses[3].re * hI_idx_0_im + responses[3].im *
        hI_idx_0_re;
      responses_2[17] = responses[3].im * hD_idx_0_re + responses[3].re *
        hD_idx_0_im;
      AutoPIDTuner_blkdiag_chw(responses_2, d_A);
      d_A[14] = -AutoPIDTuner_minimum(&f[0]);
      d_A[7] = AutoPIDTuner_maximum_f(&f[50]);
      for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
        __m128d tmp;
        __m128d tmp_0;
        tmp = _mm_loadu_pd(&f[h_k + 50]);
        tmp_0 = _mm_loadu_pd(&wHigh[h_k]);
        tmp = _mm_div_pd(tmp, tmp_0);
        _mm_storeu_pd(&f_0[h_k], tmp);
        tmp = _mm_loadu_pd(&f[h_k + 50]);
        tmp_0 = _mm_loadu_pd(&wHigh[h_k]);
        tmp = _mm_mul_pd(tmp, tmp_0);
        _mm_storeu_pd(&f_1[h_k], tmp);
      }

      d_A[15] = AutoPIDTuner_maximum_f(f_0);
      d_A[23] = AutoPIDTuner_maximum_f(f_1);
      for (int32_T h_k{0}; h_k < 40; h_k++) {
        hL3_re = d_A[h_k];
        hL3_re *= static_cast<real_T>(h[h_k]);
        d_A[h_k] = hL3_re;
      }

      real_T hL3_im;
      hL3_re = hL3[0].re;
      hL3_im = hL3[0].im;
      b_b[0] = hL3_re;
      b_b[3] = hL3_im;
      hL3_re = hL3[1].re;
      hL3_im = hL3[1].im;
      b_b[1] = hL3_re * 5.0;
      b_b[4] = hL3_im * 5.0;
      hL3_re = hL3[2].re;
      hL3_im = hL3[2].im;
      b_b[2] = hL3_re;
      b_b[5] = hL3_im;
      b_b[6] = -frequencies[1];
      b_b[7] = gammaHigh;
      Auto_utilLSQFixedSizeData_j2zds(d_A, b_b, c_x);
      *P = c_x[0];
      gammaHigh = c_x[1];
      rLast = c_x[2];
      hI_idx_1_re = 0.0;
    }
    break;

   case 7:
    {
      real_T hL3_im;
      AutoPIDTuner_computeTAU(IsDiscrete, &frequencies[1], Ts, DF, tau);
      hL3_re = hL3[0].re;
      hL3_im = hL3[0].im;
      b_b[0] = hL3_re;
      b_b[3] = hL3_im;
      hL3_re = hL3[1].re;
      hL3_im = hL3[1].im;
      b_b[1] = hL3_re * 5.0;
      b_b[4] = hL3_im * 5.0;
      hL3_re = hL3[2].re;
      hL3_im = hL3[2].im;
      b_b[2] = hL3_re;
      b_b[5] = hL3_im;
      b_b[6] = -frequencies[1];
      b_b[7] = gammaHigh;
      responses_2[0] = responses[1].re;
      responses_2[3] = responses[1].im;
      responses_2[1] = responses[2].re;
      responses_2[4] = responses[2].im;
      responses_2[2] = responses[3].re;
      responses_2[5] = responses[3].im;
      for (int32_T h_k{0}; h_k <= 48; h_k += 2) {
        __m128d tmp;
        __m128d tmp_0;
        tmp = _mm_loadu_pd(&f[h_k + 50]);
        tmp_0 = _mm_loadu_pd(&wHigh[h_k]);
        tmp = _mm_div_pd(tmp, tmp_0);
        _mm_storeu_pd(&f_0[h_k], tmp);
      }

      for (imin = 0; imin < 20; imin++) {
        real_T hD_idx_0_im;
        real_T hD_idx_0_re;
        real_T hD_idx_1_im;
        real_T hD_idx_1_re;
        real_T hD_idx_2_im;
        real_T hD_idx_2_re;
        hL3_im = tau[imin];
        if (IsDiscrete) {
          gammaHigh = hL3_im + rLast;
          hL3_re = rG3[0];
          if (hL3_re == 0.0) {
            hD_idx_0_re = 1.0 / gammaHigh;
            hD_idx_0_im = 0.0;
          } else if (gammaHigh == 0.0) {
            hD_idx_0_re = 0.0;
            hD_idx_0_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(gammaHigh);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / gammaHigh;
              brm = bim * hL3_re + gammaHigh;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            } else {
              bim = gammaHigh / hL3_re;
              brm = bim * gammaHigh + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_0_re = hL3_re / brm;
              hD_idx_0_im = bim / brm;
            }
          }

          hL3_re = rG3[1];
          if (hL3_re == 0.0) {
            hD_idx_1_re = 1.0 / gammaHigh;
            hD_idx_1_im = 0.0;
          } else if (gammaHigh == 0.0) {
            hD_idx_1_re = 0.0;
            hD_idx_1_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(gammaHigh);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / gammaHigh;
              brm = bim * hL3_re + gammaHigh;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            } else {
              bim = gammaHigh / hL3_re;
              brm = bim * gammaHigh + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_1_re = hL3_re / brm;
              hD_idx_1_im = bim / brm;
            }
          }

          hL3_re = rG3[2];
          if (hL3_re == 0.0) {
            hD_idx_2_re = 1.0 / gammaHigh;
            hD_idx_2_im = 0.0;
          } else if (gammaHigh == 0.0) {
            hD_idx_2_re = 0.0;
            hD_idx_2_im = -(1.0 / hL3_re);
          } else {
            brm = std::abs(gammaHigh);
            bim = std::abs(hL3_re);
            if (brm > bim) {
              bim = hL3_re / gammaHigh;
              brm = bim * hL3_re + gammaHigh;
              hL3_re = bim * 0.0 + 1.0;
              bim = 0.0 - bim;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            } else if (bim == brm) {
              hD_idx_2_re = gammaHigh > 0.0 ? 0.5 : -0.5;
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hL3_re = 0.0 * bim + hD_idx_2_re;
              bim = 0.0 * hD_idx_2_re - bim;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            } else {
              bim = gammaHigh / hL3_re;
              brm = bim * gammaHigh + hL3_re;
              hL3_re = bim;
              bim = bim * 0.0 - 1.0;
              hD_idx_2_re = hL3_re / brm;
              hD_idx_2_im = bim / brm;
            }
          }
        } else {
          gammaHigh = 0.0 * frequencies[1];
          hD_idx_0_im = frequencies[1];
          hL3_re = hL3_im * frequencies[1];
          if (hL3_re == 0.0) {
            if (hD_idx_0_im == 0.0) {
              hD_idx_0_re = gammaHigh;
              hD_idx_0_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_0_re = 0.0;
            } else {
              hD_idx_0_re = (rtNaN);
            }
          } else {
            bim = std::abs(hL3_re);
            if (bim < 1.0) {
              bim = hL3_re;
              brm = bim * hL3_re + 1.0;
              hD_idx_0_re = bim * hD_idx_0_im + gammaHigh;
              hD_idx_0_im -= bim * gammaHigh;
              hD_idx_0_re /= brm;
              hD_idx_0_im /= brm;
            } else if (bim == 1.0) {
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hD_idx_0_re = gammaHigh * 0.5 + hD_idx_0_im * bim;
              hD_idx_0_im = hD_idx_0_im * 0.5 - gammaHigh * bim;
            } else {
              bim = 1.0 / hL3_re;
              brm = hL3_re + bim;
              hD_idx_0_re = bim * gammaHigh + hD_idx_0_im;
              hD_idx_0_im = bim * hD_idx_0_im - gammaHigh;
              hD_idx_0_re /= brm;
              hD_idx_0_im /= brm;
            }
          }

          gammaHigh = 0.0 * frequencies[2];
          hD_idx_1_im = frequencies[2];
          hL3_re = hL3_im * frequencies[2];
          if (hL3_re == 0.0) {
            if (hD_idx_1_im == 0.0) {
              hD_idx_1_re = gammaHigh;
              hD_idx_1_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_1_re = 0.0;
            } else {
              hD_idx_1_re = (rtNaN);
            }
          } else {
            bim = std::abs(hL3_re);
            if (bim < 1.0) {
              bim = hL3_re;
              brm = bim * hL3_re + 1.0;
              hD_idx_1_re = bim * hD_idx_1_im + gammaHigh;
              hD_idx_1_im -= bim * gammaHigh;
              hD_idx_1_re /= brm;
              hD_idx_1_im /= brm;
            } else if (bim == 1.0) {
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hD_idx_1_re = gammaHigh * 0.5 + hD_idx_1_im * bim;
              hD_idx_1_im = hD_idx_1_im * 0.5 - gammaHigh * bim;
            } else {
              bim = 1.0 / hL3_re;
              brm = hL3_re + bim;
              hD_idx_1_re = bim * gammaHigh + hD_idx_1_im;
              hD_idx_1_im = bim * hD_idx_1_im - gammaHigh;
              hD_idx_1_re /= brm;
              hD_idx_1_im /= brm;
            }
          }

          gammaHigh = 0.0 * frequencies[3];
          hD_idx_2_im = frequencies[3];
          hL3_re = hL3_im * frequencies[3];
          if (hL3_re == 0.0) {
            if (hD_idx_2_im == 0.0) {
              hD_idx_2_re = gammaHigh;
              hD_idx_2_im = 0.0;
            } else if (gammaHigh == 0.0) {
              hD_idx_2_re = 0.0;
            } else {
              hD_idx_2_re = (rtNaN);
            }
          } else {
            bim = std::abs(hL3_re);
            if (bim < 1.0) {
              bim = hL3_re;
              brm = bim * hL3_re + 1.0;
              hD_idx_2_re = bim * hD_idx_2_im + gammaHigh;
              hD_idx_2_im -= bim * gammaHigh;
              hD_idx_2_re /= brm;
              hD_idx_2_im /= brm;
            } else if (bim == 1.0) {
              bim = hL3_re > 0.0 ? 0.5 : -0.5;
              hD_idx_2_re = gammaHigh * 0.5 + hD_idx_2_im * bim;
              hD_idx_2_im = hD_idx_2_im * 0.5 - gammaHigh * bim;
            } else {
              bim = 1.0 / hL3_re;
              brm = hL3_re + bim;
              hD_idx_2_re = bim * gammaHigh + hD_idx_2_im;
              hD_idx_2_im = bim * hD_idx_2_im - gammaHigh;
              hD_idx_2_re /= brm;
              hD_idx_2_im /= brm;
            }
          }
        }

        gammaHigh = hI_idx_0_re;
        hL3_re = hI_idx_0_im;
        responses_2[6] = responses[1].re * gammaHigh - responses[1].im * hL3_re;
        responses_2[12] = responses[1].re * hD_idx_0_re - responses[1].im *
          hD_idx_0_im;
        responses_2[9] = responses[1].re * hL3_re + responses[1].im * gammaHigh;
        responses_2[15] = responses[1].im * hD_idx_0_re + responses[1].re *
          hD_idx_0_im;
        hD_idx_0_re = hD_idx_1_re;
        hD_idx_0_im = hD_idx_1_im;
        gammaHigh = hI_idx_1_re;
        hL3_re = hI_idx_1_im;
        responses_2[7] = responses[2].re * gammaHigh - responses[2].im * hL3_re;
        responses_2[13] = responses[2].re * hD_idx_0_re - responses[2].im *
          hD_idx_0_im;
        responses_2[10] = responses[2].re * hL3_re + responses[2].im * gammaHigh;
        responses_2[16] = responses[2].im * hD_idx_0_re + responses[2].re *
          hD_idx_0_im;
        hD_idx_0_re = hD_idx_2_re;
        hD_idx_0_im = hD_idx_2_im;
        gammaHigh = hI_idx_2_re;
        hL3_re = hI_idx_2_im;
        responses_2[8] = responses[3].re * gammaHigh - responses[3].im * hL3_re;
        responses_2[14] = responses[3].re * hD_idx_0_re - responses[3].im *
          hD_idx_0_im;
        responses_2[11] = responses[3].re * hL3_re + responses[3].im * gammaHigh;
        responses_2[17] = responses[3].im * hD_idx_0_re + responses[3].re *
          hD_idx_0_im;
        AutoPIDTuner_blkdiag_chw(responses_2, d_A);
        d_A[14] = -AutoPIDTuner_minimum(&f[0]);
        d_A[7] = AutoPIDTuner_maximum_f(&f[50]);
        d_A[15] = AutoPIDTuner_maximum_f(f_0);
        for (int32_T h_k{0}; h_k < 50; h_k++) {
          gammaHigh = wHigh[h_k];
          d_x.im = gammaHigh * hL3_im;
          hL3_re = rt_hypotd_snf(1.0, d_x.im);
          f_1[h_k] = f[h_k + 50] * gammaHigh / hL3_re;
        }

        d_A[23] = AutoPIDTuner_maximum_f(f_1);
        for (int32_T h_k{0}; h_k < 40; h_k++) {
          hL3_re = d_A[h_k];
          hL3_re *= static_cast<real_T>(h[h_k]);
          d_A[h_k] = hL3_re;
        }

        Aut_utilLSQFixedSizeData_j2zdsx(d_A, b_b, &c_xtau[5 * imin], &gap[imin]);
      }

      if ((AutoPIDTuner_maximum_f5j(gap) - AutoPIDTuner_minimum_g(gap)) /
          AutoPIDTuner_mean(gap) < 0.1) {
        imin = 0;
      } else {
        AutoPIDTuner_minimum_g1(gap, &gammaHigh, &imin);
        imin--;
      }

      if (c_xtau[5 * imin + 2] == 0.0) {
        *P = c_xtau[5 * imin];
        gammaHigh = c_xtau[5 * imin + 1];
        rLast = 0.0;
        hI_idx_1_re = 0.0;
      } else {
        *P = c_xtau[5 * imin];
        gammaHigh = c_xtau[5 * imin + 1];
        rLast = c_xtau[5 * imin + 2];
        hI_idx_1_re = tau[imin];
      }
    }
    break;

   default:
    *P = 0.0;
    gammaHigh = 0.0;
    rLast = 0.0;
    hI_idx_1_re = 0.0;
    break;
  }

  *P *= LoopSign;
  gammaHigh *= LoopSign;
  rLast *= LoopSign;
  if (form == 1) {
    *b_I = gammaHigh;
    *D = rLast;
    if (hI_idx_1_re == 0.0) {
      *N = 100.0;
    } else {
      *N = 1.0 / hI_idx_1_re;
    }
  } else {
    if (*P == 0.0) {
      *b_I = 0.0;
      *D = 0.0;
    } else {
      *b_I = gammaHigh / *P;
      *D = rLast / *P;
    }

    if (hI_idx_1_re == 0.0) {
      *N = 100.0;
    } else {
      *N = 1.0 / hI_idx_1_re;
    }
  }

  hL3_re = frequencies[2];
  if (hL3_re == 0.0) {
    hI_idx_0_re = gammaHigh > 0.0 ? (rtInf) : gammaHigh < 0.0 ? (rtMinusInf) :
      (rtNaN);
    hI_idx_0_im = 0.0;
  } else if (gammaHigh == 0.0) {
    hI_idx_0_re = 0.0 / hL3_re;
    hI_idx_0_im = 0.0;
  } else {
    hI_idx_0_re = 0.0;
    hI_idx_0_im = -(gammaHigh / hL3_re);
  }

  hI_idx_1_im = frequencies[2];
  gammaHigh = rLast * 0.0;
  rLast *= hI_idx_1_im;
  hL3_re = hI_idx_1_re * frequencies[2];
  if (hL3_re == 0.0) {
    if (rLast == 0.0) {
      hL3_re = gammaHigh;
      rLast = 0.0;
    } else if (gammaHigh == 0.0) {
      hL3_re = 0.0;
    } else {
      hL3_re = (rtNaN);
    }
  } else {
    bim = std::abs(hL3_re);
    if (bim < 1.0) {
      bim = hL3_re;
      brm = bim * hL3_re + 1.0;
      hL3_re = bim * rLast + gammaHigh;
      rLast -= bim * gammaHigh;
      hL3_re /= brm;
      rLast /= brm;
    } else if (bim == 1.0) {
      bim = hL3_re > 0.0 ? 0.5 : -0.5;
      hL3_re = gammaHigh * 0.5 + rLast * bim;
      rLast = rLast * 0.5 - gammaHigh * bim;
    } else {
      bim = 1.0 / hL3_re;
      brm = hL3_re + bim;
      hL3_re = bim * gammaHigh + rLast;
      rLast = bim * rLast - gammaHigh;
      hL3_re /= brm;
      rLast /= brm;
    }
  }

  gammaHigh = (*P + hI_idx_0_re) + hL3_re;
  rLast += hI_idx_0_im;
  hI_idx_0_re = gammaHigh * responses[2].re - rLast * responses[2].im;
  rLast = gammaHigh * responses[2].im + rLast * responses[2].re;
  P_1.re = LoopSign * hI_idx_0_re;
  P_1.im = LoopSign * rLast;
  *achievedPM = AutoPIDTuner_computePM(P_1);
}

/* Model step function */
void AutoPIDTuner::step()
{
  creal_T tmp_0[5];
  real_T ai;
  real_T bi;
  real_T bim;
  real_T brm;
  real_T u;
  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    /* set solver stop time */
    if (!((&AutoPIDTuner_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&AutoPIDTuner_M)->solverInfo, (((&AutoPIDTuner_M
        )->Timing.clockTickH0 + 1) * (&AutoPIDTuner_M)->Timing.stepSize0 *
        4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&AutoPIDTuner_M)->solverInfo, (((&AutoPIDTuner_M
        )->Timing.clockTick0 + 1) * (&AutoPIDTuner_M)->Timing.stepSize0 +
        (&AutoPIDTuner_M)->Timing.clockTickH0 * (&AutoPIDTuner_M)
        ->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&AutoPIDTuner_M))) {
    (&AutoPIDTuner_M)->Timing.t[0] = rtsiGetT(&(&AutoPIDTuner_M)->solverInfo);
  }

  /* Sum: '<Root>/Sum' incorporates:
   *  Constant: '<Root>/Ref'
   *  Inport: '<Root>/EncoderPos'
   */
  AutoPIDTuner_B.Sum = AutoPIDTuner_P.Ref_Value + AutoPIDTuner_U.EncoderPos;

  /* Gain: '<S111>/Proportional Gain' */
  AutoPIDTuner_B.ProportionalGain = AutoPIDTuner_P.PIDController_P *
    AutoPIDTuner_B.Sum;

  /* Integrator: '<S106>/Integrator' */
  AutoPIDTuner_B.Integrator = AutoPIDTuner_X.Integrator_CSTATE;

  /* Gain: '<S100>/Derivative Gain' */
  AutoPIDTuner_B.DerivativeGain = AutoPIDTuner_P.PIDController_D *
    AutoPIDTuner_B.Sum;

  /* Integrator: '<S101>/Filter' */
  AutoPIDTuner_B.Filter = AutoPIDTuner_X.Filter_CSTATE;

  /* Sum: '<S101>/SumD' */
  AutoPIDTuner_B.SumD = AutoPIDTuner_B.DerivativeGain - AutoPIDTuner_B.Filter;

  /* Gain: '<S109>/Filter Coefficient' */
  AutoPIDTuner_B.FilterCoefficient = AutoPIDTuner_P.PIDController_N *
    AutoPIDTuner_B.SumD;

  /* Sum: '<S115>/Sum' */
  AutoPIDTuner_B.Sum_l = (AutoPIDTuner_B.ProportionalGain +
    AutoPIDTuner_B.Integrator) + AutoPIDTuner_B.FilterCoefficient;
  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    /* ZeroOrderHold: '<S74>/Zero-Order Hold1' */
    AutoPIDTuner_B.ZeroOrderHold1 = AutoPIDTuner_B.Sum_l;
    for (int32_T i{0}; i <= 2; i += 2) {
      __m128d tmp;

      /* Gain: '<S1>/GainWC' */
      tmp = _mm_loadu_pd(&AutoPIDTuner_P.GainWC_Gain[i]);

      /* Gain: '<S1>/GainWC' incorporates:
       *  Constant: '<S1>/bandwidth constant'
       */
      tmp = _mm_mul_pd(tmp, _mm_set1_pd
                       (AutoPIDTuner_P.ClosedLoopPIDAutotuner_Bandwidt));

      /* Gain: '<S1>/GainWC' */
      _mm_storeu_pd(&AutoPIDTuner_B.w[i], tmp);
    }

    for (int32_T i{4}; i < 5; i++) {
      /* Gain: '<S1>/GainWC' incorporates:
       *  Constant: '<S1>/bandwidth constant'
       */
      AutoPIDTuner_B.w[i] = AutoPIDTuner_P.GainWC_Gain[i] *
        AutoPIDTuner_P.ClosedLoopPIDAutotuner_Bandwidt;
    }

    /* SampleTimeMath: '<S1>/Weighted Ts'
     *
     * About '<S1>/Weighted Ts':
     *  y = K where K = ( w * Ts )
     */
    AutoPIDTuner_B.compiledTs = AutoPIDTuner_P.WeightedTs_WtEt;

    /* Gain: '<S1>/GainTs' */
    AutoPIDTuner_B.GainTs = AutoPIDTuner_P.GainTs_Gain *
      AutoPIDTuner_B.compiledTs;

    /* ManualSwitch: '<Root>/Manual Switch' */
    if (AutoPIDTuner_P.ManualSwitch_CurrentSetting == 1) {
      /* ManualSwitch: '<Root>/Manual Switch' incorporates:
       *  Constant: '<Root>/Constant'
       */
      AutoPIDTuner_B.ManualSwitch = AutoPIDTuner_P.Constant_Value;
    } else {
      /* ManualSwitch: '<Root>/Manual Switch' incorporates:
       *  Constant: '<Root>/Constant1'
       */
      AutoPIDTuner_B.ManualSwitch = AutoPIDTuner_P.Constant1_Value;
    }

    /* End of ManualSwitch: '<Root>/Manual Switch' */

    /* Outputs for Enabled SubSystem: '<S12>/Perturbation Generator' incorporates:
     *  EnablePort: '<S14>/Enable'
     */
    if (rtsiIsModeUpdateTimeStep(&(&AutoPIDTuner_M)->solverInfo)) {
      if (AutoPIDTuner_B.ManualSwitch > 0.0) {
        if (!AutoPIDTuner_DW.PerturbationGenerator_MODE) {
          /* InitializeConditions for UnitDelay: '<S16>/Output' */
          AutoPIDTuner_DW.Output_DSTATE = AutoPIDTuner_P.Output_InitialCondition;
          AutoPIDTuner_DW.PerturbationGenerator_MODE = true;
        }
      } else {
        AutoPIDTuner_DW.PerturbationGenerator_MODE = false;
      }
    }

    if (AutoPIDTuner_DW.PerturbationGenerator_MODE) {
      /* UnitDelay: '<S16>/Output' */
      AutoPIDTuner_B.Output = AutoPIDTuner_DW.Output_DSTATE;

      /* Sum: '<S17>/FixPt Sum1' incorporates:
       *  Constant: '<S17>/FixPt Constant'
       */
      AutoPIDTuner_B.FixPtSum1 = AutoPIDTuner_B.Output +
        AutoPIDTuner_P.FixPtConstant_Value;

      /* Switch: '<S18>/FixPt Switch' */
      if (AutoPIDTuner_B.FixPtSum1 > AutoPIDTuner_P.WrapToZero_Threshold) {
        /* Switch: '<S18>/FixPt Switch' incorporates:
         *  Constant: '<S18>/Constant'
         */
        AutoPIDTuner_B.FixPtSwitch = AutoPIDTuner_P.Constant_Value_f;
      } else {
        /* Switch: '<S18>/FixPt Switch' */
        AutoPIDTuner_B.FixPtSwitch = AutoPIDTuner_B.FixPtSum1;
      }

      /* End of Switch: '<S18>/FixPt Switch' */

      /* Signum: '<S14>/Sign' incorporates:
       *  Constant: '<S12>/has_integrator_constant1'
       */
      u = AutoPIDTuner_P.has_integrator_constant1_Value;
      if (std::isnan(u)) {
        /* Signum: '<S14>/Sign' */
        AutoPIDTuner_B.Sign = (rtNaN);
      } else if (u < 0.0) {
        /* Signum: '<S14>/Sign' */
        AutoPIDTuner_B.Sign = -1.0;
      } else {
        /* Signum: '<S14>/Sign' */
        AutoPIDTuner_B.Sign = (u > 0.0);
      }

      /* End of Signum: '<S14>/Sign' */

      /* Product: '<S14>/Product3' incorporates:
       *  Constant: '<S1>/sine Amp constant'
       */
      AutoPIDTuner_B.Product3 = AutoPIDTuner_P.ClosedLoopPIDAutotuner_AmpSine *
        AutoPIDTuner_B.Sign;

      /* Sum: '<S14>/Sum of Elements' */
      u = -0.0;
      for (int32_T i{0}; i < 5; i++) {
        /* Product: '<S14>/Product' */
        AutoPIDTuner_B.Product_g[i] = AutoPIDTuner_B.GainTs * static_cast<real_T>
          (AutoPIDTuner_B.Output) * AutoPIDTuner_B.w[i];

        /* Trigonometry: '<S14>/Trigonometric Function' */
        AutoPIDTuner_B.TrigonometricFunction[i] = std::sin
          (AutoPIDTuner_B.Product_g[i]);

        /* Product: '<S14>/Product1' */
        AutoPIDTuner_B.Product1_e[i] = AutoPIDTuner_B.TrigonometricFunction[i] *
          AutoPIDTuner_B.Product3;

        /* Trigonometry: '<S14>/Trigonometric Function1' */
        AutoPIDTuner_B.TrigonometricFunction1[i] = std::cos
          (AutoPIDTuner_B.Product_g[i]);

        /* Product: '<S14>/Product2' */
        AutoPIDTuner_B.Product2[i] = AutoPIDTuner_B.TrigonometricFunction1[i] *
          AutoPIDTuner_B.Product3;

        /* Sum: '<S14>/Sum of Elements' */
        u += AutoPIDTuner_B.Product1_e[i];
      }

      /* Sum: '<S14>/Sum of Elements' */
      AutoPIDTuner_B.SumofElements = u;

      /* Switch: '<S14>/Switch2' incorporates:
       *  Constant: '<S12>/has_integrator_constant1'
       */
      if (AutoPIDTuner_P.has_integrator_constant1_Value >
          AutoPIDTuner_P.Switch2_Threshold) {
        /* Switch: '<S14>/Switch2' incorporates:
         *  Constant: '<S14>/zero_constant'
         */
        AutoPIDTuner_B.Switch2 = AutoPIDTuner_P.zero_constant_Value;
      } else {
        /* Switch: '<S14>/Switch2' */
        AutoPIDTuner_B.Switch2 = AutoPIDTuner_P.has_integrator_constant1_Value;
      }

      /* End of Switch: '<S14>/Switch2' */

      /* Sum: '<S14>/Sum' */
      AutoPIDTuner_B.Sum_az = AutoPIDTuner_B.SumofElements +
        AutoPIDTuner_B.Switch2;

      /* Switch: '<S14>/Switch1' incorporates:
       *  Constant: '<S12>/has_integrator_constant1'
       */
      if (AutoPIDTuner_P.has_integrator_constant1_Value >
          AutoPIDTuner_P.Switch1_Threshold) {
        /* Switch: '<S14>/Switch1' incorporates:
         *  Constant: '<S14>/one_constant'
         */
        AutoPIDTuner_B.Switch1 = AutoPIDTuner_P.one_constant_Value;
      } else {
        /* Switch: '<S14>/Switch1' */
        AutoPIDTuner_B.Switch1 = AutoPIDTuner_P.has_integrator_constant1_Value;
      }

      /* End of Switch: '<S14>/Switch1' */

      /* Update for UnitDelay: '<S16>/Output' */
      AutoPIDTuner_DW.Output_DSTATE = AutoPIDTuner_B.FixPtSwitch;
    }

    /* End of Outputs for SubSystem: '<S12>/Perturbation Generator' */

    /* Sum: '<S74>/Sum3' */
    AutoPIDTuner_B.Sum3 = AutoPIDTuner_B.ZeroOrderHold1 + AutoPIDTuner_B.Sum_az;
  }

  /* Switch: '<S74>/Switch' */
  if (AutoPIDTuner_B.ManualSwitch > AutoPIDTuner_P.Switch_Threshold) {
    /* Switch: '<S74>/Switch' */
    AutoPIDTuner_B.Switch = AutoPIDTuner_B.Sum3;
  } else {
    /* Switch: '<S74>/Switch' */
    AutoPIDTuner_B.Switch = AutoPIDTuner_B.Sum_l;
  }

  /* End of Switch: '<S74>/Switch' */

  /* Outport: '<Root>/Velo' */
  AutoPIDTuner_Y.Velo = AutoPIDTuner_B.Switch;
  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    /* ZeroOrderHold: '<S1>/Zero-Order Hold1' incorporates:
     *  Inport: '<Root>/EncoderPos'
     */
    AutoPIDTuner_B.ZeroOrderHold1_h = AutoPIDTuner_U.EncoderPos;

    /* Delay: '<S7>/Enabled Delay Y' */
    if (rt_ZCFcn(RISING_ZERO_CROSSING,
                 &AutoPIDTuner_PrevZCX.EnabledDelayY_Reset_ZCE,
                 (AutoPIDTuner_B.ManualSwitch)) != NO_ZCEVENT) {
      AutoPIDTuner_DW.icLoad = true;
    }

    if (AutoPIDTuner_DW.icLoad) {
      AutoPIDTuner_DW.EnabledDelayY_DSTATE = AutoPIDTuner_B.ZeroOrderHold1_h;
    }

    /* Delay: '<S7>/Enabled Delay Y' */
    AutoPIDTuner_B.y = AutoPIDTuner_DW.EnabledDelayY_DSTATE;

    /* Sum: '<S12>/Sum2' */
    AutoPIDTuner_B.Sum2 = AutoPIDTuner_B.ZeroOrderHold1_h - AutoPIDTuner_B.y;

    /* ZeroOrderHold: '<S74>/Zero-Order Hold2' */
    AutoPIDTuner_B.ZeroOrderHold2 = AutoPIDTuner_B.Switch;

    /* Delay: '<S7>/Enabled Delay U' */
    if (rt_ZCFcn(RISING_ZERO_CROSSING,
                 &AutoPIDTuner_PrevZCX.EnabledDelayU_Reset_ZCE,
                 (AutoPIDTuner_B.ManualSwitch)) != NO_ZCEVENT) {
      AutoPIDTuner_DW.icLoad_m = true;
    }

    if (AutoPIDTuner_DW.icLoad_m) {
      AutoPIDTuner_DW.EnabledDelayU_DSTATE = AutoPIDTuner_B.ZeroOrderHold2;
    }

    /* Delay: '<S7>/Enabled Delay U' */
    AutoPIDTuner_B.u = AutoPIDTuner_DW.EnabledDelayU_DSTATE;

    /* Sum: '<S12>/Sum3' */
    AutoPIDTuner_B.Sum3_c = AutoPIDTuner_B.ZeroOrderHold2 - AutoPIDTuner_B.u;

    /* Product: '<S13>/Product' incorporates:
     *  Constant: '<S1>/bandwidth constant'
     */
    AutoPIDTuner_B.Product = AutoPIDTuner_B.GainTs *
      AutoPIDTuner_P.ClosedLoopPIDAutotuner_Bandwidt;

    /* Product: '<S13>/Product1' incorporates:
     *  Constant: '<S13>/Constant1'
     *  Constant: '<S13>/Constant2'
     */
    AutoPIDTuner_B.Product1 = AutoPIDTuner_P.Constant1_Value_p *
      AutoPIDTuner_P.Constant2_Value;

    /* Product: '<S13>/Divide' */
    AutoPIDTuner_B.Divide = AutoPIDTuner_B.Product / AutoPIDTuner_B.Product1;

    /* Sum: '<S13>/Sum' incorporates:
     *  Constant: '<S13>/Constant'
     */
    AutoPIDTuner_B.Sum_a = AutoPIDTuner_P.Constant_Value_k -
      AutoPIDTuner_B.Divide;

    /* Outputs for Enabled SubSystem: '<S12>/Response Estimation' incorporates:
     *  EnablePort: '<S15>/Enable'
     */
    if (rtsiIsModeUpdateTimeStep(&(&AutoPIDTuner_M)->solverInfo)) {
      if (AutoPIDTuner_B.ManualSwitch > 0.0) {
        if (!AutoPIDTuner_DW.ResponseEstimation_MODE) {
          /* InitializeConditions for Delay: '<S20>/delayBuffery' */
          AutoPIDTuner_DW.icLoad_n = true;

          /* InitializeConditions for Delay: '<S20>/delayBufferH' */
          AutoPIDTuner_DW.icLoad_h = true;

          /* InitializeConditions for Delay: '<S20>/delayTheta' */
          AutoPIDTuner_DW.icLoad_a = true;

          /* InitializeConditions for Delay: '<S20>/delayL' */
          AutoPIDTuner_DW.icLoad_i = true;

          /* InitializeConditions for Delay: '<S63>/Delay' */
          AutoPIDTuner_DW.Delay_DSTATE = AutoPIDTuner_P.Delay_InitialCondition;

          /* InitializeConditions for Delay: '<S19>/delayBuffery' */
          AutoPIDTuner_DW.icLoad_g = true;

          /* InitializeConditions for Delay: '<S19>/delayBufferH' */
          AutoPIDTuner_DW.icLoad_i5 = true;

          /* InitializeConditions for Delay: '<S19>/delayTheta' */
          AutoPIDTuner_DW.icLoad_o = true;

          /* InitializeConditions for Delay: '<S19>/delayL' */
          AutoPIDTuner_DW.icLoad_id = true;

          /* InitializeConditions for Delay: '<S38>/Delay' */
          AutoPIDTuner_DW.Delay_DSTATE_i =
            AutoPIDTuner_P.Delay_InitialCondition_j;

          /* SystemReset for MATLAB Function: '<S20>/RLS' */
          AutoPIDTuner_RLS_Reset(&AutoPIDTuner_DW.sf_RLS_h);

          /* SystemReset for MATLAB Function: '<S19>/RLS' */
          AutoPIDTuner_RLS_Reset(&AutoPIDTuner_DW.sf_RLS);
          AutoPIDTuner_DW.ResponseEstimation_MODE = true;
        }
      } else {
        AutoPIDTuner_DW.ResponseEstimation_MODE = false;
      }
    }

    if (AutoPIDTuner_DW.ResponseEstimation_MODE) {
      /* Delay: '<S20>/delayBuffery' incorporates:
       *  Constant: '<S20>/InitialOutputs'
       */
      if (AutoPIDTuner_DW.icLoad_n) {
        AutoPIDTuner_DW.delayBuffery_DSTATE =
          AutoPIDTuner_P.InitialOutputs_Value;
      }

      /* Delay: '<S20>/delayBuffery' */
      AutoPIDTuner_B.delayBuffery = AutoPIDTuner_DW.delayBuffery_DSTATE;

      /* Delay: '<S20>/delayBufferH' incorporates:
       *  Constant: '<S20>/InitialRegressors'
       */
      if (AutoPIDTuner_DW.icLoad_h) {
        AutoPIDTuner_DW.delayBufferH_DSTATE =
          AutoPIDTuner_P.InitialRegressors_Value;
      }

      /* Delay: '<S20>/delayBufferH' */
      AutoPIDTuner_B.delayBufferH = AutoPIDTuner_DW.delayBufferH_DSTATE;
      for (int32_T i{0}; i < 11; i++) {
        /* Delay: '<S20>/delayTheta' incorporates:
         *  Constant: '<S20>/InitialParameters'
         */
        if (AutoPIDTuner_DW.icLoad_a) {
          AutoPIDTuner_DW.delayTheta_DSTATE[i] =
            AutoPIDTuner_P.InitialParameters_Value[i];
        }

        /* Delay: '<S20>/delayTheta' */
        AutoPIDTuner_B.delayTheta[i] = AutoPIDTuner_DW.delayTheta_DSTATE[i];
      }

      for (int32_T i{0}; i < 121; i++) {
        /* Delay: '<S20>/delayL' incorporates:
         *  Constant: '<S20>/InitialCovariance'
         */
        if (AutoPIDTuner_DW.icLoad_i) {
          AutoPIDTuner_DW.delayL_DSTATE[i] =
            AutoPIDTuner_P.InitialCovariance_Value[i];
        }

        /* Delay: '<S20>/delayL' */
        AutoPIDTuner_B.delayL[i] = AutoPIDTuner_DW.delayL_DSTATE[i];
      }

      /* Delay: '<S63>/Delay' */
      AutoPIDTuner_B.Delay = AutoPIDTuner_DW.Delay_DSTATE;

      /* MATLAB Function: '<S20>/RLS' incorporates:
       *  Constant: '<S20>/Enable'
       */
      AutoPIDTuner_RLS(AutoPIDTuner_B.Product2, AutoPIDTuner_B.Product1_e,
                       AutoPIDTuner_B.Switch1, AutoPIDTuner_B.Sum2,
                       AutoPIDTuner_P.Enable_Value, AutoPIDTuner_B.Sum_a,
                       AutoPIDTuner_B.delayBuffery, AutoPIDTuner_B.delayBufferH,
                       AutoPIDTuner_B.delayTheta, AutoPIDTuner_B.delayL,
                       &AutoPIDTuner_B.sf_RLS_h, &AutoPIDTuner_DW.sf_RLS_h);
      for (int32_T i{0}; i < 5; i++) {
        /* RealImagToComplex: '<S15>/Real-Imag to Complex' */
        AutoPIDTuner_B.RealImagtoComplex[i].re = AutoPIDTuner_B.sf_RLS_h.x[i + 5];
        AutoPIDTuner_B.RealImagtoComplex[i].im = AutoPIDTuner_B.sf_RLS_h.x[i];
      }

      /* Delay: '<S19>/delayBuffery' incorporates:
       *  Constant: '<S19>/InitialOutputs'
       */
      if (AutoPIDTuner_DW.icLoad_g) {
        AutoPIDTuner_DW.delayBuffery_DSTATE_m =
          AutoPIDTuner_P.InitialOutputs_Value_d;
      }

      /* Delay: '<S19>/delayBuffery' */
      AutoPIDTuner_B.delayBuffery_b = AutoPIDTuner_DW.delayBuffery_DSTATE_m;

      /* Delay: '<S19>/delayBufferH' incorporates:
       *  Constant: '<S19>/InitialRegressors'
       */
      if (AutoPIDTuner_DW.icLoad_i5) {
        AutoPIDTuner_DW.delayBufferH_DSTATE_k =
          AutoPIDTuner_P.InitialRegressors_Value_g;
      }

      /* Delay: '<S19>/delayBufferH' */
      AutoPIDTuner_B.delayBufferH_h = AutoPIDTuner_DW.delayBufferH_DSTATE_k;
      for (int32_T i{0}; i < 11; i++) {
        /* Delay: '<S19>/delayTheta' incorporates:
         *  Constant: '<S19>/InitialParameters'
         */
        if (AutoPIDTuner_DW.icLoad_o) {
          AutoPIDTuner_DW.delayTheta_DSTATE_a[i] =
            AutoPIDTuner_P.InitialParameters_Value_f[i];
        }

        /* Delay: '<S19>/delayTheta' */
        AutoPIDTuner_B.delayTheta_p[i] = AutoPIDTuner_DW.delayTheta_DSTATE_a[i];
      }

      for (int32_T i{0}; i < 121; i++) {
        /* Delay: '<S19>/delayL' incorporates:
         *  Constant: '<S19>/InitialCovariance'
         */
        if (AutoPIDTuner_DW.icLoad_id) {
          AutoPIDTuner_DW.delayL_DSTATE_m[i] =
            AutoPIDTuner_P.InitialCovariance_Value_l[i];
        }

        /* Delay: '<S19>/delayL' */
        AutoPIDTuner_B.delayL_e[i] = AutoPIDTuner_DW.delayL_DSTATE_m[i];
      }

      /* Delay: '<S38>/Delay' */
      AutoPIDTuner_B.Delay_n = AutoPIDTuner_DW.Delay_DSTATE_i;

      /* MATLAB Function: '<S19>/RLS' incorporates:
       *  Constant: '<S19>/Enable'
       */
      AutoPIDTuner_RLS(AutoPIDTuner_B.Product2, AutoPIDTuner_B.Product1_e,
                       AutoPIDTuner_B.Switch1, AutoPIDTuner_B.Sum3_c,
                       AutoPIDTuner_P.Enable_Value_j, AutoPIDTuner_B.Sum_a,
                       AutoPIDTuner_B.delayBuffery_b,
                       AutoPIDTuner_B.delayBufferH_h,
                       AutoPIDTuner_B.delayTheta_p, AutoPIDTuner_B.delayL_e,
                       &AutoPIDTuner_B.sf_RLS, &AutoPIDTuner_DW.sf_RLS);
      for (int32_T i{0}; i < 5; i++) {
        /* RealImagToComplex: '<S15>/Real-Imag to Complex1' */
        AutoPIDTuner_B.RealImagtoComplex1[i].re = AutoPIDTuner_B.sf_RLS.x[i + 5];
        AutoPIDTuner_B.RealImagtoComplex1[i].im = AutoPIDTuner_B.sf_RLS.x[i];
      }

      for (int32_T i{0}; i < 5; i++) {
        real_T br;

        /* Product: '<S15>/Divide1' incorporates:
         *  RealImagToComplex: '<S15>/Real-Imag to Complex'
         *  RealImagToComplex: '<S15>/Real-Imag to Complex1'
         */
        u = AutoPIDTuner_B.RealImagtoComplex[i].re;
        ai = AutoPIDTuner_B.RealImagtoComplex[i].im;
        br = AutoPIDTuner_B.RealImagtoComplex1[i].re;
        bi = AutoPIDTuner_B.RealImagtoComplex1[i].im;
        if (bi == 0.0) {
          if (ai == 0.0) {
            bim = u / br;
            u = 0.0;
          } else if (u == 0.0) {
            bim = 0.0;
            u = ai / br;
          } else {
            bim = u / br;
            u = ai / br;
          }
        } else if (br == 0.0) {
          if (u == 0.0) {
            bim = ai / bi;
            u = 0.0;
          } else if (ai == 0.0) {
            bim = 0.0;
            u = -(u / bi);
          } else {
            bim = ai / bi;
            u = -(u / bi);
          }
        } else {
          brm = std::abs(br);
          bim = std::abs(bi);
          if (brm > bim) {
            brm = bi / br;
            br += brm * bi;
            bi = brm * ai + u;
            u = ai - brm * u;
            bim = bi / br;
            u /= br;
          } else if (bim == brm) {
            bim = br > 0.0 ? 0.5 : -0.5;
            br = bi > 0.0 ? 0.5 : -0.5;
            bi = u * bim + ai * br;
            u = ai * bim - u * br;
            bim = bi / brm;
            u /= brm;
          } else {
            brm = br / bi;
            br = brm * br + bi;
            bi = brm * u + ai;
            u = brm * ai - u;
            bim = bi / br;
            u /= br;
          }
        }

        AutoPIDTuner_B.Divide1[i].re = bim;
        AutoPIDTuner_B.Divide1[i].im = u;

        /* End of Product: '<S15>/Divide1' */

        /* Gain: '<S15>/Gain' incorporates:
         *  Product: '<S15>/Divide1'
         */
        AutoPIDTuner_B.Gain[i].re = AutoPIDTuner_P.Gain_Gain[i] *
          AutoPIDTuner_B.Divide1[i].re;
        AutoPIDTuner_B.Gain[i].im = AutoPIDTuner_P.Gain_Gain[i] *
          AutoPIDTuner_B.Divide1[i].im;
      }

      /* Update for Delay: '<S20>/delayBuffery' */
      AutoPIDTuner_DW.icLoad_n = false;
      AutoPIDTuner_DW.delayBuffery_DSTATE = AutoPIDTuner_B.sf_RLS_h.yBuffer;

      /* Update for Delay: '<S20>/delayBufferH' */
      AutoPIDTuner_DW.icLoad_h = false;
      AutoPIDTuner_DW.delayBufferH_DSTATE = AutoPIDTuner_B.sf_RLS_h.HBuffer;

      /* Update for Delay: '<S20>/delayTheta' */
      AutoPIDTuner_DW.icLoad_a = false;
      std::memcpy(&AutoPIDTuner_DW.delayTheta_DSTATE[0],
                  &AutoPIDTuner_B.sf_RLS_h.x[0], 11U * sizeof(real_T));

      /* Update for Delay: '<S20>/delayL' */
      AutoPIDTuner_DW.icLoad_i = false;
      std::memcpy(&AutoPIDTuner_DW.delayL_DSTATE[0], &AutoPIDTuner_B.sf_RLS_h.L
                  [0], 121U * sizeof(real_T));

      /* Update for Delay: '<S63>/Delay' incorporates:
       *  Constant: '<S63>/Constant'
       */
      AutoPIDTuner_DW.Delay_DSTATE = AutoPIDTuner_P.Constant_Value_b;

      /* Update for Delay: '<S19>/delayBuffery' */
      AutoPIDTuner_DW.icLoad_g = false;
      AutoPIDTuner_DW.delayBuffery_DSTATE_m = AutoPIDTuner_B.sf_RLS.yBuffer;

      /* Update for Delay: '<S19>/delayBufferH' */
      AutoPIDTuner_DW.icLoad_i5 = false;
      AutoPIDTuner_DW.delayBufferH_DSTATE_k = AutoPIDTuner_B.sf_RLS.HBuffer;

      /* Update for Delay: '<S19>/delayTheta' */
      AutoPIDTuner_DW.icLoad_o = false;
      std::memcpy(&AutoPIDTuner_DW.delayTheta_DSTATE_a[0],
                  &AutoPIDTuner_B.sf_RLS.x[0], 11U * sizeof(real_T));

      /* Update for Delay: '<S19>/delayL' */
      AutoPIDTuner_DW.icLoad_id = false;
      std::memcpy(&AutoPIDTuner_DW.delayL_DSTATE_m[0], &AutoPIDTuner_B.sf_RLS.L
                  [0], 121U * sizeof(real_T));

      /* Update for Delay: '<S38>/Delay' incorporates:
       *  Constant: '<S38>/Constant'
       */
      AutoPIDTuner_DW.Delay_DSTATE_i = AutoPIDTuner_P.Constant_Value_j;
    }

    /* End of Outputs for SubSystem: '<S12>/Response Estimation' */

    /* Switch: '<S1>/Plant Type Switch' incorporates:
     *  Constant: '<S1>/plant type constant'
     */
    if (AutoPIDTuner_P.ClosedLoopPIDAutotuner_PlantTyp >
        AutoPIDTuner_P.PlantTypeSwitch_Threshold) {
      /* Switch: '<S1>/Plant Type Switch' incorporates:
       *  Constant: '<S1>/Integrating'
       */
      AutoPIDTuner_B.PlantTypeSwitch = AutoPIDTuner_P.Integrating_Value;
    } else {
      /* Switch: '<S1>/Plant Type Switch' incorporates:
       *  Constant: '<S1>/Stable'
       */
      AutoPIDTuner_B.PlantTypeSwitch = AutoPIDTuner_P.Stable_Value;
    }

    /* End of Switch: '<S1>/Plant Type Switch' */

    /* Switch: '<S1>/Plant Sign Switch' incorporates:
     *  Constant: '<S1>/plant sign constant'
     */
    if (AutoPIDTuner_P.ClosedLoopPIDAutotuner_PlantSig >
        AutoPIDTuner_P.PlantSignSwitch_Threshold) {
      /* Switch: '<S1>/Plant Sign Switch' incorporates:
       *  Constant: '<S1>/Negative'
       */
      AutoPIDTuner_B.PlantSignSwitch = AutoPIDTuner_P.Negative_Value;
    } else {
      /* Switch: '<S1>/Plant Sign Switch' incorporates:
       *  Constant: '<S1>/Positive'
       */
      AutoPIDTuner_B.PlantSignSwitch = AutoPIDTuner_P.Positive_Value;
    }

    /* End of Switch: '<S1>/Plant Sign Switch' */

    /* Gain: '<S1>/GainPM' incorporates:
     *  Constant: '<S1>/target PM constant'
     */
    AutoPIDTuner_B.GainPM = AutoPIDTuner_P.GainPM_Gain *
      AutoPIDTuner_P.ClosedLoopPIDAutotuner_TargetPM;

    /* Outputs for Triggered SubSystem: '<S1>/Gains From Online Tuning Module' incorporates:
     *  TriggerPort: '<S6>/Trigger'
     */
    if (rtsiIsModeUpdateTimeStep(&(&AutoPIDTuner_M)->solverInfo)) {
      ZCEventType zcEvent;
      zcEvent = rt_ZCFcn(FALLING_ZERO_CROSSING,
                         &AutoPIDTuner_PrevZCX.GainsFromOnlineTuningModule_Tri,
                         (AutoPIDTuner_B.ManualSwitch));
      if (zcEvent != NO_ZCEVENT) {
        /* SignalConversion generated from: '<S72>/ SFunction ' incorporates:
         *  Constant: '<S1>/PID form constant'
         *  Constant: '<S1>/filter formula constant'
         *  Constant: '<S1>/integrator formula constant'
         *  MATLAB Function: '<S71>/DeployedMode'
         */
        AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[0] =
          AutoPIDTuner_P.ClosedLoopPIDAutotuner_Integrat;
        AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[1] =
          AutoPIDTuner_P.filterformulaconstant_Value;
        AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[2] =
          AutoPIDTuner_P.ClosedLoopPIDAutotuner_PIDForm;

        /* MATLAB Function: '<S71>/DeployedMode' incorporates:
         *  Constant: '<S1>/PID type constant'
         *  Gain: '<S15>/Gain'
         */
        /* :  IntegratorFormula = tuningParams(1); */
        /* :  FilterFormula = tuningParams(2); */
        /* :  PIDForm = tuningParams(3); */
        /* :  [P, I, D, N, achievedPM] = slpidwrapperMLFunc5w(w5, hG5, HasIntegrator, LoopSign, targetPM, Ts, PIDType, PIDForm, IntegratorFormula, FilterFormula, TimeDomain); */
        if (AutoPIDTuner_P.DeployedMode_TimeDomain == 1) {
          std::memcpy(&tmp_0[0], &AutoPIDTuner_B.Gain[0], 5U * sizeof(creal_T));
          AutoPIDTuner_slpidfivepoint
            (AutoPIDTuner_P.ClosedLoopPIDAutotuner_PIDType,
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[2], AutoPIDTuner_B.w,
             tmp_0, AutoPIDTuner_B.GainPM, AutoPIDTuner_B.PlantTypeSwitch,
             AutoPIDTuner_B.PlantSignSwitch, AutoPIDTuner_B.GainTs,
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[0],
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[1], &u, &ai, &brm,
             &bi, &bim);
        } else {
          std::memcpy(&tmp_0[0], &AutoPIDTuner_B.Gain[0], 5U * sizeof(creal_T));
          AutoPIDTuner_slpidfivepoint
            (AutoPIDTuner_P.ClosedLoopPIDAutotuner_PIDType,
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[2], AutoPIDTuner_B.w,
             tmp_0, AutoPIDTuner_B.GainPM, AutoPIDTuner_B.PlantTypeSwitch,
             AutoPIDTuner_B.PlantSignSwitch, 0.0,
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[0],
             AutoPIDTuner_B.TmpSignalConversionAtSFunctionI[1], &u, &ai, &brm,
             &bi, &bim);
        }

        AutoPIDTuner_B.P = u;
        AutoPIDTuner_B.I = ai;
        AutoPIDTuner_B.D = brm;
        AutoPIDTuner_B.N = bi;
        AutoPIDTuner_B.achievedPM = bim;
      }
    }

    /* End of Outputs for SubSystem: '<S1>/Gains From Online Tuning Module' */
  }

  /* Gain: '<S103>/Integral Gain' */
  AutoPIDTuner_B.IntegralGain = AutoPIDTuner_P.PIDController_I *
    AutoPIDTuner_B.Sum;
  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    /* Matfile logging */
    rt_UpdateTXYLogVars((&AutoPIDTuner_M)->rtwLogInfo, ((&AutoPIDTuner_M)
      ->Timing.t));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
      /* Update for Delay: '<S7>/Enabled Delay Y' */
      AutoPIDTuner_DW.icLoad = false;
      AutoPIDTuner_DW.EnabledDelayY_DSTATE = AutoPIDTuner_B.y;

      /* Update for Delay: '<S7>/Enabled Delay U' */
      AutoPIDTuner_DW.icLoad_m = false;
      AutoPIDTuner_DW.EnabledDelayU_DSTATE = AutoPIDTuner_B.u;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&AutoPIDTuner_M))) {
    /* signal main to stop simulation */
    {                                  /* Sample time: [0.0s, 0.0s] */
      if ((rtmGetTFinal((&AutoPIDTuner_M))!=-1) &&
          !((rtmGetTFinal((&AutoPIDTuner_M))-((((&AutoPIDTuner_M)
               ->Timing.clockTick1+(&AutoPIDTuner_M)->Timing.clockTickH1*
               4294967296.0)) * 0.1)) > ((((&AutoPIDTuner_M)->Timing.clockTick1+
              (&AutoPIDTuner_M)->Timing.clockTickH1* 4294967296.0)) * 0.1) *
            (DBL_EPSILON))) {
        rtmSetErrorStatus((&AutoPIDTuner_M), "Simulation finished");
      }
    }

    rt_ertODEUpdateContinuousStates(&(&AutoPIDTuner_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&AutoPIDTuner_M)->Timing.clockTick0)) {
      ++(&AutoPIDTuner_M)->Timing.clockTickH0;
    }

    (&AutoPIDTuner_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&AutoPIDTuner_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.1s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.1, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&AutoPIDTuner_M)->Timing.clockTick1++;
      if (!(&AutoPIDTuner_M)->Timing.clockTick1) {
        (&AutoPIDTuner_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void AutoPIDTuner::AutoPIDTuner_derivatives()
{
  XDot_AutoPIDTuner_T *_rtXdot;
  _rtXdot = ((XDot_AutoPIDTuner_T *) (&AutoPIDTuner_M)->derivs);

  /* Derivatives for Integrator: '<S106>/Integrator' */
  _rtXdot->Integrator_CSTATE = AutoPIDTuner_B.IntegralGain;

  /* Derivatives for Integrator: '<S101>/Filter' */
  _rtXdot->Filter_CSTATE = AutoPIDTuner_B.FilterCoefficient;
}

/* Model initialize function */
void AutoPIDTuner::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&AutoPIDTuner_M)->solverInfo, &(&AutoPIDTuner_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&AutoPIDTuner_M)->solverInfo, &rtmGetTPtr((&AutoPIDTuner_M)));
    rtsiSetStepSizePtr(&(&AutoPIDTuner_M)->solverInfo, &(&AutoPIDTuner_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&AutoPIDTuner_M)->solverInfo, &(&AutoPIDTuner_M)->derivs);
    rtsiSetContStatesPtr(&(&AutoPIDTuner_M)->solverInfo, (real_T **)
                         &(&AutoPIDTuner_M)->contStates);
    rtsiSetNumContStatesPtr(&(&AutoPIDTuner_M)->solverInfo, &(&AutoPIDTuner_M)
      ->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&AutoPIDTuner_M)->solverInfo,
      &(&AutoPIDTuner_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&AutoPIDTuner_M)->solverInfo,
      &(&AutoPIDTuner_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&AutoPIDTuner_M)->solverInfo,
      &(&AutoPIDTuner_M)->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&(&AutoPIDTuner_M)->solverInfo, (&rtmGetErrorStatus
      ((&AutoPIDTuner_M))));
    rtsiSetRTModelPtr(&(&AutoPIDTuner_M)->solverInfo, (&AutoPIDTuner_M));
  }

  rtsiSetSimTimeStep(&(&AutoPIDTuner_M)->solverInfo, MAJOR_TIME_STEP);
  (&AutoPIDTuner_M)->intgData.y = (&AutoPIDTuner_M)->odeY;
  (&AutoPIDTuner_M)->intgData.f[0] = (&AutoPIDTuner_M)->odeF[0];
  (&AutoPIDTuner_M)->intgData.f[1] = (&AutoPIDTuner_M)->odeF[1];
  (&AutoPIDTuner_M)->intgData.f[2] = (&AutoPIDTuner_M)->odeF[2];
  (&AutoPIDTuner_M)->contStates = ((X_AutoPIDTuner_T *) &AutoPIDTuner_X);
  rtsiSetSolverData(&(&AutoPIDTuner_M)->solverInfo, static_cast<void *>
                    (&(&AutoPIDTuner_M)->intgData));
  rtsiSetIsMinorTimeStepWithModeChange(&(&AutoPIDTuner_M)->solverInfo, false);
  rtsiSetSolverName(&(&AutoPIDTuner_M)->solverInfo,"ode3");
  rtmSetTPtr((&AutoPIDTuner_M), &(&AutoPIDTuner_M)->Timing.tArray[0]);
  rtmSetTFinal((&AutoPIDTuner_M), 10.0);
  (&AutoPIDTuner_M)->Timing.stepSize0 = 0.1;

  /* Setup for data logging */
  {
    static RTWLogInfo rt_DataLoggingInfo;
    rt_DataLoggingInfo.loggingInterval = (nullptr);
    (&AutoPIDTuner_M)->rtwLogInfo = &rt_DataLoggingInfo;
  }

  /* Setup for data logging */
  {
    rtliSetLogXSignalInfo((&AutoPIDTuner_M)->rtwLogInfo, (nullptr));
    rtliSetLogXSignalPtrs((&AutoPIDTuner_M)->rtwLogInfo, (nullptr));
    rtliSetLogT((&AutoPIDTuner_M)->rtwLogInfo, "tout");
    rtliSetLogX((&AutoPIDTuner_M)->rtwLogInfo, "");
    rtliSetLogXFinal((&AutoPIDTuner_M)->rtwLogInfo, "");
    rtliSetLogVarNameModifier((&AutoPIDTuner_M)->rtwLogInfo, "rt_");
    rtliSetLogFormat((&AutoPIDTuner_M)->rtwLogInfo, 4);
    rtliSetLogMaxRows((&AutoPIDTuner_M)->rtwLogInfo, 0);
    rtliSetLogDecimation((&AutoPIDTuner_M)->rtwLogInfo, 1);
    rtliSetLogY((&AutoPIDTuner_M)->rtwLogInfo, "");
    rtliSetLogYSignalInfo((&AutoPIDTuner_M)->rtwLogInfo, (nullptr));
    rtliSetLogYSignalPtrs((&AutoPIDTuner_M)->rtwLogInfo, (nullptr));
  }

  /* Matfile logging */
  rt_StartDataLoggingWithStartTime((&AutoPIDTuner_M)->rtwLogInfo, 0.0,
    rtmGetTFinal((&AutoPIDTuner_M)), (&AutoPIDTuner_M)->Timing.stepSize0,
    (&rtmGetErrorStatus((&AutoPIDTuner_M))));
  AutoPIDTuner_PrevZCX.EnabledDelayY_Reset_ZCE = UNINITIALIZED_ZCSIG;
  AutoPIDTuner_PrevZCX.EnabledDelayU_Reset_ZCE = UNINITIALIZED_ZCSIG;
  AutoPIDTuner_PrevZCX.GainsFromOnlineTuningModule_Tri = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Integrator: '<S106>/Integrator' */
  AutoPIDTuner_X.Integrator_CSTATE =
    AutoPIDTuner_P.PIDController_InitialConditio_k;

  /* InitializeConditions for Integrator: '<S101>/Filter' */
  AutoPIDTuner_X.Filter_CSTATE = AutoPIDTuner_P.PIDController_InitialConditionF;

  /* InitializeConditions for Delay: '<S7>/Enabled Delay Y' */
  AutoPIDTuner_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S7>/Enabled Delay U' */
  AutoPIDTuner_DW.icLoad_m = true;

  /* SystemInitialize for Enabled SubSystem: '<S12>/Perturbation Generator' */
  /* InitializeConditions for UnitDelay: '<S16>/Output' */
  AutoPIDTuner_DW.Output_DSTATE = AutoPIDTuner_P.Output_InitialCondition;

  /* SystemInitialize for Sum: '<S14>/Sum' incorporates:
   *  Outport: '<S14>/signal'
   */
  AutoPIDTuner_B.Sum_az = AutoPIDTuner_P.signal_Y0;

  /* SystemInitialize for Switch: '<S14>/Switch1' incorporates:
   *  Outport: '<S14>/regressors'
   */
  AutoPIDTuner_B.Switch1 = AutoPIDTuner_P.regressors_Y0;

  /* End of SystemInitialize for SubSystem: '<S12>/Perturbation Generator' */

  /* SystemInitialize for Enabled SubSystem: '<S12>/Response Estimation' */
  /* InitializeConditions for Delay: '<S20>/delayBuffery' */
  AutoPIDTuner_DW.icLoad_n = true;

  /* InitializeConditions for Delay: '<S20>/delayBufferH' */
  AutoPIDTuner_DW.icLoad_h = true;

  /* InitializeConditions for Delay: '<S20>/delayTheta' */
  AutoPIDTuner_DW.icLoad_a = true;

  /* InitializeConditions for Delay: '<S20>/delayL' */
  AutoPIDTuner_DW.icLoad_i = true;

  /* InitializeConditions for Delay: '<S63>/Delay' */
  AutoPIDTuner_DW.Delay_DSTATE = AutoPIDTuner_P.Delay_InitialCondition;

  /* InitializeConditions for Delay: '<S19>/delayBuffery' */
  AutoPIDTuner_DW.icLoad_g = true;

  /* InitializeConditions for Delay: '<S19>/delayBufferH' */
  AutoPIDTuner_DW.icLoad_i5 = true;

  /* InitializeConditions for Delay: '<S19>/delayTheta' */
  AutoPIDTuner_DW.icLoad_o = true;

  /* InitializeConditions for Delay: '<S19>/delayL' */
  AutoPIDTuner_DW.icLoad_id = true;

  /* InitializeConditions for Delay: '<S38>/Delay' */
  AutoPIDTuner_DW.Delay_DSTATE_i = AutoPIDTuner_P.Delay_InitialCondition_j;

  /* SystemInitialize for Enabled SubSystem: '<S12>/Perturbation Generator' */
  for (int32_T i{0}; i < 5; i++) {
    /* SystemInitialize for Product: '<S14>/Product2' incorporates:
     *  Outport: '<S14>/regressors'
     */
    AutoPIDTuner_B.Product2[i] = AutoPIDTuner_P.regressors_Y0;

    /* SystemInitialize for Product: '<S14>/Product1' incorporates:
     *  Outport: '<S14>/regressors'
     */
    AutoPIDTuner_B.Product1_e[i] = AutoPIDTuner_P.regressors_Y0;

    /* SystemInitialize for Gain: '<S15>/Gain' incorporates:
     *  Outport: '<S15>/FreqResp'
     */
    AutoPIDTuner_B.Gain[i].re = AutoPIDTuner_P.FreqResp_Y0;
    AutoPIDTuner_B.Gain[i].im = 0.0;
  }

  /* End of SystemInitialize for SubSystem: '<S12>/Perturbation Generator' */
  /* End of SystemInitialize for SubSystem: '<S12>/Response Estimation' */

  /* SystemInitialize for Triggered SubSystem: '<S1>/Gains From Online Tuning Module' */
  /* SystemInitialize for Outport: '<S6>/P' */
  AutoPIDTuner_B.P = AutoPIDTuner_P.P_Y0;

  /* SystemInitialize for Outport: '<S6>/I' */
  AutoPIDTuner_B.I = AutoPIDTuner_P.I_Y0;

  /* SystemInitialize for Outport: '<S6>/D' */
  AutoPIDTuner_B.D = AutoPIDTuner_P.D_Y0;

  /* SystemInitialize for Outport: '<S6>/N' */
  AutoPIDTuner_B.N = AutoPIDTuner_P.N_Y0;

  /* SystemInitialize for Outport: '<S6>/achievedPM' */
  AutoPIDTuner_B.achievedPM = AutoPIDTuner_P.achievedPM_Y0;

  /* End of SystemInitialize for SubSystem: '<S1>/Gains From Online Tuning Module' */
}

/* Model terminate function */
void AutoPIDTuner::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
AutoPIDTuner::AutoPIDTuner() :
  AutoPIDTuner_U(),
  AutoPIDTuner_Y(),
  AutoPIDTuner_B(),
  AutoPIDTuner_DW(),
  AutoPIDTuner_X(),
  AutoPIDTuner_PrevZCX(),
  AutoPIDTuner_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
AutoPIDTuner::~AutoPIDTuner()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_AutoPIDTuner_T * AutoPIDTuner::getRTM()
{
  return (&AutoPIDTuner_M);
}
