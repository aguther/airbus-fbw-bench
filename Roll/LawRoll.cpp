#include "LawRoll.h"
#include <cmath>

LawRoll::LawRoll() {
}

void LawRoll::setErrorFactor(
    double factor
) {
  directWeightFactor = 1.0 - factor;
}

void LawRoll::setPidParameters(
    double rollDemandKp,
    double rollDemandKi,
    double rollDemandKd,
    double bankDemandKp,
    double bankDemandKi,
    double bankDemandKd
) {
  k_xi_phi = bankDemandKp;
  j_xi_phi = bankDemandKi;
  h_xi_phi = bankDemandKd;
  k_xi_p = rollDemandKp;
}

LawRoll::Output LawRoll::dataUpdated(
    LawRoll::Input input
) {
  // store input
  inputCurrent = input;

  // calculate load demand depending on sidestick position
  outputCurrent.rollRateDemand = 15 * inputCurrent.stickDeflection;

  // bank angle limiter
  outputCurrent.rollRateDemandLimiter = (abs(inputCurrent.bank) - 33.0) * (15.0 / 33.0);
  outputCurrent.rollRateDemandLimiter = fmax(0, outputCurrent.rollRateDemandLimiter);
  outputCurrent.rollRateDemandLimiter = copysign(outputCurrent.rollRateDemandLimiter, inputCurrent.bank);
  outputCurrent.rollRateDemandLimiter *= -1;
  outputCurrent.rollRateDemand += outputCurrent.rollRateDemandLimiter;

  // integrator for bank demand
  outputCurrent.bankDemand = limit(
      outputCurrent.bankDemand + (outputCurrent.rollRateDemand * SAMPLE_TIME),
      -66.0,
      +66.0
  );

  double phi = inputCurrent.bank;
  double p_k = inputCurrent.rollRateRadPerSecond;

  double phi_c = outputCurrent.bankDemand;
  double phi_d = phi - phi_c;

  phi_d_integral += phi_d;
  phi_d_integral = limit(phi_d_integral, -1.0, 1.0);

  // calculate output
  double xi = (phi_c * h_xi_phi) + (phi_d * k_xi_phi) + (phi_d * j_xi_phi) + (p_k * k_xi_p);

  outputCurrent.aileronPosition = (1.0 - directWeightFactor) * xi;

  // add weighted direct control
  outputCurrent.aileronPosition += directWeightFactor * inputCurrent.stickDeflection;

  // limit output
  outputCurrent.aileronPosition = limit(outputCurrent.aileronPosition, -1.0, 1.0);

  // store input and output for next iteration
  inputLast = inputCurrent;
  outputLast = outputCurrent;

  outputCurrent.bankDemand = phi_c;

  return outputCurrent;
}

double LawRoll::limit(
    double value,
    double min,
    double max
) {
  if (value > max) {
    return max;
  } else if (value < min) {
    return min;
  }
  return value;
}
