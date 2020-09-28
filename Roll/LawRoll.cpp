#include "LawRoll.h"
#include <cmath>

LawRoll::LawRoll()
    : pidControllerRollRate(SAMPLE_TIME, 1, -1, 10.00, 0.20, 4.00, 0.0),
      pidControllerBank(SAMPLE_TIME, 15, -15, 1.2, 0.0, 0.4, 0.0) {
}

void LawRoll::setErrorFactor(
    double factor
) {
  pidControllerRollRate.setErrorWeightFactor(factor);
  pidControllerBank.setErrorWeightFactor(factor);
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
  pidControllerRollRate.setKp(rollDemandKp);
  pidControllerRollRate.setKi(rollDemandKi);
  pidControllerRollRate.setKd(rollDemandKd);
  pidControllerBank.setKp(bankDemandKp);
  pidControllerBank.setKi(bankDemandKi);
  pidControllerBank.setKd(bankDemandKd);
}

LawRoll::Output LawRoll::dataUpdated(
    LawRoll::Input input
) {
  // store input
  inputCurrent = input;

//  // calculate load demand depending on sidestick position
//  outputCurrent.rollRateDemand = 15 * inputCurrent.stickDeflection;
//
//  // integrator for bank demand
//  outputCurrent.bankDemand = limit(
//      outputCurrent.bankDemand + (outputCurrent.rollRateDemand * SAMPLE_TIME),
//      -66.0,
//      +66.0
//  );
//
//  // feed pid controller for bank demand -> roll rate demand
//  outputCurrent.rollRateDemand = pidControllerBank.calculate(
//      outputCurrent.bankDemand,
//      inputCurrent.bank
//  );
//
//  // bank angle limiter
//  outputCurrent.rollRateDemandLimiter = (abs(inputCurrent.bank) - 33.0) * (15.0 / 33.0);
//  outputCurrent.rollRateDemandLimiter = fmax(0, outputCurrent.rollRateDemandLimiter);
//  outputCurrent.rollRateDemandLimiter = copysign(outputCurrent.rollRateDemandLimiter, inputCurrent.bank);
//  outputCurrent.rollRateDemandLimiter *= -1;
//  outputCurrent.rollRateDemand += outputCurrent.rollRateDemandLimiter;
//
//  // feed pid controller for roll rate -> aileron position
//  outputCurrent.aileronPosition = pidControllerRollRate.calculate(
//      outputCurrent.rollRateDemand * DEG_TO_RAD,
//      input.rollRateRadPerSecond
//  );

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

  double k_xi_phi = pidControllerBank.getKp();
  double j_xi_phi = pidControllerBank.getKi();
  double h_xi_phi = pidControllerBank.getKd();
  double k_xi_p = pidControllerRollRate.getKp();

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
