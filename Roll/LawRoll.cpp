#include "LawRoll.h"
#include <cmath>

LawRoll::LawRoll()
    : pidController(
    0.03,
    1,
    -1,
    10.00,
    0.20,
    4.00,
    0.0
) {
}

void LawRoll::setErrorFactor(
    double factor
) {
  pidController.setErrorWeightFactor(factor);
  directWeightFactor = 1.0 - factor;
}

void LawRoll::setPidParameters(
    double Kp,
    double Ki,
    double Kd
) {
  pidController.setKp(Kp);
  pidController.setKi(Ki);
  pidController.setKd(Kd);
}

LawRoll::Output LawRoll::dataUpdated(
    LawRoll::Input input
) {
  // store input
  inputCurrent = input;

  // calculate load demand depending on sidestick position
  outputCurrent.rollRateDemand = 15 * inputCurrent.stickDeflection;

  outputCurrent.rollRateDemandLimiter = (abs(inputCurrent.bank) - 33.0) * (15.0 / 34.0);
  outputCurrent.rollRateDemandLimiter = fmax(0, outputCurrent.rollRateDemandLimiter);
  outputCurrent.rollRateDemandLimiter = copysign(outputCurrent.rollRateDemandLimiter, inputCurrent.bank);
  outputCurrent.rollRateDemandLimiter *= -1;

  outputCurrent.rollRateDemand += outputCurrent.rollRateDemandLimiter;

  // feed pid controller and calculate elevator position
  outputCurrent.aileronPosition = pidController.calculate(
      outputCurrent.rollRateDemand * DEG_TO_RAD,
      input.rollRateRadPerSecond
  );
  // add weighted direct control
  outputCurrent.aileronPosition += directWeightFactor * inputCurrent.stickDeflection;

  // store input and output for next iteration
  inputLast = inputCurrent;
  outputLast = outputCurrent;

  return outputCurrent;
}
