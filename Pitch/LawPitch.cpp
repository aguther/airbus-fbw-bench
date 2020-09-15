#include "LawPitch.h"

LawPitch::LawPitch()
    : pidControllerPitchRate(SAMPLE_TIME, 1, -1, 0.20, 0.005, 0.40, 0.0),
      pidControllerLoadDemand(SAMPLE_TIME, 10, -10, 1.0, 0.0, 0.0, 0.0) {
}

void LawPitch::setErrorFactor(
    double factor
) {
  pidControllerPitchRate.setErrorWeightFactor(factor);
  pidControllerLoadDemand.setErrorWeightFactor(factor);
  directWeightFactor = 1.0 - factor;
}

void LawPitch::setPidParameters(
    double pitchRateKp,
    double pitchRateKi,
    double pitchRateKd,
    double loadDemandKp,
    double loadDemandKi,
    double loadDemandKd
) {
  pidControllerPitchRate.setKp(pitchRateKp);
  pidControllerPitchRate.setKi(pitchRateKi);
  pidControllerPitchRate.setKd(pitchRateKd);
  pidControllerLoadDemand.setKp(loadDemandKp);
  pidControllerLoadDemand.setKi(loadDemandKi);
  pidControllerLoadDemand.setKd(loadDemandKd);
}

LawPitch::Output LawPitch::dataUpdated(
    LawPitch::Input input
) {
  // store input
  inputCurrent = input;

  // calculate needed variables
  outputCurrent.gForceDelta = inputCurrent.gForce - inputLast.gForce;
  outputCurrent.cStar = outputCurrent.gForceDelta + ((240 / 9.81) * inputCurrent.pitchRateRadPerSecond);

  // correction factor
  double pitchBankCompensation = (cos(inputCurrent.pitch * DEG_TO_RAD) / cos(inputCurrent.bank * DEG_TO_RAD));

  // calculate load demand depending on sidestick position
  outputCurrent.loadDemand = 1 + (1.5 * inputCurrent.stickDeflection * pitchBankCompensation);

  // calculate C* rule
  outputCurrent.cStarDemand = 3.0 * inputCurrent.stickDeflection * pitchBankCompensation;

  // feed pid controller for load demand -> pitch rate demand
//  outputCurrent.pitchRateDemand = pidControllerLoadDemand.calculate(
//      outputCurrent.loadDemand,
//      inputCurrent.gForce
//  );

  // feed pid controller for pitch rate -> elevator position
  outputCurrent.elevatorPosition = pidControllerPitchRate.calculate(
      outputCurrent.cStarDemand,
      outputCurrent.cStar
  );

  // add weighted direct control
  outputCurrent.elevatorPosition += directWeightFactor * inputCurrent.stickDeflection;

  // store input and output for next iteration
  inputLast = inputCurrent;
  outputLast = outputCurrent;

  return outputCurrent;
}
