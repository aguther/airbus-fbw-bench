#include "LawPitch.h"

LawPitch::LawPitch()
    : pidController_cStar(SAMPLE_TIME, 1, -1, 0.20, 0.005, 0.40, 0.0) {
}

void LawPitch::setErrorFactor(
    double factor
) {
  pidController_cStar.setErrorWeightFactor(factor);
  directWeightFactor = 1.0 - factor;
}

void LawPitch::setPidParameters(
    double pitchRateKp,
    double pitchRateKi,
    double pitchRateKd
) {
  pidController_cStar.setKp(pitchRateKp);
  pidController_cStar.setKi(pitchRateKi);
  pidController_cStar.setKd(pitchRateKd);
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

  // feed pid controller for pitch rate -> elevator position
  outputCurrent.elevatorPosition = pidController_cStar.calculate(
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
