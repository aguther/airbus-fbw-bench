#include "LawPitch.h"
LawPitch::LawPitch()
    : pidController(0.03, 1, -1, 0.20, 0.005, 0.40) {
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

  // feed pid controller and calculate elevator position
  outputCurrent.elevatorPosition = pidController.calculate(outputCurrent.cStarDemand, outputCurrent.cStar);

  // store input and output for next iteration
  inputLast = inputCurrent;
  outputLast = outputCurrent;

  return outputCurrent;
}
