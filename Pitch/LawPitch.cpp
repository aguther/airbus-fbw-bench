#include <iostream>
#include "LawPitch.h"

LawPitch::LawPitch()
    : pidController_cStar(SAMPLE_TIME, 1, -1, 0.15, 0.005, 0.40, 0.0) {
}

void LawPitch::setErrorFactor(
    double factor
) {
  overrideWeightFactor = factor;
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

  // check if we touched down
  if (inputCurrent.radioHeightFeet == 0.0 && inputLast.radioHeightFeet > 0.0) {
    touchDownTime = QDateTime::currentDateTimeUtc();
  }

  // update flight mode weight
  if (inputCurrent.radioHeightFeet > 9.0) {
    // transition to flight mode
    if ((flightModeWeightFactor < 1.0 && inputCurrent.pitch > 8.0) || inputCurrent.radioHeightFeet > 400.0) {
      flightModeWeightFactor = limit(
          flightModeWeightFactor + (1.0 / (5.0 / SAMPLE_TIME)),
          0.0,
          1.0
      );
    }
  } else {
    // transition to ground mode
    if (touchDownTime.secsTo(QDateTime::currentDateTimeUtc()) > 5
        && flightModeWeightFactor > 0.0
        && inputCurrent.pitch < 2.5) {
      flightModeWeightFactor = limit(
          flightModeWeightFactor - (1.0 / (5.0 / SAMPLE_TIME)),
          0.0,
          1.0
      );
    }
  }
  flightModeWeightFactor = limit(
      flightModeWeightFactor,
      0.0,
      overrideWeightFactor
  );
  pidController_cStar.setErrorWeightFactor(flightModeWeightFactor);
  directWeightFactor = 1.0 - flightModeWeightFactor;

  // calculate needed variables
  outputCurrent.gForceDelta = inputCurrent.gForce - inputLast.gForce;
  outputCurrent.cStar = inputCurrent.gForce + inputCurrent.pitchRateRadPerSecond * C_STAR_FACTOR;

  // correction factor
  double pitchBankCompensation = (cos(inputCurrent.pitch * DEG_TO_RAD) / cos(inputCurrent.bank * DEG_TO_RAD));

  // calculate load demand depending on sidestick position
  outputCurrent.loadDemand = inputCurrent.stickDeflection >= 0
                             ? inputCurrent.stickDeflection * 1.5
                             : inputCurrent.stickDeflection * 2.0;
  outputCurrent.loadDemand += 1.0;
  outputCurrent.loadDemand *= pitchBankCompensation;
  outputCurrent.loadDemand = limit(outputCurrent.loadDemand, -1.0, +2.5);

  // calculate pitch rate demand
  if (inputCurrent.groundSpeed > 0.0) {
    outputCurrent.pitchRateDemand = RAD_TO_DEG * (
        (outputCurrent.loadDemand * 9.81 - cos(inputCurrent.pitch * DEG_TO_RAD) * 9.81)
            / (inputCurrent.groundSpeed / 1.825)
    );
  } else {
    outputCurrent.pitchRateDemand = 0.0;
  }

  // calculate C* rule
  outputCurrent.cStarDemand = outputCurrent.loadDemand;//3.0 * inputCurrent.stickDeflection * pitchBankCompensation;

  // feed pid controller for pitch rate -> elevator position
  outputCurrent.elevatorPosition = pidController_cStar.calculate(
      outputCurrent.cStarDemand,
      outputCurrent.cStar
  );

  // add weighted direct control
  outputCurrent.elevatorPosition += directWeightFactor * inputCurrent.stickDeflection;

  // integrate signal to trim
  outputCurrent.elevatorTrimPosition += DEG_TO_RAD *
      limit(
          limit(
              (outputCurrent.elevatorPosition * 13.5) / (13.0 / SAMPLE_TIME),
              -0.3,
              +0.3
          ),
          -13.0,
          +3.5
      );

  // store flight mode factor
  outputCurrent.flightModeWeightFactor = flightModeWeightFactor;

  // store input and output for next iteration
  inputLast = inputCurrent;
  outputLast = outputCurrent;

  return outputCurrent;
}

double LawPitch::limit(
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
