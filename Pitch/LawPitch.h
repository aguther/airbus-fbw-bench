#pragma once

#include "Controller/DataDefinition.h"
#include "PidController/pid.h"
#include <cmath>

class LawPitch {
 public:

  struct Input {
    double gForce;
    double pitch;
    double bank;
    double pitchRateRadPerSecond;
    double stickDeflection;
  };

  struct Output {
    double gForceDelta;
    double cStar;
    double loadDemand;
    double cStarDemand;
    double elevatorPosition;
  };

  LawPitch();

  LawPitch::Output dataUpdated(
      LawPitch::Input input
  );

 private:
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  PID pidController;

  Input inputCurrent = {};
  Input inputLast = {};
  Output outputCurrent = {};
  Output outputLast = {};
};
