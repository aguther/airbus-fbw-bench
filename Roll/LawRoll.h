#pragma once

#include "Controller/DataDefinition.h"
#include "PidController/pid.h"
#include <cmath>

class LawRoll {
 public:

  struct Input {
    double bank;
    double rollRateRadPerSecond;
    double stickDeflection;
  };

  struct Output {
    double bankDemand;
    double rollRateDemand;
    double rollRateDemandLimiter;
    double aileronPosition;
  };

  LawRoll();

  LawRoll::Output dataUpdated(
      LawRoll::Input input
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
