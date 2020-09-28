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
    double pitchRateDemand;
    double loadDemand;
    double cStarDemand;
    double elevatorPosition;
  };

  LawPitch();

  void setErrorFactor(
      double factor
  );

  void LawPitch::setPidParameters(
      double pitchRateKp,
      double pitchRateKi,
      double pitchRateKd
  );

  LawPitch::Output dataUpdated(
      LawPitch::Input input
  );

 private:
  double SAMPLE_TIME = 0.03;
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  PID pidController_cStar;

  double directWeightFactor = 1.0;

  Input inputCurrent = {};
  Input inputLast = {};
  Output outputCurrent = {};
  Output outputLast = {};
};
