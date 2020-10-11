#pragma once

#include "Controller/DataDefinition.h"
#include "PidController/pid.h"
#include <cmath>
#include <QDateTime>

class LawPitch {
 public:

  struct Input {
    double gForce;
    double pitch;
    double bank;
    double pitchRateRadPerSecond;
    double stickDeflection;
    double radioHeightFeet;
    double groundSpeed;
  };

  struct Output {
    double gForceDelta;
    double cStar;
    double pitchRateDemand;
    double loadDemand;
    double cStarDemand;
    double elevatorPosition;
    double elevatorTrimPosition;
    double flightModeWeightFactor;
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
  const double SAMPLE_TIME = 0.03;
  const double PI = 2 * acos(0.0);
  const double DEG_TO_RAD = PI / 180.0;
  const double RAD_TO_DEG = 180 / PI;
  const double C_STAR_FACTOR = 121.92 / 9.81;

  PID pidController_cStar;

  double overrideWeightFactor = 0.0;
  double directWeightFactor = 1.0;
  double flightModeWeightFactor = 0.0;

  QDateTime touchDownTime = QDateTime::currentDateTimeUtc();

  Input inputCurrent = {};
  Input inputLast = {};
  Output outputCurrent = {};
  Output outputLast = {};

  static double limit(
      double value,
      double min,
      double max
  );
};
