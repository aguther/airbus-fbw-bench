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
    double pitch;
    double radioHeightFeet;
  };

  struct Output {
    double bankDemand;
    double rollRateDemand;
    double rollRateDemandLimiter;
    double aileronPosition;
    double flightModeWeightFactor;
  };

  LawRoll();

  void setErrorFactor(
      double factor
  );

  void setPidParameters(
      double rollDemandKp,
      double rollDemandKi,
      double rollDemandKd,
      double bankDemandKp,
      double bankDemandKi,
      double bankDemandKd
  );

  LawRoll::Output dataUpdated(
      LawRoll::Input input
  );

 private:
  double SAMPLE_TIME = 0.03;
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  double overrideWeightFactor = 0.0;
  double flightModeWeightFactor = 0.0;

  double k_xi_phi = 0.0;
  double j_xi_phi = -0.2;
  double h_xi_phi = 0.0;
  double k_xi_p = -5.0;
  //double phi_d_integral = 0.0;

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
