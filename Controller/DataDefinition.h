#pragma once

#include "SimConnect.h"

struct InputAircraftData {
  double gForce;
  double pitch;
  double bank;
  SIMCONNECT_DATA_XYZ worldRotationVelocity;
  double incidenceAlpha;
  double incidenceBeta;
  double radioHeight;
  double groundSpeedKnots;
};

struct AircraftData {
  double updateTime;
  double gForce;
  double pitch;
  double pitchRateRadPerSecond;
  double pitchRateDegreePerSecond;
  double bank;
  double rollRateRadPerSecond;
  double rollRateDegreePerSecond;
};

struct InputControllerData {
  double elevatorPosition;
  double aileronPosition;
};

struct OutputData {
  double elevatorPosition;
  double elevatorTrimPosition;
  double aileronPosition;
};
