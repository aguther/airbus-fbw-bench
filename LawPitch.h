#pragma once

#include "DataDefinition.h"
#include "PidController/pid.h"
#include <cmath>

class LawPitch {
 public:
  LawPitch();

  PitchLawOutputData dataUpdated(
      AircraftData aircraftData,
      InputControllerData inputControllerData
  );

 private:
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  PID pidController;
  PitchLawOutputData data;
};
