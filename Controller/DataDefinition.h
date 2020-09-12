#pragma once

struct InputAircraftData {
  double gForce;
  double pitch;
  double bank;
  double pitchRateDegreePerSecond;
};

struct AircraftData {
  double updateTime;
  double gForce;
  double pitch;
  double bank;
  double pitchRateDegreePerSecond;
  double pitchRateRadPerSecond;
  double delta_g_force;
  double c_star;
};

struct InputControllerData {
  double elevatorPosition;
};

struct PitchLawOutputData {
  double loadDemand;
  double c_star_demand;
  double elevatorPosition;
};

struct OutputData {
  double elevatorPosition;
};
