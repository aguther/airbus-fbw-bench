#pragma once

struct InputAircraftData {
  double gForce;
  double pitch;
  double pitchRateRadPerSecond;
  double bank;
  double rollRateRadPerSecond;
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
  double aileronPosition;
};
