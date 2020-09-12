#include "LawPitch.h"
LawPitch::LawPitch()
    : pidController(0.03, 1, -1, 0.20, 0.005, 0.40) {
}
PitchLawOutputData LawPitch::dataUpdated(
    AircraftData aircraftData,
    InputControllerData inputControllerData
) {
  // calculate load demand depending on sidestick position
  data.loadDemand = 1 + 1.5 * inputControllerData.elevatorPosition;

  // calculate load demand corrected for pitch/roll
  data.loadDemandCorrected =
      data.loadDemand * (cos(aircraftData.pitch * DEG_TO_RAD) / cos(aircraftData.bank * DEG_TO_RAD));

  // calculate C* rule
  data.c_star_demand = data.loadDemandCorrected - 1;

  // feed pid controller and calculate elevator position
  data.elevatorPosition = pidController.calculate(data.c_star_demand, aircraftData.c_star);

  return data;
}
