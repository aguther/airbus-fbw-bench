#pragma once

#include "SimConnect.h"
#include "DataDefinition.h"
#include "LawPitch.h"

#include <QTimer>
#include <QDateTime>
#include <QThread>
#include <windows.h>

class MainController : public QObject {
 Q_OBJECT

 public:
  MainController();
  ~MainController() override;

  void start();
  void stop();

 signals:
  void simConnectConnected();
  void simConnectDisconnected();
  void simConnectFailed();
  void dataUpdated(
      InputAircraftData inputAircraftData,
      AircraftData aircraftData,
      InputControllerData inputControllerData,
      PitchLawOutputData setPointData,
      OutputData outputData
  );

 public slots:
  void timeout();
  void update();

 private:
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  QTimer *updateTimer = nullptr;
  HANDLE hSimConnect = nullptr;
  InputAircraftData inputAircraftData = {};
  AircraftData aircraftData;
  AircraftData lastAircraftData = {};
  InputControllerData inputControllerData = {};
  PitchLawOutputData pitchLawData = {};
  OutputData outputData = {};
  QDateTime lastUpdateTime;

  LawPitch lawPitch = LawPitch();

  void simConnectSetupAircraftData() const;
  void simConnectSetupOutputData() const;
  void simConnectSetupInputControllerData() const;
  void simConnectProcessDispatchMessage(SIMCONNECT_RECV *pData, DWORD *cbData);
  void simConnectProcessEvent(const SIMCONNECT_RECV *pData);
  void simConnectProcessSimObjectDataByType(const SIMCONNECT_RECV *pData);
  void processData();
};
