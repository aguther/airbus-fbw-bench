#pragma once

#include "SimConnect.h"
#include "DataDefinition.h"
#include "Pitch/LawPitch.h"

#include <QTimer>
#include <QDateTime>
#include <QThread>
#include <Windows.h>

class MainController : public QObject {
 Q_OBJECT

 public:
  MainController();
  ~MainController() override;

 signals:
  void simConnectConnected();
  void simConnectDisconnected();
  void simConnectFailed();
  void dataUpdated(
      InputAircraftData inputAircraftData,
      AircraftData aircraftData,
      InputControllerData inputControllerData,
      LawPitch::Output lawPitchOutput,
      OutputData outputData
  );

 public slots:
  void start(int configurationIndex);
  void stop();
  void update();

 private:
  double PI = 2 * acos(0.0);
  double DEG_TO_RAD = PI / 180.0;
  double RAD_TO_DEG = 180 / PI;

  QTimer *updateTimer = nullptr;
  HANDLE hSimConnect = nullptr;
  InputAircraftData inputAircraftData = {};
  AircraftData aircraftData = {};
  InputControllerData inputControllerData = {};
  OutputData outputData = {};
  QDateTime lastUpdateTime;

  LawPitch lawPitch = LawPitch();
  LawPitch::Output lawPitchOutput = {};

  void simConnectSetupAircraftData() const;
  void simConnectSetupOutputData() const;
  void simConnectSetupInputControllerData() const;
  void simConnectProcessDispatchMessage(SIMCONNECT_RECV *pData, DWORD *cbData);
  void simConnectProcessEvent(const SIMCONNECT_RECV *pData);
  void simConnectProcessSimObjectDataByType(const SIMCONNECT_RECV *pData);
  void processData();
};
