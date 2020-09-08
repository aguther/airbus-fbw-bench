#pragma once

#include "SimConnect.h"

#include <QTimer>
#include <windows.h>

class MainController : public QObject {
 Q_OBJECT

 public:
  MainController();
  ~MainController() override;

  void start();
  void stop();

 public slots:
  void update();

 private:
  struct AircraftData {
    double gForce;
  };

  struct InputControllerData {
    double elevatorPosition;
  };

  struct OutputData {
    double elevatorPosition;
  };

  QTimer *updateTimer = nullptr;
  HANDLE hSimConnect = nullptr;
  AircraftData aircraftData = {};
  InputControllerData inputControllerData = {};
  OutputData outputData = {};

  void simConnectSetupAircraftData() const;
  void simConnectSetupOutputData() const;
  void simConnectSetupInputControllerData() const;
  void simConnectProcessDispatchMessage(SIMCONNECT_RECV *pData, DWORD *cbData);
  void simConnectProcessEvent(const SIMCONNECT_RECV *pData);
  void simConnectProcessSimObjectDataByType(const SIMCONNECT_RECV *pData);
};
