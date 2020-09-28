#pragma once

#include "SimConnect.h"
#include "DataDefinition.h"
#include "Pitch/LawPitch.h"
#include "Roll/LawRoll.h"

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
      LawRoll::Output lawRollOutput,
      OutputData outputData
  );

 public slots:
  void start(
      int configurationIndex,
      bool isPitchAxisMasked,
      bool isRollAxisMasked
  );

  void stop();

  void update();

  void inputMaskingChanged(
      bool isPitchEnabled,
      bool isRollEnabled
  );

  void weightFactorChanged(
      double pitch,
      double roll
  );

  void pitchParametersChanged(
      double pitchRateKp,
      double pitchRateKi,
      double pitchRateKd,
      double loadDemandKp,
      double loadDemandKi,
      double loadDemandKd
  );

  void rollParametersChanged(
      double rollDemandKp,
      double rollDemandKi,
      double rollDemandKd,
      double bankDemandKp,
      double bankDemandKi,
      double bankDemandKd
  );

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

  bool isConnected = false;

  bool isPitchAxisMasked = false;
  bool isRollAxisMasked = false;

  LawPitch lawPitch = LawPitch();
  LawPitch::Output lawPitchOutput = {};

  LawRoll lawRoll = LawRoll();
  LawRoll::Output lawRollOutput = {};

  void simConnectSetupAircraftData() const;
  void simConnectSetupOutputData() const;
  void simConnectProcessDispatchMessage(SIMCONNECT_RECV *pData, DWORD *cbData);
  void simConnectProcessEvent(const SIMCONNECT_RECV *pData);
  void simConnectProcessSimObjectDataByType(const SIMCONNECT_RECV *pData);
  void simConnectMaskPitchAxis();
  void simConnectUnmaskPitchAxis();
  void simConnectMaskRollAxis();
  void simConnectUnmaskRollAxis();
  void processData();
};
