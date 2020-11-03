#pragma once

#include <Windows.h>
#include <SimConnectData.h>
#include <SimConnectDataDefinition.h>
#include <SimConnectDataInterface.h>
#include <SimConnectInputInterface.h>

#include <QTimer>
#include <QDateTime>
#include <QThread>

#include <FlyByWire.h>

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
      ExternalOutputs_FlyByWire_T data,
      double updateTime
  );

 public slots:
  void start(
      int configurationIndex
  );

  void stop();

  void update();

 private:
  const int STEP_TIME = 20;

  QTimer *updateTimer = nullptr;
  QDateTime lastUpdateTime;

  FlyByWireModelClass model;

  std::shared_ptr<simconnect::toolbox::connection::SimConnectDataDefinition> dataDefinitionRead;
  std::shared_ptr<simconnect::toolbox::connection::SimConnectDataDefinition> dataDefinitionWrite;
  std::shared_ptr<simconnect::toolbox::connection::SimConnectDataDefinition> dataDefinitionInput;

  std::shared_ptr<simconnect::toolbox::connection::SimConnectData> simConnectDataRead;
  std::shared_ptr<simconnect::toolbox::connection::SimConnectData> simConnectDataWrite;
  std::shared_ptr<simconnect::toolbox::connection::SimConnectData> simConnectDataInput;

  simconnect::toolbox::connection::SimConnectDataInterface simConnectInterfaceRead;
  simconnect::toolbox::connection::SimConnectDataInterface simConnectInterfaceWrite;
  simconnect::toolbox::connection::SimConnectInputInterface simConnectInterfaceInput;

  bool isConnected = false;

  void simConnectSetupAircraftData();
  void simConnectSetupOutputData();
  void simConnectSetupInputData();
};
