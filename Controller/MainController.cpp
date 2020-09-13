#include "MainController.h"
#include "SimConnect.h"

#include <QDebug>
#include <cmath>

MainController::MainController() {
  updateTimer = new QTimer(this);
  updateTimer->setTimerType(Qt::PreciseTimer);
  QObject::connect(
      updateTimer,
      &QTimer::timeout,
      this,
      &MainController::update
  );
}

MainController::~MainController() {
  if (updateTimer != nullptr) {
    updateTimer->stop();
    delete updateTimer;
  }
}

void MainController::start(
    int configurationIndex
) {
  // connect to FS2020
  HRESULT result = SimConnect_Open(
      &hSimConnect,
      "airbus-fbw-bench",
      nullptr,
      0,
      nullptr,
      configurationIndex
  );

  // add data to definition
  simConnectSetupAircraftData();
  simConnectSetupInputControllerData();
  simConnectSetupOutputData();

  // start update loop
  if (S_OK == result) {
    updateTimer->start(30);
  }
}

void MainController::stop() {
  // stop update loop
  updateTimer->stop();
  // disconnect from FS2020
  SimConnect_RemoveClientEvent(
      hSimConnect,
      0,
      0
  );
  SimConnect_RemoveClientEvent(
      hSimConnect,
      1,
      1
  );
  SimConnect_Close(hSimConnect);
}

void MainController::update() {
  // request data
  SimConnect_RequestDataOnSimObjectType(
      hSimConnect,
      0,
      0,
      0,
      SIMCONNECT_SIMOBJECT_TYPE_USER
  );

  // get next dispatch message(s) and process them
  DWORD cbData;
  SIMCONNECT_RECV *pData;
  while (SUCCEEDED(SimConnect_GetNextDispatch(hSimConnect, &pData, &cbData))) {
    simConnectProcessDispatchMessage(pData, &cbData);
  }

  // process
  processData();

  // set output data
  SimConnect_SetDataOnSimObject(
      hSimConnect,
      1,
      SIMCONNECT_OBJECT_ID_USER,
      0,
      0,
      sizeof(outputData),
      &outputData
  );

  // fire signal that data was updated
  emit dataUpdated(
      inputAircraftData,
      aircraftData,
      inputControllerData,
      lawPitchOutput,
      lawRollOutput,
      outputData
  );
}

void MainController::processData() {
  // get update time
  auto now = QDateTime::currentDateTimeUtc();
  aircraftData.updateTime = lastUpdateTime.msecsTo(now) / 1000.0;
  lastUpdateTime = now;

  // ******************************************************************************************************************

  // correct values
  aircraftData.gForce = inputAircraftData.gForce;
  aircraftData.pitch = -1.0 * inputAircraftData.pitch;
  aircraftData.pitchRateRadPerSecond = -1.0 * inputAircraftData.pitchRateRadPerSecond;
  aircraftData.pitchRateDegreePerSecond = aircraftData.pitchRateRadPerSecond * RAD_TO_DEG;
  aircraftData.bank = -1.0 * inputAircraftData.bank;
  aircraftData.rollRateRadPerSecond = -1.0 * inputAircraftData.rollRateRadPerSecond;
  aircraftData.rollRateDegreePerSecond = aircraftData.rollRateRadPerSecond * RAD_TO_DEG;

  // ******************************************************************************************************************

  // get update on law pitch data
  LawPitch::Input inputPitch = {
      aircraftData.gForce,
      aircraftData.pitch,
      aircraftData.bank,
      aircraftData.pitchRateRadPerSecond,
      inputControllerData.elevatorPosition
  };
  lawPitchOutput = lawPitch.dataUpdated(inputPitch);

  // set output data
  outputData.elevatorPosition = lawPitchOutput.elevatorPosition;

  // ******************************************************************************************************************

  // get update on law roll data
  LawRoll::Input inputRoll = {
      aircraftData.bank,
      aircraftData.rollRateRadPerSecond,
      inputControllerData.aileronPosition
  };
  lawRollOutput = lawRoll.dataUpdated(inputRoll);

  // set output data
  outputData.aileronPosition = lawRollOutput.aileronPosition;
}

void MainController::simConnectSetupAircraftData() const {
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "G Force",
      "GFORCE"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "PLANE PITCH DEGREES",
      "DEGREES"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "ROTATION VELOCITY BODY X",
      "RADIANS PER SECOND"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "PLANE BANK DEGREES",
      "DEGREES"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "ROTATION VELOCITY BODY Z",
      "RADIANS PER SECOND"
  );
}

void MainController::simConnectSetupInputControllerData() const {
  SimConnect_MapClientEventToSimEvent(
      hSimConnect,
      0,
      "AXIS_ELEVATOR_SET"
  );
  SimConnect_AddClientEventToNotificationGroup(
      hSimConnect,
      0,
      0,
      TRUE
  );
  SimConnect_SetNotificationGroupPriority(
      hSimConnect,
      0,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE
  );
  SimConnect_MapClientEventToSimEvent(
      hSimConnect,
      1,
      "AXIS_AILERONS_SET"
  );
  SimConnect_AddClientEventToNotificationGroup(
      hSimConnect,
      1,
      1,
      TRUE
  );
  SimConnect_SetNotificationGroupPriority(
      hSimConnect,
      1,
      SIMCONNECT_GROUP_PRIORITY_HIGHEST_MASKABLE
  );
}

void MainController::simConnectSetupOutputData() const {
  SimConnect_AddToDataDefinition(
      hSimConnect,
      1,
      "ELEVATOR POSITION",
      "POSITION"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      1,
      "AILERON POSITION",
      "POSITION"
  );
}

void MainController::simConnectProcessDispatchMessage(
    SIMCONNECT_RECV *pData,
    DWORD *cbData
) {
  switch (pData->dwID) {
    case SIMCONNECT_RECV_ID_OPEN:
      // connection established
      QTextStream(stdout) << "OPEN" << Qt::endl;
      break;

    case SIMCONNECT_RECV_ID_QUIT:
      // connection lost
      QTextStream(stdout) << "CLOSED" << Qt::endl;
      break;

    case SIMCONNECT_RECV_ID_EVENT:
      // get event
      simConnectProcessEvent(pData);
      break;

    case SIMCONNECT_RECV_ID_SIMOBJECT_DATA_BYTYPE:
      // process data
      simConnectProcessSimObjectDataByType(pData);
      break;

    case SIMCONNECT_RECV_ID_EXCEPTION:
      // exception
      auto *except = (SIMCONNECT_RECV_EXCEPTION *) pData;
      QTextStream(stdout) << except->dwException << Qt::endl;
      break;
  }
}

void MainController::simConnectProcessEvent(
    const SIMCONNECT_RECV *pData
) {
  // get event
  auto *event = (SIMCONNECT_RECV_EVENT *) pData;

  // process depending on event id
  switch (event->uEventID) {
    case 0:
      // store elevator position
      inputControllerData.elevatorPosition = 0 - (static_cast<long>(event->dwData) / 16384.0);
      break;

    case 1:
      // store aileron position
      inputControllerData.aileronPosition = 0 - (static_cast<long>(event->dwData) / 16384.0);
      break;

    default:
      // print unknown event id
      QTextStream(stdout) << "Unknown event id received (" << event->uEventID << ")" << Qt::endl;
      break;
  }
}

void MainController::simConnectProcessSimObjectDataByType(
    const SIMCONNECT_RECV *pData
) {
  // get data object
  auto *simObjectDataByType = (SIMCONNECT_RECV_SIMOBJECT_DATA_BYTYPE *) pData;

  // process depending on request id
  switch (simObjectDataByType->dwRequestID) {
    case 0:
      // store aircraft data
      inputAircraftData = *((InputAircraftData *) &simObjectDataByType->dwData);
      break;

    default:
      // print unknown request id
      QTextStream(stdout) << "Unknown request id received (" << simObjectDataByType->dwRequestID << ")" << Qt::endl;
      break;
  }
}
