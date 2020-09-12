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
      &MainController::timeout
  );
}

MainController::~MainController() {
  if (updateTimer != nullptr) {
    updateTimer->stop();
    delete updateTimer;
  }
}

void MainController::start() {
  // connect to FS2020
  HRESULT result = SimConnect_Open(
      &hSimConnect,
      "airbus-fbw-bench",
      nullptr,
      0,
      nullptr,
      3
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
  SimConnect_Close(&hSimConnect);
}

void MainController::timeout() {
  update();
//  auto result = std::async(&MainController::update, this);
//  result.wait();
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
      pitchLawData,
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
  aircraftData.bank = -1.0 * inputAircraftData.bank;
  aircraftData.pitchRateDegreePerSecond = -1.0 * inputAircraftData
      .pitchRateDegreePerSecond;//(aircraftData.pitch - lastAircraftData.pitch) / aircraftData.updateTime;
  aircraftData.pitchRateRadPerSecond = aircraftData.pitchRateDegreePerSecond * DEG_TO_RAD;

  // get calculated data
  aircraftData.delta_g_force = aircraftData.gForce - lastAircraftData.gForce;
  aircraftData.c_star = aircraftData.delta_g_force + ((240 / 9.81) * aircraftData.pitchRateRadPerSecond);

  // store last aircraft data
  lastAircraftData = aircraftData;

  // ******************************************************************************************************************

  // get update on law pitch data
  pitchLawData = lawPitch.dataUpdated(aircraftData, inputControllerData);

  // ******************************************************************************************************************

  // get needed output values from each law data
  outputData.elevatorPosition = pitchLawData.elevatorPosition;
  //outputData.elevatorPosition = inputControllerData.elevatorPosition * 0.5;
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
      "PLANE BANK DEGREES",
      "DEGREES"
  );
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "ROTATION VELOCITY BODY X",
      "DEGREES PER SECOND"
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
}

void MainController::simConnectSetupOutputData() const {
  SimConnect_AddToDataDefinition(
      hSimConnect,
      1,
      "ELEVATOR POSITION",
      "Position"
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
