#include "MainController.h"
#include "SimConnect.h"

#include <QDebug>

MainController::MainController() {
  updateTimer = new QTimer(this);
  connect(updateTimer, SIGNAL(timeout()), this, SLOT(update()));
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
    updateTimer->start(20);
  }
}

void MainController::stop() {
  // stop update loop
  updateTimer->stop();
  // disconnect from FS2020
  SimConnect_Close(&hSimConnect);
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

  // "process" -> here will be the update of the control laws with result
  outputData.elevatorPosition = inputControllerData.elevatorPosition;

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
}

void MainController::simConnectSetupAircraftData() const {
  SimConnect_AddToDataDefinition(
      hSimConnect,
      0,
      "G Force",
      "GForce"
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
      aircraftData = *((AircraftData *) &simObjectDataByType->dwData);
      break;

    default:
      // print unknown request id
      QTextStream(stdout) << "Unknown request id received (" << simObjectDataByType->dwRequestID << ")" << Qt::endl;
      break;
  }
}
