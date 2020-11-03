#include "MainController.h"

#include <QDebug>
#include <cmath>

using namespace std;
using namespace simconnect::toolbox::connection;

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
  // prepare data definitions
  simConnectSetupAircraftData();
  simConnectSetupInputData();
  simConnectSetupOutputData();

  // connect to FS2020
  bool result = simConnectInterfaceRead.connect(
      configurationIndex,
      "airbus-fbw-bench-read",
      *dataDefinitionRead,
      simConnectDataRead
  );
  result &= simConnectInterfaceInput.connect(
      configurationIndex,
      "airbus-fbw-bench-input",
      *dataDefinitionInput,
      simConnectDataInput
  );
  result &= simConnectInterfaceWrite.connect(
      configurationIndex,
      "airbus-fbw-bench-write",
      *dataDefinitionWrite,
      simConnectDataWrite
  );

  // check if everything worked
  if (!result) {
    stop();
    return;
  }

  // we are connected
  isConnected = true;

  // initialize model
  model.initialize();

  // start update loop
  updateTimer->start(STEP_TIME);
}

void MainController::stop() {
  // stop update loop
  updateTimer->stop();

  // terminate model
  model.terminate();

  // disconnect from FS2020
  if (isConnected) {
    // close connection
    simConnectInterfaceRead.disconnect();
    simConnectInterfaceInput.disconnect();
    simConnectInterfaceWrite.disconnect();

    // reset data objects
    simConnectDataRead.reset();
    dataDefinitionRead.reset();
    simConnectDataInput.reset();
    dataDefinitionInput.reset();
    simConnectDataWrite.reset();
    dataDefinitionWrite.reset();

    // set flag
    isConnected = false;
  }
}

void MainController::update() {
  // get update time
  auto now = QDateTime::currentDateTimeUtc();
  auto updateTime = lastUpdateTime.msecsTo(now) / 1000.0;
  lastUpdateTime = now;

  // ******************************************************************************************************************

  // request data
  simConnectInterfaceRead.requestData();

  // read data
  simConnectInterfaceRead.readData();
  model.FlyByWire_U.in.data.nz_g
      = any_cast<double>(simConnectDataRead->get(0));

  model.FlyByWire_U.in.data.Theta_deg
      = any_cast<double>(simConnectDataRead->get(1));

  model.FlyByWire_U.in.data.Phi_deg
      = any_cast<double>(simConnectDataRead->get(2));

  SIMCONNECT_DATA_XYZ worldRotationVelocity
      = any_cast<SIMCONNECT_DATA_XYZ>(simConnectDataRead->get(3));
  model.FlyByWire_U.in.data.qk_rad_s = worldRotationVelocity.x;
  model.FlyByWire_U.in.data.rk_rad_s = worldRotationVelocity.y;
  model.FlyByWire_U.in.data.pk_rad_s = worldRotationVelocity.z;

  model.FlyByWire_U.in.data.Vk_kt
      = any_cast<double>(simConnectDataRead->get(4));

  model.FlyByWire_U.in.data.radio_height_ft
      = any_cast<double>(simConnectDataRead->get(5));

  model.FlyByWire_U.in.data.CG_percent_MAC
      = any_cast<double>(simConnectDataRead->get(6));

  // ******************************************************************************************************************

  // read inputs
  simConnectInterfaceInput.readData();
  model.FlyByWire_U.in.input.delta_eta_pos
      = any_cast<double>(simConnectDataInput->get(0));
  model.FlyByWire_U.in.input.delta_xi_pos = any_cast<double>(simConnectDataInput->get(1));

  // ******************************************************************************************************************

  // step model
  model.step();

  // ******************************************************************************************************************

  // get model result and prepare data structure to write
  simConnectDataWrite->set(0, model.FlyByWire_Y.out.sim.raw.output.eta_pos);
  simConnectDataWrite->set(1, model.FlyByWire_Y.out.sim.raw.output.iH_deg);
  simConnectDataWrite->set(2, model.FlyByWire_Y.out.sim.raw.output.xi_pos);

  // set output data
  simConnectInterfaceWrite.sendData();

  // ******************************************************************************************************************

  // fire signal that data was updated
  emit dataUpdated(
      model.FlyByWire_Y,
      updateTime
  );
}

void MainController::simConnectSetupAircraftData() {
  // create object
  dataDefinitionRead = make_shared<SimConnectDataDefinition>();

  // add data definitions
  dataDefinitionRead->add(SimConnectVariable("G FORCE", "GFORCE"));
  dataDefinitionRead->add(SimConnectVariable("PLANE PITCH DEGREES", "DEGREE"));
  dataDefinitionRead->add(SimConnectVariable("PLANE BANK DEGREES", "DEGREE"));
  dataDefinitionRead->add(SimConnectVariable("STRUCT WORLD ROTATION VELOCITY", "STRUCT"));
  dataDefinitionRead->add(SimConnectVariable("AIRSPEED INDICATED", "KNOTS"));
  dataDefinitionRead->add(SimConnectVariable("RADIO HEIGHT", "FEET"));
  dataDefinitionRead->add(SimConnectVariable("CG PERCENT", "PERCENT OVER 100"));

  // create data object holding the actual data
  simConnectDataRead = make_shared<SimConnectData>(*dataDefinitionRead);
}

void MainController::simConnectSetupOutputData() {
  // create object
  dataDefinitionWrite = make_shared<SimConnectDataDefinition>();

  // add data definitions
  dataDefinitionWrite->add(SimConnectVariable("ELEVATOR POSITION", "POSITION"));
  dataDefinitionWrite->add(SimConnectVariable("ELEVATOR TRIM POSITION", "DEGREE"));
  dataDefinitionWrite->add(SimConnectVariable("AILERON POSITION", "POSITION"));

  // create data object holding the actual data
  simConnectDataWrite = make_shared<SimConnectData>(*dataDefinitionWrite);
}

void MainController::simConnectSetupInputData() {
  // create object
  dataDefinitionInput = make_shared<SimConnectDataDefinition>();

  // add data definitions
  dataDefinitionInput->add(SimConnectVariable("AXIS_ELEVATOR_SET", "TRUE"));
  dataDefinitionInput->add(SimConnectVariable("AXIS_AILERONS_SET", "TRUE"));

  // create data object holding the actual data
  simConnectDataInput = make_shared<SimConnectData>(*dataDefinitionInput);
}
