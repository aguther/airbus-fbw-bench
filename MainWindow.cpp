#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(
    QWidget *parent
) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->loadFactor->setRangeY(-0.5, 2.5);
  ui->pitchRate->setRangeY(-15.0, 15.0);
  ui->c_star->setRangeY(-1.5, 1.5);

  ui->statusbar->setFont(QFont("Cascadia Code", 9));
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::updateData(
    InputAircraftData inputAircraftData,
    AircraftData aircraftData,
    InputControllerData inputControllerData,
    PitchLawOutputData setPointData,
    OutputData outputData
) {
  ui->loadFactor->updateActualDemand(aircraftData.gForce, setPointData.loadDemandCorrected);
  ui->pitchRate->updateActualDemand(aircraftData.pitchRateDegreePerSecond, aircraftData.pitchRateRadPerSecond);
  ui->c_star->updateActualDemand(aircraftData.c_star, setPointData.c_star_demand);

  ui->statusbar->showMessage(QString::asprintf(
      "Time = %4.2f | C* = %+4.2f | Pitch = %+4.2f° | Pitch Rate = %+4.2f °/s",
      aircraftData.updateTime,
      aircraftData.c_star,
      aircraftData.pitch,
      aircraftData.pitchRateDegreePerSecond
  ));
}
