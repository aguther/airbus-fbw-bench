#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(
    QWidget *parent
) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->loadFactor->setRangeY(-0.5, 2.5);
  ui->pitchRate->setRangeY(-15.0, 15.0);
  ui->c_star->setRangeY(-3.0, 3.0);

  ui->statusbar->setFont(QFont("Cascadia Code", 9));

  QObject::connect(
      ui->pushButtonConnect,
      &QPushButton::clicked,
      this,
      &MainWindow::buttonConnectClicked
  );
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::buttonConnectClicked() {
  if (ui->pushButtonConnect->text() == "CONNECT") {
    emit connect(ui->spinBoxConfigurationIndex->value());
    ui->pushButtonConnect->setText("DISCONNECT");
  } else {
    emit disconnect();
    ui->pushButtonConnect->setText("CONNECT");
  }
}

void MainWindow::updateData(
    InputAircraftData inputAircraftData,
    AircraftData aircraftData,
    InputControllerData inputControllerData,
    LawPitch::Output lawPitchOutput,
    OutputData outputData
) {
  ui->loadFactor->updateActualDemand(aircraftData.gForce, lawPitchOutput.loadDemand);
  ui->pitchRate->updateActualDemand(aircraftData.pitchRateDegreePerSecond, aircraftData.pitchRateRadPerSecond);
  ui->c_star->updateActualDemand(lawPitchOutput.cStar, lawPitchOutput.cStarDemand);

  ui->statusbar->showMessage(QString::asprintf(
      "T=%4.2f | C*=%+4.2f | Pitch=%+4.2f | PitchRate=%+4.2f | nz=%+4.2f | nzd=%4.2f",
      aircraftData.updateTime,
      lawPitchOutput.cStar,
      aircraftData.pitch,
      aircraftData.pitchRateDegreePerSecond,
      aircraftData.gForce,
      lawPitchOutput.loadDemand
  ));
}
