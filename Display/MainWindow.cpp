#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(
    QWidget *parent
) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->loadFactor->setRangeY(-0.5, 2.5);
  ui->pitchRate->setRangeY(-15.0, 15.0);
  ui->c_star->setRangeY(-3.0, 3.0);

  ui->bank->setRangeY(-67.0, 67.0);
  ui->rollRate->setRangeY(-15.0, 15.0);

  ui->statusbar->setFont(QFont("Cascadia Code", 9));
  ui->labelPitch->setFont(QFont("Cascadia Code", 9));
  ui->labelRoll->setFont(QFont("Cascadia Code", 9));

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
    LawRoll::Output lawRollOutput,
    OutputData outputData
) {
  // common
  ui->statusbar->showMessage(QString::asprintf(
      "Update time = %4.2f s",
      aircraftData.updateTime
  ));

  // pitch
  ui->loadFactor->updateActualDemand(aircraftData.gForce, lawPitchOutput.loadDemand);
  ui->pitchRate->updateActualDemand(aircraftData.pitchRateDegreePerSecond, aircraftData.pitchRateRadPerSecond);
  ui->c_star->updateActualDemand(lawPitchOutput.cStar, lawPitchOutput.cStarDemand);
  ui->labelPitch->setText(QString::asprintf(
      "C*c=%+4.2f C*=%+4.2f | nzc=%+4.2f nz=%+4.2f | PR=%+4.2f°/s | P=%+4.2f°",
      lawPitchOutput.cStarDemand,
      lawPitchOutput.cStar,
      lawPitchOutput.loadDemand,
      aircraftData.gForce,
      aircraftData.pitchRateDegreePerSecond,
      aircraftData.pitch
  ));

  // roll
  ui->bank->updateActualDemand(aircraftData.bank, lawRollOutput.bankDemand);
  ui->rollRate->updateActualDemand(aircraftData.rollRateDegreePerSecond, lawRollOutput.rollRateDemand);
  ui->labelRoll->setText(QString::asprintf(
      "Bc=%+4.2f° B=%+4.2f° | RRc=%+4.2f°/s RR=%+4.2f°/s | %+4.2f",
      lawRollOutput.bankDemand,
      aircraftData.bank,
      lawRollOutput.rollRateDemand,
      aircraftData.rollRateDegreePerSecond,
      lawRollOutput.rollRateDemandLimiter
  ));
}
