#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(
    QWidget *parent
) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  ui->c_star->setRangeY(-1.0, 3.0);
  ui->pitchRate->setRangeY(-5.0, 5.0);

  ui->bank->setRangeY(-67.0, 67.0);
  ui->rollRate->setRangeY(-15.0, 15.0);

  ui->statusbar->setFont(QFont("Consolas", 9));
  ui->labelPitch->setFont(QFont("Consolas", 9));
  ui->labelRoll->setFont(QFont("Consolas", 9));

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
    emit connect(
        ui->spinBoxConfigurationIndex->value()
    );
    ui->pushButtonConnect->setText("DISCONNECT");
  } else {
    emit disconnect();
    ui->pushButtonConnect->setText("CONNECT");
  }
}

void MainWindow::updateData(
    ExternalOutputs_FlyByWire_T data,
    double updateTime
) {
  // common
  ui->statusbar->showMessage(QString::asprintf(
      "Update time = %4.2f s",
      updateTime
  ));

  // pitch
  ui->c_star->updateActualDemand(data.out.pitch.law_normal.Cstar_g, data.out.pitch.law_normal.Cstar_c_g);
  ui->pitchRate->updateActualDemand(data.out.sim.data.qk_deg_s, 0);
  ui->labelPitch->setText(QString::asprintf(
      "FMF=%+4.2f | C*c=%+4.2f C*=%+4.2f | nz=%+4.2f | PR=%+4.2f°/s | P=%+4.2f°",
      data.out.pitch.data_computed.in_flight_gain,
      data.out.pitch.law_normal.Cstar_c_g,
      data.out.pitch.law_normal.Cstar_g,
      data.out.sim.data.nz_g,
      data.out.sim.data.pk_deg_s,
      data.out.sim.data.Theta_deg
  ));

  // roll
  ui->bank->updateActualDemand(data.out.sim.data.Phi_deg, data.out.roll.law_normal.Phi_c_deg);
  ui->rollRate->updateActualDemand(data.out.sim.data.pk_deg_s, data.out.roll.law_normal.pk_c_deg_s);
  ui->labelRoll->setText(QString::asprintf(
      "FMF=%+4.2f | Bc=%+4.2f° B=%+4.2f° | RRc=%+4.2f°/s RR=%+4.2f°/s",
      data.out.roll.data_computed.in_flight_gain,
      data.out.roll.law_normal.Phi_c_deg,
      data.out.sim.data.Phi_deg,
      data.out.roll.law_normal.pk_c_deg_s,
      data.out.sim.data.pk_deg_s
  ));
}
