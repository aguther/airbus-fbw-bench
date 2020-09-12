#pragma once

#include "Controller/DataDefinition.h"
#include "Pitch/LawPitch.h"

#include <QMainWindow>
#include <QDateTime>
#include <Windows.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
 Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow() override;

 public slots:
  void buttonConnectClicked();
  void updateData(
      InputAircraftData inputAircraftData,
      AircraftData aircraftData,
      InputControllerData inputControllerData,
      LawPitch::Output lawPitchOutput,
      OutputData outputData
  );

 signals:
  void connect(int configurationIndex);
  void disconnect();

 private:
  Ui::MainWindow *ui;
  QDateTime lastUpdateTime;
};
