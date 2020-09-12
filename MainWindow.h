#pragma once

#include "DataDefinition.h"

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
  void updateData(
      InputAircraftData inputAircraftData,
      AircraftData aircraftData,
      InputControllerData inputControllerData,
      PitchLawOutputData setPointData,
      OutputData outputData
  );

 private:
  Ui::MainWindow *ui;
  QDateTime lastUpdateTime;
};
