#pragma once

#include "Controller/DataDefinition.h"
#include "Pitch/LawPitch.h"
#include "Roll/LawRoll.h"

#include <QMainWindow>
#include <Windows.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
 Q_OBJECT

 public:
  explicit MainWindow(
      QWidget *parent = nullptr
  );

  ~MainWindow() override;

 public slots:
  void buttonConnectClicked();

  void buttonPitchParametersUpdateClicked();

  void buttonRollParametersUpdateClicked();

  void inputMaskingCheckBoxChanged(
      int state
  );

  void weightFactorSliderChanged(
      int value
  );

  void updateData(
      InputAircraftData inputAircraftData,
      AircraftData aircraftData,
      InputControllerData inputControllerData,
      LawPitch::Output lawPitchOutput,
      LawRoll::Output lawRollOutput,
      OutputData outputData
  );

 signals:
  void connect(
      int configurationIndex,
      bool isPitchEnabled,
      bool isRollEnabled
  );

  void disconnect();

  void inputMaskingChanged(
      bool isPitchEnabled,
      bool isRollEnabled
  );

  void weightFactorChanged(
      double pitch,
      double roll
  );

  void pitchParametersChanged(
      double pitchRateKp,
      double pitchRateKi,
      double pitchRateKd,
      double loadDemandKp,
      double loadDemandKi,
      double loadDemandKd
  );

  void rollParametersChanged(
      double rollDemandKp,
      double rollDemandKi,
      double rollDemandKd,
      double bankDemandKp,
      double bankDemandKi,
      double bankDemandKd
  );

 private:
  Ui::MainWindow *ui;
};
