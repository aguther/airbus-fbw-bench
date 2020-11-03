#pragma once

#include <QMainWindow>
#include <Windows.h>
#include <FlyByWire.h>

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

  void updateData(
      ExternalOutputs_FlyByWire_T data,
      double updateTime);

  signals:
  void connect(
      int configurationIndex
  );

  void disconnect();

 private:
  Ui::MainWindow *ui;
};
