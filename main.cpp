#include "Display/MainWindow.h"
#include "Controller/MainController.h"

#include <QApplication>

int main(
    int argc,
    char *argv[]
) {
  // create application
  QApplication application(argc, argv);

  // create main window
  MainWindow mainWindow;
  // show main window
  mainWindow.show();

  // create main controller
  MainController mainController;

  // connect signals and slots
  QObject::connect(
      &mainController,
      &MainController::dataUpdated,
      &mainWindow,
      &MainWindow::updateData
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::connect,
      &mainController,
      &MainController::start
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::disconnect,
      &mainController,
      &MainController::stop
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::inputMaskingChanged,
      &mainController,
      &MainController::inputMaskingChanged
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::weightFactorChanged,
      &mainController,
      &MainController::weightFactorChanged
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::pitchParametersChanged,
      &mainController,
      &MainController::pitchParametersChanged
  );

  QObject::connect(
      &mainWindow,
      &MainWindow::rollParametersChanged,
      &mainController,
      &MainController::rollParametersChanged
  );

  // execute
  int result = QApplication::exec();

  // ensure everything is stopped
  mainController.stop();

  // return
  return result;
}
