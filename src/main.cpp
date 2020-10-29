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

  // execute
  int result = QApplication::exec();

  // ensure everything is stopped
  mainController.stop();

  // return
  return result;
}
