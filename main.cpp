#include "MainWindow.h"
#include "MainController.h"

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

  // start main controller
  mainController.start();

  // execute
  return QApplication::exec();
}
