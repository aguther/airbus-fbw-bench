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
  MainWindow w;
  // show main window
  w.show();

  // create main controller
  MainController mainController;
  mainController.start();

  // execute
  return QApplication::exec();
}
