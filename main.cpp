#include <QApplication>
#include <QDir>
#include "mainwindow.h"

int main(int argc, char *argv[]) {
  // get set up for highDPI scaling
  qunsetenv("QT_AUTO_SCREEN_SCALE_FACTOR");
  QGuiApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication a(argc, argv);
  a.setWindowIcon(QIcon(":/images/qtTurnProfiler.png"));
  QApplication::setQuitOnLastWindowClosed(true);
  MainWindow w;
  // find the screen that holds the current mouse pointer
  QScreen *screen = a.screenAt(QCursor::pos());
  // and its geometry
  QRect screenRect = screen->availableGeometry();
  // center the app window on that screen.
  QRect appRect = w.frameGeometry();
  // the move is what seems to force the application of scaling
  w.move(screenRect.center() - appRect.center());
  // finally, we are ready to go.
  w.show();

  return a.exec();
}
