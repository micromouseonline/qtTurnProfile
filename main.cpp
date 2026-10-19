#include <QApplication>
#include <QDir>
#include "mainwindow.h"

int main(int argc, char *argv[]) {
  // high-DPI scaling is always enabled in Qt6
#ifdef Q_OS_LINUX
  // Use Fusion on Linux unless a style was requested via -style or QT_STYLE_OVERRIDE.
  // Checked before QApplication strips -style from argv.
  bool styleRequested = !qEnvironmentVariableIsEmpty("QT_STYLE_OVERRIDE");
  for (int i = 1; i < argc; i++) {
    QByteArray arg(argv[i]);
    if (arg.startsWith("-style") || arg.startsWith("--style")) {
      styleRequested = true;
    }
  }
#endif
  QApplication a(argc, argv);
#ifdef Q_OS_LINUX
  if (!styleRequested) {
    QApplication::setStyle("Fusion");
  }
#endif
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
