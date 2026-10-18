/**
 * @file main_window_ui.h
 * @brief Widgets and layouts of the main window, built in code.
 *
 * Replaces the old hand-edited uic output. Everything is placed by layouts,
 * so sizes follow the style and font metrics. Signals are connected by
 * MainWindow; nothing here is auto-connected by name.
 */

#ifndef MAIN_WINDOW_UI_H
#define MAIN_WINDOW_UI_H

#include <array>
#include "turn_profile.h"

class QCustomPlot;
class QDoubleSpinBox;
class QFormLayout;
class QGraphicsView;
class QGroupBox;
class QLabel;
class QMainWindow;
class QPushButton;
class QRadioButton;
class QSlider;
class QSpinBox;
class QTextEdit;
class QVBoxLayout;

/**
 * @brief Pointers to the main window widgets, owned by the window.
 *
 * Layout:
 * - top: profile graph (left), mouse path view with progress slider (right)
 * - bottom: turn summary, Parameters, Turn Type, then a column with
 *   Turn Profile, Start position and the Auto Scale/Reset buttons
 *
 * Profile radio buttons and profile parameter rows are added by MainWindow:
 * the profile buttons into profileLayout, the parameter rows into
 * paramLayout between the acceleration row and the offset row.
 */
struct MainWindowUi {
    QLabel *lblDataView;       ///< title above the graph
    QCustomPlot *customPlot;   ///< omega and speed against time
    QLabel *lblPathView;       ///< mouse state at the slider position
    QGraphicsView *pathView;   ///< maze, breadcrumbs and mouse
    QSlider *progressSlider;   ///< position along the turn, in track records

    QTextEdit *textEdit;       ///< turn summary, read only

    QGroupBox *gbSpinners;     ///< Parameters box
    QFormLayout *paramLayout;  ///< rows: speed, acc, profile params, offset, sensors, slip gradient, load
    QSpinBox *turnSpeedSpinBox;
    QSpinBox *accelerationSpinBox;
    QSpinBox *offsetSpinBox;
    QSpinBox *sensorAngleSpinBox;
    QDoubleSpinBox *slipGradientSpinBox;  ///< tyre slip angle per g, deg/g; 0 = no slip; all profiles
    QDoubleSpinBox *loadFactorSpinBox;    ///< tyre normal load / weight; 1 = no fan; all profiles

    QGroupBox *gbTurnType;     ///< Turn Type box
    std::array<QRadioButton *, TURN_TYPE_COUNT> turnButtons;  ///< indexed by Turn_t

    QGroupBox *gbTurnProfile;  ///< Turn Profile box
    QVBoxLayout *profileLayout;

    QSpinBox *startXSpinBox;
    QSpinBox *startYSpinBox;
    QPushButton *btnAutoScale;  ///< checkable: auto-scaled graph axes when checked
    QPushButton *btnReset;

    /**
     * @brief Create the widgets and layouts as the window's central widget.
     * @param window the main window; becomes the owner of everything created
     */
    void setupUi(QMainWindow *window);
};

#endif  // MAIN_WINDOW_UI_H
