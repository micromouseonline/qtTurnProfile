#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>

#include "mouseitem.h"
#include "qcustomplot.h"
#include "track.h"
#include "robotstate.h"
#include <array>
#include <vector>
#include "turn_profile.h"

namespace Ui {
    class MainWindow;
}

const float loopInterval = 0.001;

/// Editable values for one turn of one profile; geometry is in turnGeometry
struct TurnValues {
    float offset;               ///< start offset back from the turn base point, mm
    float speed;                ///< turn speed, mm/s
    std::vector<float> values;  ///< profile parameters, in the order of TurnProfile::params()
};

class QButtonGroup;
class QDoubleSpinBox;
class QLabel;

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

public slots:

    void rePaint();

private slots:

    void selectProfile(int profile_index);

    void selectTurn(int turn_index);

    void on_btnReset_clicked();

    void on_btnRefresh_clicked();

    void on_startYSpinBox_valueChanged(int arg1);

    void on_startXSpinBox_valueChanged(int arg1);

    void on_progressSlider_valueChanged(int value);

    void on_accelerationSpinBox_valueChanged(int arg1);

    void on_turnSpeedSpinBox_valueChanged(int arg1);

    void on_offsetSpinBox_valueChanged(int arg1);

    void on_sensorAngleSpinBox_valueChanged(int arg1);

private:
    /// Working values, indexed [profile][turn]. Edits persist until reset;
    /// profiles are independent of each other.
    std::vector<std::array<TurnValues, TURN_TYPE_COUNT>> stored;
    Track track;  ///< the simulated turn and leadout
    Turn_t turn = TURN_SS90F;
    int turnProfile = 0;  ///< index into profile_registry()
    Ui::MainWindow *ui;
    MouseItem *theMouse;
    QGraphicsScene *mouseScene;
    QButtonGroup *profileGroup;  ///< profile radio buttons, id = registry index
    QButtonGroup *turnGroup;     ///< turn type radio buttons, id = Turn_t
    QGraphicsItemGroup *breadcrumbs = nullptr;  ///< breadcrumb markers, rebuilt each recalculation
    std::vector<QLabel *> paramLabels;          ///< labels for the current profile's parameters
    std::vector<QDoubleSpinBox *> paramBoxes;   ///< spinboxes for the current profile's parameters

    /// The current profile
    const TurnProfile &profile() const { return *profile_registry()[turnProfile]; }

    /// Working values for the current profile and turn type
    TurnValues &params() { return stored[turnProfile][turn]; }

    /// Working values for the current profile and turn type
    const TurnValues &params() const { return stored[turnProfile][turn]; }

    void setParams(int turn_index);

    void drawMazeWalls();

    void buildProfileButtons();

    void buildParamRows();

    void layoutParameterBox();

    void paramChanged(int index, double value);

    TurnSetup turnSetup() const;

    float peakAcceleration(float speed) const;

    float speedForAcceleration(float acceleration) const;

    void updateAccelerationBox();

    void updateStartPosition();

    void constrainParams();

    void showProgress();

    void showTurnSummary() const;

    void drawBreadcrumbs();

    void drawProfileGraph() const;

};

#endif  // MAINWINDOW_H
