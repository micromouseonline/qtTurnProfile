#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>

#include "mouseitem.h"
#include "qcustomplot.h"
#include "track.h"
#include "pathitem.h"
#include "robotstate.h"

namespace Ui {
    class MainWindow;
}

// parameters for each turn
struct TurnParams {
    float baseX;
    float baseY;
    float offs;
    float lambda;
    float radius;
    float startAngle;
    float angle;
    float speed;   // this could be acceleration to match Decimus
    float length;  // for cubics
};


// index inti the turn parameters table
enum Turn_t {
    TURN_SS90F = 0, TURN_SS180, TURN_SD45, TURN_SD135, TURN_DS45, TURN_DS135, TURN_DD90, TURN_SS90E, TURN_TYPE_COUNT
};

enum Profile_t {
    PR_TRAPEZOID, PR_QUADRATIC, PR_SINUSOID, PR_CUBIC
};

const float loopInterval = 0.001;

extern TurnParams defaultParams[TURN_TYPE_COUNT];

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    Track g_track;

    explicit MainWindow(QWidget *parent = nullptr);

    ~MainWindow() override;

    void drawMazeWalls();

    void reset_track();

protected:
    void closeEvent(QCloseEvent *event) Q_DECL_OVERRIDE;

public slots:


    void rePaint();

    void calculateTrapezoid(float start_x, float start_y);

    void calculateQuadratic(float start_x, float start_y);

    void calculateCubic(float start_x, float start_y);

    void calculateSine(float start_x, float start_y);


private slots:

    void on_btnReset_clicked();

    void on_btnRefresh_clicked();

    void on_startYSpinBox_valueChanged(int arg1);

    void on_startXSpinBox_valueChanged(int arg1);

    void on_progressSlider_valueChanged(int value);

    void on_radiusSpinBox_valueChanged(int arg1);

    void on_cubicLengthSpinBox_valueChanged(int arg1);

    void on_cubicGammaSpinBox_valueChanged(int arg1);

    void on_lambdaSpinBox_valueChanged(int arg1);

    void on_accelerationSpinBox_valueChanged(int arg1);

    void on_turnSpeedSpinBox_valueChanged(int arg1);

    void on_offsetSpinBox_valueChanged(int arg1);

    void on_rbTrapezoid_clicked();

    void on_rbQuadratic_clicked();

    void on_rbSinusoid_clicked();

    void on_rbCubic_clicked();

    void on_sensorAngleSpinBox_valueChanged(int arg1);

    void on_rbSS90F_clicked();

    void on_rbSS180_clicked();

    void on_rbSD45_clicked();

    void on_rbSD135_clicked();

    void on_rbDS45_clicked();

    void on_rbDS135_clicked();

    void on_rbDD90_clicked();

    void on_rbSS90E_clicked();

private:
    TurnParams currentParams[TURN_TYPE_COUNT]{};
    Turn_t turn = TURN_SS90F;
    int turnProfile = PR_TRAPEZOID;
    Ui::MainWindow *ui;
    MouseItem *theMouse;
    PathItem *thePath;
    QGraphicsScene *mouseScene;

    void setParams(int turn_index);

    void showTurnSummary() const;

    void drawBreadcrumbs() const;

    void drawProfileGraph() const;

    void calculate_leadout(RobotState &robot_state, float distance = 100);
};

#endif  // MAINWINDOW_H
