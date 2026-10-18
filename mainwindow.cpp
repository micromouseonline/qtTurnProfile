#include "mainwindow.h"
#include <QButtonGroup>
#include <cmath>
#include "algorithm"
#include "ui_mainwindow.h"
#include "track.h"
#include "pathitem.h"
#include "robotstate.h"

#define PI 3.14159265359
#define RADIANS(x) ((x)*PI / 180.0)
#define DEGREES(x) ((x)*180.0 / PI)

// Default speeds in all tables give a peak centripetal acceleration of 9800 mm/s/s:
// speed = sqrt(9800 * min radius), rounded to the nearest mm/s.
TurnParams defaultParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 112, .startAngle = 0, .angle = 90, .speed = 1048, .length = 248},    // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 85, .startAngle = 0, .angle = 180, .speed = 913, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 65, .radius = 110, .startAngle = 0, .angle = 45, .speed = 1038, .length = 134},     // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 83, .startAngle = 0, .angle = 135, .speed = 902, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 120, .startAngle = -45, .angle = 45, .speed = 1084, .length = 144},   // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 80, .startAngle = -45, .angle = 135, .speed = 885, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 74, .startAngle = -45, .angle = 90, .speed = 852, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 56, .startAngle = 0, .angle = 90, .speed = 741, .length = 124},       // SS90E
};

// as defaultParams, but SD45 lambda reduced: 65 exceeds the quadratic limit (64) at radius 110
TurnParams quadraticParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 112, .startAngle = 0, .angle = 90, .speed = 1048, .length = 248},    // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 85, .startAngle = 0, .angle = 180, .speed = 913, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 60, .radius = 110, .startAngle = 0, .angle = 45, .speed = 1038, .length = 134},     // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 83, .startAngle = 0, .angle = 135, .speed = 902, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 120, .startAngle = -45, .angle = 45, .speed = 1084, .length = 144},   // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 80, .startAngle = -45, .angle = 135, .speed = 885, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 74, .startAngle = -45, .angle = 90, .speed = 852, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 56, .startAngle = 0, .angle = 90, .speed = 741, .length = 124},       // SS90E
};

TurnParams trapezoidParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 96, .startAngle = 0, .angle = 90, .speed = 970, .length = 248},     // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 82, .startAngle = 0, .angle = 180, .speed = 896, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 65, .radius = 89, .startAngle = 0, .angle = 45, .speed = 934, .length = 134},      // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 77, .startAngle = 0, .angle = 135, .speed = 869, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 95, .startAngle = -45, .angle = 45, .speed = 965, .length = 144},    // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 73, .startAngle = -45, .angle = 135, .speed = 846, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 61, .startAngle = -45, .angle = 90, .speed = 773, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 48, .startAngle = 0, .angle = 90, .speed = 686, .length = 124},       // SS90E
};

// cubic profile: as defaultParams, speeds set from the cubic peak curvature
// (min radius = length / (1.5 * angle in radians))
TurnParams cubicParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 112, .startAngle = 0, .angle = 90, .speed = 1016, .length = 248},    // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 85, .startAngle = 0, .angle = 180, .speed = 877, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 65, .radius = 110, .startAngle = 0, .angle = 45, .speed = 1056, .length = 134},     // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 83, .startAngle = 0, .angle = 135, .speed = 859, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 120, .startAngle = -45, .angle = 45, .speed = 1094, .length = 144},   // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 80, .startAngle = -45, .angle = 135, .speed = 844, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 74, .startAngle = -45, .angle = 90, .speed = 848, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 56, .startAngle = 0, .angle = 90, .speed = 718, .length = 124},       // SS90E
};

/**
 * @brief Default parameter table for a profile.
 * @param profile profile index (Profile_t)
 * @return table of TURN_TYPE_COUNT entries the profile starts from and resets to
 */
static const TurnParams *profileDefaults(int profile) {
    switch (profile) {
        case PR_TRAPEZOID:
            return trapezoidParams;
        case PR_QUADRATIC:
            return quadraticParams;
        case PR_CUBIC:
            return cubicParams;
        default:
            return defaultParams;
    }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    // on_<widget>_<signal> slots are auto-connected by setupUi (connectSlotsByName)

    for (int p = 0; p < PR_COUNT; p++) {
        for (int i = 0; i < TURN_TYPE_COUNT; i++) {
            profileParams[p][i] = profileDefaults(p)[i];
        }
    }

    thePath = new PathItem;
    theMouse = new MouseItem;
    theMouse->setSensorAngle(ui->sensorAngleSpinBox->value());
    mouseScene = new QGraphicsScene;
    mouseScene->setSceneRect(-100, -100, 380, 380);
    mouseScene->setBackgroundBrush(QBrush(Qt::black, Qt::SolidPattern));
    drawMazeWalls();
    mouseScene->addItem(theMouse);
//    mouseScene->addItem(thePath);
    ui->pathView->setScene(mouseScene);
    ui->pathView->setRenderHint(QPainter::Antialiasing);
    // button ids are the Turn_t / Profile_t values
    profileGroup = new QButtonGroup(this);
    profileGroup->addButton(ui->rbTrapezoid, PR_TRAPEZOID);
    profileGroup->addButton(ui->rbQuadratic, PR_QUADRATIC);
    profileGroup->addButton(ui->rbSinusoid, PR_SINUSOID);
    profileGroup->addButton(ui->rbCubic, PR_CUBIC);
    connect(profileGroup, &QButtonGroup::idClicked, this, &MainWindow::selectProfile);

    turnGroup = new QButtonGroup(this);
    turnGroup->addButton(ui->rbSS90F, TURN_SS90F);
    turnGroup->addButton(ui->rbSS180, TURN_SS180);
    turnGroup->addButton(ui->rbSD45, TURN_SD45);
    turnGroup->addButton(ui->rbSD135, TURN_SD135);
    turnGroup->addButton(ui->rbDS45, TURN_DS45);
    turnGroup->addButton(ui->rbDS135, TURN_DS135);
    turnGroup->addButton(ui->rbDD90, TURN_DD90);
    turnGroup->addButton(ui->rbSS90E, TURN_SS90E);
    connect(turnGroup, &QButtonGroup::idClicked, this, &MainWindow::selectTurn);

    ui->rbSinusoid->click();
    ui->rbSS90F->click();
    setWindowTitle(tr("Turn Render"));
    // now we have everything set up, top the window from resizing
    setFixedSize(840, 820);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_btnRefresh_clicked() {
    rePaint();
}

void MainWindow::showTurnSummary() const {
    float max_alpha = g_track.get_max_alpha() / loopInterval;
    qreal wheelAcceleration = theMouse->m_radius * RADIANS(max_alpha);
    Pose exit_pose = g_track.get_exit_pose();

    ui->textEdit->clear();
    ui->textEdit->append("Turn Parameters");
    ui->textEdit->append("   Min Radius: " + QString::number(ui->radiusSpinBox->value(), 'f', 0).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("   Turn Speed: " + QString::number(ui->turnSpeedSpinBox->value(), 'f', 0).rightJustified(6, ' ') + " mm/s");
    ui->textEdit->append("Cent'l Acceln: " + QString::number(g_track.get_max_acceleration(), 'f', 0).rightJustified(6, ' ') + " mm/s/s");
    ui->textEdit->append(" Wheel Acceln: " + QString::number(wheelAcceleration, 'f', 0).rightJustified(6, ' ') + " mm/s/s");
    ui->textEdit->append("        Alpha: " + QString::number(max_alpha, 'f', 0).rightJustified(6, ' ') + " deg/s/s");
    ui->textEdit->append("    Omega Max: " + QString::number(g_track.get_max_omega(), 'f', 0).rightJustified(6, ' ') + " deg/s");
    ui->textEdit->append("       Exit X: " + QString::number(exit_pose.x(), 'f', 1).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("       Exit Y: " + QString::number(exit_pose.y(), 'f', 1).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("        Theta: " + QString::number(exit_pose.theta(), 'f', 1).rightJustified(6, ' ') + " deg");
    ui->textEdit->append("     Distance: " + QString::number(g_track.turn_length(), 'f', 0).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("         Time: " + QString::number(g_track.get_turn_time(), 'f', 3).rightJustified(6, ' ') + " s");
    ui->textEdit->append("   Exit Speed: " + QString::number(g_track.get_exit_speed(), 'f', 0).rightJustified(6, ' ') + " mm/s");
    if (turnProfile != PR_CUBIC) {
        ui->textEdit->append("   Lambda Max: " + QString::number(int(maxLambda())).rightJustified(6, ' ') + " mm");
    }
}

void MainWindow::rePaint() {
    clampLambda();
    simulate_turn(Profile_t(turnProfile), turnInput(), g_track);
    showTurnSummary();
    ui->progressSlider->setMinimum(0);
    ui->progressSlider->setMaximum(g_track.finish_index());
    drawBreadcrumbs();
    drawProfileGraph();
    showProgress();
}

/**
 * @brief Show the mouse and its state at the progress slider position.
 *
 * Uses the already calculated track, so it is cheap enough to run on every
 * slider movement. rePaint() calls it after recalculating.
 */
void MainWindow::showProgress() {
    int progress = ui->progressSlider->value();
    Pose pose = g_track.get_pose_at(progress);
    theMouse->setPose(pose);
    theMouse->setVelocity(g_track.get_speed_at(progress));
    ui->lblPathView->setText(
            QString::number(g_track.record_count(), 'f', 0).rightJustified(4, ' ') + "  " +
            QString::number(g_track.get_distance_at(progress), 'f', 0).rightJustified(3, ' ') + "mm " +
            "(" + QString::number(pose.x(), 'f', 0).rightJustified(3, ' ') +
            "," + QString::number(pose.y(), 'f', 0).rightJustified(3, ' ') +
            ") ang:" + QString::number(pose.theta(), 'f', 0).rightJustified(3, ' ') + "deg, " +
            "omega: " + QString::number(g_track.get_omega_at(progress), 'f', 0).rightJustified(4, ' ') + " deg/s");
}

void MainWindow::drawProfileGraph() const {
    // create graph and assign data to it:
    ui->customPlot->clearGraphs();
    ui->customPlot->addGraph();
    QVector<qreal> xaxis;
    QVector<qreal> yaxis_omega;
    QVector<qreal> yaxis_speed;
    QVector<qreal> yaxis_left_wheel;
    QVector<qreal> yaxis_right_wheel;
    for (int i = 1; i < g_track.record_count(); i++) {
        xaxis.push_back(g_track.get_time_at(i));
        float omega = g_track.get_omega_at(i);
        float speed = g_track.get_speed_at(i);
        yaxis_omega.push_back(omega);
        yaxis_speed.push_back(speed);
        yaxis_left_wheel.push_back(speed + theMouse->m_radius * RADIANS(omega));
        yaxis_right_wheel.push_back(speed - theMouse->m_radius * RADIANS(omega));
    }

    ui->customPlot->graph(0)->setData(xaxis, yaxis_omega);
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue));

    ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
    ui->customPlot->graph(1)->setData(xaxis, yaxis_speed);

    ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
    ui->customPlot->graph(2)->setData(xaxis, yaxis_left_wheel);
    ui->customPlot->graph(2)->setPen(QPen(Qt::darkCyan));

    ui->customPlot->addGraph(ui->customPlot->xAxis, ui->customPlot->yAxis2);
    ui->customPlot->graph(3)->setData(xaxis, yaxis_right_wheel);
    ui->customPlot->graph(3)->setPen(QPen(Qt::darkCyan));

    //  ui->customPlot->graph(1)->setPen(QPen(Qt::red));
    QPen speedPen;
    speedPen.setColor(Qt::darkMagenta);
    speedPen.setStyle(Qt::DotLine);
    speedPen.setWidthF(2);
    ui->customPlot->graph(1)->setPen(speedPen);
    // give the axes some labels:
    ui->customPlot->xAxis->setLabel("Time (s)");
    ui->customPlot->yAxis->setLabel("Angular Velocity (deg/s)");
    ui->customPlot->yAxis->setLabelColor(Qt::blue);
    // set axes ranges, so we see all data:
    ui->customPlot->xAxis->setRange(0, 0.5);
    //  ui->customPlot->xAxis->setAutoTickStep(false);
    //  ui->customPlot->xAxis->setTickStep(0.1);

    ui->customPlot->yAxis->setRange(0, 1600);
    ui->customPlot->yAxis->setVisible(true);
    //  ui->customPlot->yAxis->setAutoTickStep(false);
    //  ui->customPlot->yAxis->setTickStep(200);

    ui->customPlot->yAxis2->setRange(0, 3200);
    ui->customPlot->yAxis2->setVisible(true);
    //  ui->customPlot->yAxis2->setAutoTickStep(false);
    //  ui->customPlot->yAxis2->setTickStep(400);

    ui->customPlot->yAxis2->setLabel("Tangential Velocity (mm/s)");
    ui->customPlot->yAxis2->setLabelColor(Qt::darkMagenta);
    ui->customPlot->replot();
}

/**
 * @brief Replace the breadcrumb markers with ones for the current track.
 *
 * Markers (every 5th record, coloured by phase) belong to one group so they
 * can be removed together. The group is above the mouse, as before.
 */
void MainWindow::drawBreadcrumbs() {
    delete breadcrumbs;  // also deletes the markers
    breadcrumbs = new QGraphicsItemGroup;
    breadcrumbs->setZValue(1);
    mouseScene->addItem(breadcrumbs);
    QPen markerPen;
    markerPen.setWidthF(2.0);
    markerPen.setStyle(Qt::DotLine);
    for (int i = 0; i < g_track.record_count(); i += 5) {
        switch (g_track.get_phase_at(i)) {
            case 0:
                markerPen.setColor(Qt::blue);
                break;
            case 1:
                markerPen.setColor(Qt::green);
                break;
            case 2:
                markerPen.setColor(Qt::red);
                break;
            case 3:
                markerPen.setColor(Qt::yellow);
                break;
            case 4:
                markerPen.setColor(Qt::cyan);
                break;
        }
        Pose pose = g_track.get_pose_at(i);// draw a breadcrumb
        mouseScene->addEllipse(pose.x(), pose.y(), 1, 1, markerPen)->setParentItem(breadcrumbs);
    }
}

void MainWindow::on_btnReset_clicked() {
    params() = profileDefaults(turnProfile)[turn];
    setParams(turn);
    rePaint();
}

/**
 * @brief Add the fixed maze walls, posts and grid lines to the scene.
 *
 * Called once from the constructor; these items never change.
 */
void MainWindow::drawMazeWalls() {
    QPen wallPen(QColor(255, 0, 0, 90));
    QPen postPen(QColor(255, 0, 0, 180));
    QPen linePen(QColor(255, 0, 0, 90));
    QPen originPen(QColor(255, 200, 0, 90));
    wallPen.setWidthF(12.0);
    wallPen.setJoinStyle(Qt::MiterJoin);
    wallPen.setStyle(Qt::SolidLine);
    postPen.setWidthF(12.0);
    postPen.setJoinStyle(Qt::MiterJoin);
    postPen.setStyle(Qt::SolidLine);
    linePen.setWidthF(2.0);
    linePen.setStyle(Qt::DotLine);
    originPen.setWidthF(2.0);
    originPen.setStyle(Qt::DotLine);

    mouseScene->addLine(-90, -90, -90, 270, wallPen); // left wall
    mouseScene->addLine(-90, -90, 90, -90, wallPen); // top left wall
    mouseScene->addLine(-90, 270, 270, 270, wallPen); // bottom wall
    mouseScene->addLine(90, 90, 90, 270, wallPen); // inner wall

    mouseScene->addLine(270, 90, 270, 91, postPen); // just a post on the right
    mouseScene->addLine(269, -90, 270, -90, postPen); // top right post
    mouseScene->addLine(90, 90, 91, 90, postPen); // center post
    mouseScene->addLine(90, -90, 91, -90, postPen); // top center post

    for (int i = 0; i < 5; i++) {
        mouseScene->addLine(-90 + 90 * i, -90, -90 + 90 * i, 270, linePen);
        mouseScene->addLine(-90, -90 + 90 * i, 270, -90 + 90 * i, linePen);
    }
    // overlay marker lines through the origin
    mouseScene->addLine(-90, 0, 270, 0, originPen);
    mouseScene->addLine(0, -90, 0, 270, originPen);

    mouseScene->addLine(-90, 180, 0, 270, linePen);
    mouseScene->addLine(-90, 90, 90, 270, linePen);
    mouseScene->addLine(-90, 0, 180, 270, linePen);
    mouseScene->addLine(-90, -90, 270, 270, linePen);
    mouseScene->addLine(0, -90, 270, 180, linePen);
    mouseScene->addLine(90, -90, 270, 90, linePen);
    mouseScene->addLine(180, -90, 270, 0, linePen);
    mouseScene->addLine(0, -90, -90, 0, linePen);
    mouseScene->addLine(90, -90, -90, 90, linePen);
    mouseScene->addLine(180, -90, -90, 180, linePen);
    mouseScene->addLine(270, -90, -90, 270, linePen);
    mouseScene->addLine(270, 0, 0, 270, linePen);
    mouseScene->addLine(270, 90, 90, 270, linePen);
    mouseScene->addLine(270, 180, 180, 270, linePen);

    // indicate the origin
    mouseScene->addLine(-5, 0, 5, 0);
    mouseScene->addLine(0, -5, 0, 5);
    mouseScene->addEllipse(-5, -5, 10, 10);
}

void MainWindow::closeEvent(QCloseEvent *event) {
    event->accept();
}

void MainWindow::on_progressSlider_valueChanged(int value) {
    (void) value;
    showProgress();
}

void MainWindow::on_radiusSpinBox_valueChanged(int arg1) {
    float turnRadius = ui->radiusSpinBox->value();
    params().radius = turnRadius;
    updateAccelerationBox();
    rePaint();
}

void MainWindow::on_accelerationSpinBox_valueChanged(int arg1) {
    float turnAcceleration = arg1;
    float turnSpeed = speedForAcceleration(turnAcceleration);
    params().speed = turnSpeed;
    ui->turnSpeedSpinBox->blockSignals(true);
    ui->turnSpeedSpinBox->setValue(qRound(turnSpeed));
    ui->turnSpeedSpinBox->blockSignals(false);
    rePaint();
}

void MainWindow::on_startYSpinBox_valueChanged(int arg1) {
    (void) arg1;
    rePaint();
}

void MainWindow::on_startXSpinBox_valueChanged(int arg1) {
    (void) arg1;
    rePaint();
}

void MainWindow::on_lambdaSpinBox_valueChanged(int arg1) {
    params().lambda = arg1;
    rePaint();
}

void MainWindow::on_cubicLengthSpinBox_valueChanged(int arg1) {
    params().length = arg1;
    updateAccelerationBox();
    rePaint();
}

void MainWindow::on_cubicGammaSpinBox_valueChanged(int arg1) {
    (void) arg1;
    rePaint();
}








void MainWindow::on_turnSpeedSpinBox_valueChanged(int arg1) {
    float turnSpeed = arg1;
    params().speed = turnSpeed;
    updateAccelerationBox();
    rePaint();
}

void MainWindow::on_offsetSpinBox_valueChanged(int arg1) {
    params().offs = arg1;
    updateStartPosition();
    rePaint();
}

/**
 * @brief Set the start X/Y boxes from the turn base point, start angle and offset.
 *
 * The start point is the offset distance back from the base point along the
 * start heading. Signals are blocked; the caller repaints.
 */
void MainWindow::updateStartPosition() {
    float offset = ui->offsetSpinBox->value();
    float start_x = params().baseX - offset * cos(PI / 2 + RADIANS(params().startAngle));
    float start_y = params().baseY - offset * sin(PI / 2 + RADIANS(params().startAngle));
    ui->startXSpinBox->blockSignals(true);
    ui->startXSpinBox->setValue(int(start_x));
    ui->startXSpinBox->blockSignals(false);
    ui->startYSpinBox->blockSignals(true);
    ui->startYSpinBox->setValue(int(start_y));
    ui->startYSpinBox->blockSignals(false);
}

void MainWindow::on_sensorAngleSpinBox_valueChanged(int arg1) {
    theMouse->setSensorAngle(arg1);
    rePaint();
}

/**
 * @brief Load the stored parameters for a turn into the spinboxes.
 *
 * Signals are blocked while loading so the handlers do not repaint once per
 * value; their side effects (acceleration and start position) are done once
 * at the end. Does not repaint: the caller calls rePaint() once.
 *
 * @param turn_index turn type (Turn_t) in the current profile
 */
void MainWindow::setParams(int turn_index) {
    const TurnParams &p = profileParams[turnProfile][turn_index];
    QSpinBox *boxes[] = {ui->radiusSpinBox, ui->lambdaSpinBox, ui->offsetSpinBox,
                         ui->turnSpeedSpinBox, ui->cubicLengthSpinBox};
    for (QSpinBox *box : boxes) {
        box->blockSignals(true);
    }
    ui->radiusSpinBox->setValue(int(p.radius));
    ui->lambdaSpinBox->setValue(int(p.lambda));
    ui->offsetSpinBox->setValue(int(p.offs));
    ui->turnSpeedSpinBox->setValue(int(p.speed));
    ui->cubicLengthSpinBox->setValue(int(p.length));
    for (QSpinBox *box : boxes) {
        box->blockSignals(false);
    }
    updateAccelerationBox();
    updateStartPosition();
}

/**
 * @brief Peak centripetal acceleration of the current turn, see peak_acceleration().
 * @param speed turn speed in mm/s
 * @return peak centripetal acceleration in mm/s/s
 */
float MainWindow::peakAcceleration(float speed) const {
    return peak_acceleration(Profile_t(turnProfile), speed, ui->radiusSpinBox->value(),
                             ui->cubicLengthSpinBox->value(), profileParams[turnProfile][turn].angle);
}

/**
 * @brief Turn speed for a peak centripetal acceleration, see speed_for_acceleration().
 * @param acceleration peak centripetal acceleration in mm/s/s
 * @return turn speed in mm/s
 */
float MainWindow::speedForAcceleration(float acceleration) const {
    return speed_for_acceleration(Profile_t(turnProfile), acceleration, ui->radiusSpinBox->value(),
                                  ui->cubicLengthSpinBox->value(), profileParams[turnProfile][turn].angle);
}

/**
 * @brief Show peakAcceleration() for the current speed in the acceleration box.
 *
 * Signals are blocked so the speed is not recalculated from the rounded value.
 */
void MainWindow::updateAccelerationBox() {
    ui->accelerationSpinBox->blockSignals(true);
    ui->accelerationSpinBox->setValue(qRound(peakAcceleration(ui->turnSpeedSpinBox->value())));
    ui->accelerationSpinBox->blockSignals(false);
}

/**
 * @brief Largest valid lambda for the current turn, see max_lambda().
 * @return maximum lambda in mm
 */
float MainWindow::maxLambda() const {
    return max_lambda(Profile_t(turnProfile), ui->radiusSpinBox->value(), profileParams[turnProfile][turn].angle);
}

/**
 * @brief Collect the simulation inputs from the UI and the current turn parameters.
 *
 * Start Y and the start angle are negated into the simulation convention.
 *
 * @return inputs for simulate_turn()
 */
TurnInput MainWindow::turnInput() const {
    TurnInput in;
    in.start_x = ui->startXSpinBox->value();
    in.start_y = -ui->startYSpinBox->value();
    in.start_angle = -profileParams[turnProfile][turn].startAngle;
    in.turn_angle = profileParams[turnProfile][turn].angle;
    in.speed = ui->turnSpeedSpinBox->value();
    in.radius = ui->radiusSpinBox->value();
    in.lambda = ui->lambdaSpinBox->value();
    in.length = ui->cubicLengthSpinBox->value();
    in.gamma = ui->cubicGammaSpinBox->value() / 100.0f;
    in.interval = loopInterval;
    return in;
}

/**
 * @brief Reduce lambda to maxLambda() if it is too large.
 *
 * Updates the spinbox (without signals) and the stored parameter.
 */
void MainWindow::clampLambda() {
    int limit = int(maxLambda());
    if (ui->lambdaSpinBox->value() > limit) {
        ui->lambdaSpinBox->blockSignals(true);
        ui->lambdaSpinBox->setValue(limit);
        ui->lambdaSpinBox->blockSignals(false);
        params().lambda = limit;
    }
}

/**
 * @brief Switch to a turn profile and load its stored parameters for the current turn.
 *
 * Enables the spinboxes the profile uses. The progress slider is left where it is.
 *
 * @param profile profile index (Profile_t), the profileGroup button id
 */
void MainWindow::selectProfile(int profile) {
    turnProfile = profile;
    setParams(turn);
    bool cubic = (profile == PR_CUBIC);
    ui->lambdaSpinBox->setEnabled(!cubic);
    ui->radiusSpinBox->setEnabled(!cubic);
    ui->cubicLengthSpinBox->setEnabled(cubic);
    ui->cubicGammaSpinBox->setEnabled(cubic);
    rePaint();
}

/**
 * @brief Switch to a turn type, load its parameters and rewind the progress slider.
 *
 * @param turn_index turn type (Turn_t), the turnGroup button id
 */
void MainWindow::selectTurn(int turn_index) {
    turn = Turn_t(turn_index);
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}
