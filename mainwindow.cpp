#include "mainwindow.h"
#include <cmath>
#include "algorithm"
#include "ui_mainwindow.h"
#include "track.h"
#include "pathitem.h"
#include "robotstate.h"

#define PI 3.14159265359
#define RADIANS(x) ((x)*PI / 180.0)
#define DEGREES(x) ((x)*180.0 / PI)

TurnParams defaultParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 112, .startAngle = 0, .angle = 90, .speed = 2200, .length = 248},    // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 85, .startAngle = 0, .angle = 180, .speed = 2000, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 65, .radius = 110, .startAngle = 0, .angle = 45, .speed = 2200, .length = 134},     // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 83, .startAngle = 0, .angle = 135, .speed = 1800, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 120, .startAngle = -45, .angle = 45, .speed = 2200, .length = 144},   // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 80, .startAngle = -45, .angle = 135, .speed = 1800, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 74, .startAngle = -45, .angle = 90, .speed = 1800, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 56, .startAngle = 0, .angle = 90, .speed = 1000, .length = 124},       // SS90E
};

TurnParams trapezoidParams[TURN_TYPE_COUNT] = {
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 100, .radius = 96, .startAngle = 0, .angle = 90, .speed = 2200, .length = 248},     // SS90F
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 127, .radius = 82, .startAngle = 0, .angle = 180, .speed = 2000, .length = 370},    // SS180
        {.baseX = 0, .baseY = 0, .offs = 160, .lambda = 65, .radius = 89, .startAngle = 0, .angle = 45, .speed = 2200, .length = 134},      // SD45
        {.baseX = 0, .baseY = 0, .offs = 150, .lambda = 90, .radius = 77, .startAngle = 0, .angle = 135, .speed = 1800, .length = 266},     // SD135
        {.baseX = 90, .baseY = 0, .offs = 75, .lambda = 70, .radius = 95, .startAngle = -45, .angle = 45, .speed = 2200, .length = 144},    // DS45
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 90, .radius = 73, .startAngle = -45, .angle = 135, .speed = 1800, .length = 257},  // DS135
        {.baseX = 90, .baseY = 0, .offs = 105, .lambda = 80, .radius = 61, .startAngle = -45, .angle = 90, .speed = 1800, .length = 173},   // DD90
        {.baseX = 0, .baseY = 0, .offs = 75, .lambda = 50, .radius = 48, .startAngle = 0, .angle = 90, .speed = 1000, .length = 124},       // SS90E
};

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);

    connect(ui->btnReset, SIGNAL(clicked(bool)), this, SLOT(on_btnReset_clicked()));
    // spinboxes
    connect(ui->turnSpeedSpinBox, SIGNAL(valueChanged(int)), this, SLOT(on_turnSpeedSpinBox_valueChanged(int)));

    for (int i = 0; i < TURN_TYPE_COUNT; i++) {
        currentParams[i] = defaultParams[i];
    }

    thePath = new PathItem;
    theMouse = new MouseItem;
    theMouse->setSensorAngle(ui->sensorAngleSpinBox->value());
    mouseScene = new QGraphicsScene;
    mouseScene->setSceneRect(-100, -100, 380, 380);
    mouseScene->setBackgroundBrush(QBrush(Qt::black, Qt::SolidPattern));
    mouseScene->addItem(theMouse);
//    mouseScene->addItem(thePath);
    ui->pathView->setScene(mouseScene);
    ui->pathView->setRenderHint(QPainter::Antialiasing);
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
}

void MainWindow::rePaint() {
    drawMazeWalls();
    float turnAcceleration = ui->accelerationSpinBox->value();
    float start_x = ui->startXSpinBox->value();
    float start_y = -ui->startYSpinBox->value();
    switch (turnProfile) {
        case PR_CUBIC:
            calculateCubic(start_x, start_y);
            break;
        case PR_SINUSOID:
            calculateSine(start_x, start_y);
            break;
        case PR_QUADRATIC:
            calculateQuadratic(start_x, start_y);
            break;
        case PR_TRAPEZOID:
        default:
            calculateTrapezoid(start_x, start_y);
            break;
    }
    showTurnSummary();
    ui->progressSlider->setMinimum(0);
    ui->progressSlider->setMaximum(g_track.finish_index());
    ui->accelerationSpinBox->blockSignals(true);
    ui->accelerationSpinBox->setValue(int(turnAcceleration));
    ui->accelerationSpinBox->blockSignals(false);
    drawBreadcrumbs();
    drawProfileGraph();
    int progress = ui->progressSlider->value();
    Pose pose = g_track.get_pose_at(progress);
    theMouse->setPose(pose);
    ui->lblPathView->setText(
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

void MainWindow::drawBreadcrumbs() const {
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
        mouseScene->addEllipse(pose.x(), pose.y(), 1, 1, markerPen);
    }
}

void MainWindow::on_btnReset_clicked() {
    currentParams[turn] = defaultParams[turn];
    setParams(turn);
    rePaint();
}

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

    mouseScene->removeItem(theMouse);

    mouseScene->clear();
    mouseScene->addLine(-90, -90, -90, 270, wallPen); // left wall
    mouseScene->addLine(-90, -90, 90, -90, wallPen); // top left wall
    mouseScene->addLine(-90, 270, 270, 270, wallPen); // bottom wall
    mouseScene->addLine(90, 90, 90, 270, wallPen); // inner wall

    mouseScene->addLine(270, 90, 270, 91, postPen); // just a post on the right
    mouseScene->addLine(90, 90, 91, 90, postPen); // center post
    mouseScene->addLine(269, -90, 270, -90, postPen); // top right post
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
    mouseScene->addItem(theMouse);

}

void MainWindow::closeEvent(QCloseEvent *event) {
    event->accept();
}

void MainWindow::on_progressSlider_valueChanged(int value) {
    (void) value;
    rePaint();
}

void MainWindow::on_radiusSpinBox_valueChanged(int arg1) {
    float turnRadius = ui->radiusSpinBox->value();
    float turnSpeed = ui->turnSpeedSpinBox->value();
    currentParams[turn].radius = turnRadius;
    float turnAcceleration = turnSpeed * turnSpeed / turnRadius;
    ui->accelerationSpinBox->blockSignals(true);
    ui->accelerationSpinBox->setValue(int(turnAcceleration));
    ui->accelerationSpinBox->blockSignals(false);
    rePaint();
}

void MainWindow::on_accelerationSpinBox_valueChanged(int arg1) {
    float turnAcceleration = arg1;
    float turnSpeed = sqrt(turnAcceleration * ui->radiusSpinBox->value());
    currentParams[turn].speed = turnSpeed;
    ui->turnSpeedSpinBox->blockSignals(true);
    ui->turnSpeedSpinBox->setValue(int(turnSpeed));
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
    currentParams[turn].lambda = arg1;
    rePaint();
}

void MainWindow::on_cubicLengthSpinBox_valueChanged(int arg1) {
    currentParams[turn].length = arg1;
    rePaint();
}

void MainWindow::on_cubicGammaSpinBox_valueChanged(int arg1) {
    (void) arg1;
    rePaint();
}

//enum turnState {
//    PHASE1,PHASE2, PHASE3, LEADOUT, IDLE
//};

void MainWindow::reset_track() {
    g_track.clear();
}

/**
 *
 * The trapezoidal turn has the robot transition from straight-line motion to
 * a constant angular velocity arc by linearly increasing the angular velocity
 * over a fixed distance (lambda). During this portion of the turn, the robot
 * path follows a clothoid curve.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another linear
 * change to angular velocity
 *
 * Four parameters are needed to define the turn:
 *  - turn minimum radius - that is for the constant arc portion, in mm
 *  - total turn angle - in degrees
 *  - lambda - the robot transitions distance in mm
 *  - speed - the robot forward velocity. Constant through the turn
 *
 * Everything else is calculated from that.
 *
 * For this turn type only, setting the lambda to be the same as
 * the mouse radius causes the peak lateral and forward wheel accelerations
 * to be about the same
 */
void MainWindow::calculateTrapezoid(float start_x, float start_y) {
    // get some parameters from the UI
    // these are enough to fully define the turn for the robot
    float turnRadius = (float) ui->radiusSpinBox->value();  // the minimum radius portion
    float turnSpeed = (float) ui->turnSpeedSpinBox->value();// constant throughout the turn
    float turnAngle = (float) currentParams[turn].angle;    // total turn angle in degrees
    float lambda = (float) ui->lambdaSpinBox->value();      // distance of angular acceleration phase

    // this is only needed for the simulator
    float startAngle = -currentParams[turn].startAngle;

    // these are derived values needed to execute the turn
    float endAngle = startAngle + turnAngle;
    float turnOmega = DEGREES(turnSpeed / turnRadius);

    float transitionAngle = lambda * turnOmega / (turnSpeed * 2);
    // angle turned during the constant omega phase
    float theta2 = turnAngle - 2 * transitionAngle;
    float midArcLength = turnSpeed * theta2 / turnOmega;
    float turnDistance = lambda + midArcLength + lambda;

    reset_track();
    RobotState robot_state(start_x, start_y, startAngle, turnSpeed, loopInterval);

    // In the robot, take care that rounding errors do not preventit from
    // exactly reaching the total angle as omega gets small. a residual error of
    // 0.1 degrees is good enough to have the robot no more than 0.3mm off track
    // after 180 mm of forward travel.
    // You can carry residual errors through to the subsequent straight for
    // correction if desired.
    while (robot_state.theta < endAngle - 0.1) {
        if (robot_state.theta < startAngle + transitionAngle) {
            robot_state.omega = turnOmega * robot_state.distance / lambda;
            robot_state.phase = 1;
        } else if (robot_state.theta < (endAngle - transitionAngle)) {
            robot_state.omega = turnOmega;
            robot_state.phase = 2;
        } else {
            robot_state.omega = turnOmega * ((turnDistance - robot_state.distance) / lambda);
            robot_state.phase = 3;
        }
        robot_state.update();
        g_track.add_record(robot_state);
    }
    g_track.set_turn_end();
    calculate_leadout(robot_state);
}


/***
 *
 * CUBIC PROFILE
 *
 * The cubic profile is completely different to the other types. It is not
 * executed in phases and the robot describes a completely smooth path from
 * start to finish.
 *
 * In the simulation, Dalpha is not needed because we read the turn length
 * directly from the UI.
 *
 * If there is only the start and end points (x1,y1) and (x2,y2) respectively,
 * then the length is calculated from those. Let d be the euclidean distance
 * between the points:
 *
 *  distance =  sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))
 *
 * then
 *
 * length = distance / dalpha(turnAngle)
 *
 * where turnAngle is in radians, positive is anti-clockwise
 *
 * the performance metrics can be calculated from the curvature equations
 *    turnDistance = length;
 *    turnOmega = (turnSpeed * k * length * length / 4);
 *    turnRadius = 4 * length / 6 / RADIANS(turnAngle);
 *    turnAcceleration = turnSpeed * turnSpeed * 6 * RADIANS(turnAngle) / 4 / length;
 *    turnAlpha = turnSpeed * k * length;
 *
 * This version has a parameter, gamma, that allows the mouse to
 * enter the turn more quickly - braking into and accelerating
 * out of he turn. the calculation retains the peak centripetal
 * acceleration at the given value. While the technique has promise,
 * it relies on the inner wheel being able to brake much harder
 * than the standard turn.
 *
 * Setting gamma at zero maintains a constant speed throughout.
 ***/

void MainWindow::calculateCubic(float start_x, float start_y) {
    float turnSpeed = ui->turnSpeedSpinBox->value();
    float turnAngle = currentParams[turn].angle;
    float startAngle = -currentParams[turn].startAngle;
    float endAngle = startAngle + turnAngle;

    float length = ui->cubicLengthSpinBox->value();
    float gamma = ui->cubicGammaSpinBox->value() / 100.0f;
    // k is the constant used to obtain the curvature of the path
    float k = 6.0f * (turnAngle) / (length * length * length);

    // prepare the local data
    reset_track();
    RobotState robot_state(start_x, start_y, startAngle, turnSpeed, loopInterval);
    g_track.add_record(robot_state);
    // perform the turn;
    // unlike the other turn types, the cubic spiral is based on distance
    // rather than the turn angle.
    // TODO: is this a problem if there is wheel slip?
    while (robot_state.distance < length) {
        // calcualate the speed adjustment if used
        float frac = robot_state.distance / length;
        float q = 4.0 * gamma * (frac - 1) * frac + 1 + gamma;
        robot_state.speed = turnSpeed * q;

        float omega = robot_state.speed * k * robot_state.distance * (length - robot_state.distance);
        robot_state.omega = omega;
        if (frac < 0.5) {
            robot_state.phase = 1;
        } else {
            robot_state.phase = 3;
        }
        robot_state.update();
        g_track.add_record(robot_state);
    }
    g_track.set_turn_end();
    calculate_leadout(robot_state);
}

/**
 *
 * The quadratic turn has three sections, or phases, like the trapezoidal turn.
 *
 * The difference is that the change in angular velocity is calculated as a
 * portion of a quadratic function. Like the trapezoidal version, the acceleration
 * is over a fixed distance.
 *
 * It is not clear that there is any practical advantage to this profile although
 * it does significantly ease the transition into and out of the constant radius
 * phase.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another quadratic
 * change to angular velocity
 *
 * Four parameters are needed to define the turn:
 *  - turn minimum radius - that is for the constant arc portion, in mm
 *  - total turn angle - in degrees
 *  - lambda - the robot transitions distance in mm
 *  - speed - the robot forward velocity. Constant through the turn
 *
 * Everything else is calculated from that.
 *
 * For this turn type only, setting the lambda to be the same as
 * the distance between the wheels causes the peak lateral and forward
 * wheel accelerations to be about the same.
 */
void MainWindow::calculateQuadratic(float start_x, float start_y) {
    // get some parameters from the UI
    // these are enough to fully define the turn for the robot
    float turnRadius = (float) ui->radiusSpinBox->value();  // the minimum radius portion
    float turnSpeed = (float) ui->turnSpeedSpinBox->value();// constant throughout the turn
    float turnAngle = (float) currentParams[turn].angle;    // total turn angle in degrees
    float lambda = (float) ui->lambdaSpinBox->value();      // distance of angular acceleration phase

    // this is only needed for the simulator
    float startAngle = -currentParams[turn].startAngle;

    // these are derived values needed to execute the turn
    float endAngle = startAngle + turnAngle;
    float turnOmega = DEGREES(turnSpeed / turnRadius);

    float transitionAngle = lambda * 2 * turnOmega / (turnSpeed * 3);
    // angle turned during the constant omega phase
    float theta2 = turnAngle - 2 * transitionAngle;
    float midArcLength = turnSpeed * theta2 / turnOmega;
    float turnDistance = lambda + midArcLength + lambda;

    reset_track();
    RobotState robot_state(start_x, start_y, startAngle, turnSpeed, loopInterval);
    g_track.add_record(robot_state);


    // In the robot, take care that rounding errors do not preventit from
    // exactly reaching the total angle as omega gets small. a residual error of
    // 0.1 degrees is good enough to have the robot no more than 0.3mm off track
    // after 180 mm of forward travel.
    // You can carry residual errors through to the subsequent straight for
    // correction if desired.
    while (robot_state.theta < endAngle - 0.01) {
        if (robot_state.theta < startAngle + transitionAngle) {
            float t = (robot_state.distance / lambda);
            robot_state.omega = turnOmega * t * (2.0 - t);
            robot_state.phase = 1;
        } else if (robot_state.theta < (endAngle - transitionAngle)) {
            robot_state.omega = turnOmega;
            robot_state.phase = 2;
        } else {
            float t = ((turnDistance - robot_state.distance) / lambda);
            robot_state.omega = turnOmega * t * (2.0 - t);
            robot_state.phase = 3;
        }
        robot_state.update();
        g_track.add_record(robot_state);
    }
    // remember where the turn finished
    g_track.set_turn_end();
    calculate_leadout(robot_state);
}

/**
 *
 * Sinusoidal profile
 *
 * Like the trapezoidal turn, this turn has tyhree phases. The central constant
 * radius phase is the same but the transitions in and out have the angular
 * velocity follow a portion of a sinusoidal profile.
 *
 * This profile is intended to keep the forces on the tyres within its traction
 * circle at all times and provides a smooth transition as the robot reaches its
 * maximum angular velocity.
 *
 * Some builders specify a time for a reference turn rather than a distance
 * for this accelerating portion. that has some advantages but the time will
 * change as the robot tries to run the turn faster. A little bit of calculation
 * reveals that the distance always comes out the same so it is just easier
 * to specify a distance rather than a time and the resulting turn path
 * is easily made invariant with robot speed.
 *
 * After the constant arc, the robot 'unwinds' the turn with another sinusoidal
 * change to angular velocity
 *
 * Four parameters are needed to define the turn:
 *  - turn minimum radius - that is for the constant arc portion, in mm
 *  - total turn angle - in degrees
 *  - lambda - the robot transitions distance in mm
 *  - speed - the robot forward velocity. Constant through the turn
 *
 * Everything else is calculated from that.
 *
 * For this turn type only, setting the lambda to be the same as
 * the pi times mouse radius causes the peak lateral and forward wheel accelerations
 * to be about the same
 */
void MainWindow::calculateSine(float start_x, float start_y) {
    // get some parameters from the UI
    // these are enough to fully define the turn for the robot
    float turnRadius = (float) ui->radiusSpinBox->value();  // the minimum radius portion
    float turnSpeed = (float) ui->turnSpeedSpinBox->value();// constant throughout the turn
    float turnAngle = (float) currentParams[turn].angle;    // total turn angle in degrees
    float lambda = (float) ui->lambdaSpinBox->value();      // distance of angular acceleration phase

    // this is only needed for the simulator
    float startAngle = -currentParams[turn].startAngle;

    // these are derived values needed to execute the turn
    float endAngle = startAngle + turnAngle;
    float turnOmega = DEGREES(turnSpeed / turnRadius);

    float transitionAngle = 2 * lambda * turnOmega / PI / turnSpeed;
    // angle turned during the constant omega phase
    float theta2 = turnAngle - 2 * transitionAngle;
    float midArcLength = turnSpeed * theta2 / turnOmega;
    float turnDistance = lambda + midArcLength + lambda;

    // prepare the local data
    reset_track();
    RobotState robot_state(start_x, start_y, startAngle, turnSpeed, loopInterval);
    g_track.add_record(robot_state);

    // In the robot, take care that rounding errors do not preventit from
    // exactly reaching the total angle as omega gets small. a residual error of
    // 0.1 degrees is good enough to have the robot no more than 0.3mm off track
    // after 180 mm of forward travel.
    // You can carry residual errors through to the subsequent straight for
    // correction if desired.
    while (robot_state.theta < endAngle - 0.01) {
        if (robot_state.theta < (startAngle + transitionAngle)) {
            robot_state.omega = turnOmega * sin((PI / 2) * robot_state.distance / lambda);
            robot_state.phase = 1;
        } else if (robot_state.theta < (endAngle - transitionAngle)) {
            robot_state.omega = turnOmega;
            robot_state.phase = 2;
        } else {
            robot_state.omega = turnOmega * sin((PI / 2) * (turnDistance - robot_state.distance) / lambda);
            robot_state.phase = 3;
        }
        robot_state.update();
        g_track.add_record(robot_state);
    }
    g_track.set_turn_end();
    calculate_leadout(robot_state);
}

void MainWindow::calculate_leadout(RobotState &robot_state, float distance) {// leadout
    robot_state.omega = 0;
    robot_state.phase = 4;
    qreal targetPos = robot_state.distance + distance;
    while (robot_state.distance < targetPos) {
        robot_state.update();
        g_track.add_record(robot_state);
    }
}


void MainWindow::on_turnSpeedSpinBox_valueChanged(int arg1) {
    float turnSpeed = arg1;
    currentParams[turn].speed = turnSpeed;
    float turnAcceleration = turnSpeed * turnSpeed / ui->radiusSpinBox->value();
    ui->accelerationSpinBox->blockSignals(true);
    ui->accelerationSpinBox->setValue(int(turnAcceleration));
    ui->accelerationSpinBox->blockSignals(false);
    rePaint();
}

void MainWindow::on_offsetSpinBox_valueChanged(int arg1) {
    currentParams[turn].offs = arg1;
    float baseX = currentParams[turn].baseX;
    float baseY = currentParams[turn].baseY;
    float startAngle = currentParams[turn].startAngle;
    float start_x = baseX - double(arg1) * cos(PI / 2 + RADIANS(startAngle));
    float start_y = baseY - double(arg1) * sin(PI / 2 + RADIANS(startAngle));
    ui->startXSpinBox->blockSignals(true);
    ui->startXSpinBox->setValue(int(start_x));
    ui->startXSpinBox->blockSignals(false);
    ui->startYSpinBox->blockSignals(true);
    ui->startYSpinBox->setValue(int(start_y));
    ui->startYSpinBox->blockSignals(false);
    rePaint();
}

void MainWindow::on_rbTrapezoid_clicked() {
    for (int i = 0; i < TURN_TYPE_COUNT; i++) {
        currentParams[i] = trapezoidParams[i];
    }
    int v = ui->turnSpeedSpinBox->value();
    setParams(turn);
    turnProfile = PR_TRAPEZOID;
    ui->turnSpeedSpinBox->setValue(v + 1);  // TODO: why does this have  to happen
    // twice? (also in related functions)
    ui->turnSpeedSpinBox->setValue(v);
    ui->lambdaSpinBox->setEnabled(true);
    ui->radiusSpinBox->setEnabled(true);
    ui->cubicLengthSpinBox->setEnabled(false);
    ui->cubicGammaSpinBox->setEnabled(false);
    rePaint();
}

void MainWindow::on_rbQuadratic_clicked() {
    for (int i = 0; i < TURN_TYPE_COUNT; i++) {
        currentParams[i] = defaultParams[i];
    }
    int v = ui->turnSpeedSpinBox->value();
    setParams(turn);
    turnProfile = PR_QUADRATIC;
    ui->turnSpeedSpinBox->setValue(v + 1);
    ui->turnSpeedSpinBox->setValue(v);
    ui->lambdaSpinBox->setEnabled(true);
    ui->radiusSpinBox->setEnabled(true);
    ui->cubicLengthSpinBox->setEnabled(false);
    ui->cubicGammaSpinBox->setEnabled(false);
    rePaint();
}

void MainWindow::on_rbSinusoid_clicked() {
    for (int i = 0; i < TURN_TYPE_COUNT; i++) {
        currentParams[i] = defaultParams[i];
    }
    int v = ui->turnSpeedSpinBox->value();
    setParams(turn);
    turnProfile = PR_SINUSOID;
    ui->turnSpeedSpinBox->setValue(v + 1);
    ui->turnSpeedSpinBox->setValue(v);
    ui->lambdaSpinBox->setEnabled(true);
    ui->radiusSpinBox->setEnabled(true);
    ui->cubicLengthSpinBox->setEnabled(false);
    ui->cubicGammaSpinBox->setEnabled(false);
    rePaint();
}

void MainWindow::on_rbCubic_clicked() {
    for (int i = 0; i < TURN_TYPE_COUNT; i++) {
        currentParams[i] = defaultParams[i];
    }
    int v = ui->turnSpeedSpinBox->value();
    setParams(turn);
    turnProfile = PR_CUBIC;
    ui->turnSpeedSpinBox->setValue(v + 1);
    ui->turnSpeedSpinBox->setValue(v);
    ui->lambdaSpinBox->setEnabled(false);
    ui->radiusSpinBox->setEnabled(false);
    ui->cubicLengthSpinBox->setEnabled(true);
    ui->cubicGammaSpinBox->setEnabled(true);
    rePaint();
}

void MainWindow::on_sensorAngleSpinBox_valueChanged(int arg1) {
    theMouse->setSensorAngle(arg1);
    rePaint();
}

void MainWindow::setParams(int turn_index) {
    ui->radiusSpinBox->setValue(int(currentParams[turn_index].radius));
    ui->lambdaSpinBox->setValue(int(currentParams[turn_index].lambda));
    ui->offsetSpinBox->setValue(int(currentParams[turn_index].offs));
    //  ui->turnSpeedSpinBox->setValue(int(currentParams[turn].speed));
    ui->cubicLengthSpinBox->setValue(int(currentParams[turn_index].length));
}

void MainWindow::on_rbSS90F_clicked() {
    turn = TURN_SS90F;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbSS180_clicked() {
    turn = TURN_SS180;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbSD45_clicked() {
    turn = TURN_SD45;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbSD135_clicked() {
    turn = TURN_SD135;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbDS45_clicked() {
    turn = TURN_DS45;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbDS135_clicked() {
    turn = TURN_DS135;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbDD90_clicked() {
    turn = TURN_DD90;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}

void MainWindow::on_rbSS90E_clicked() {
    turn = TURN_SS90E;
    setParams(turn);
    ui->progressSlider->setValue(0);
    rePaint();
}
