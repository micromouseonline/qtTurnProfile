#include "mainwindow.h"
#include <QButtonGroup>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGraphicsView>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <cmath>
#include <algorithm>
#include "main_window_ui.h"
#include "angles.h"
#include "track.h"
#include "robotstate.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new MainWindowUi) {
    ui->setupUi(this);
    connect(ui->btnAutoScale, &QPushButton::toggled, this, &MainWindow::drawProfileGraph);
    connect(ui->btnReset, &QPushButton::clicked, this, &MainWindow::resetTurn);
    connect(ui->startXSpinBox, &QSpinBox::valueChanged, this, &MainWindow::rePaint);
    connect(ui->startYSpinBox, &QSpinBox::valueChanged, this, &MainWindow::rePaint);
    connect(ui->progressSlider, &QSlider::valueChanged, this, &MainWindow::showProgress);
    connect(ui->accelerationSpinBox, &QSpinBox::valueChanged, this, &MainWindow::accelerationChanged);
    connect(ui->turnSpeedSpinBox, &QSpinBox::valueChanged, this, &MainWindow::speedChanged);
    connect(ui->offsetSpinBox, &QSpinBox::valueChanged, this, &MainWindow::offsetChanged);
    connect(ui->sensorAngleSpinBox, &QSpinBox::valueChanged, this, &MainWindow::sensorAngleChanged);

    // every profile starts from its own defaults
    stored.resize(profile_registry().size());
    for (size_t p = 0; p < stored.size(); p++) {
        for (int t = 0; t < TURN_TYPE_COUNT; t++) {
            TurnDefaults d = profile_registry()[p]->defaults(Turn_t(t));
            stored[p][t] = {d.offset, d.speed, d.values};
        }
    }

    theMouse = new MouseItem;
    theMouse->setSensorAngle(ui->sensorAngleSpinBox->value());
    mouseScene = new QGraphicsScene(this);  // deleted with the window; owns its items
    mouseScene->setSceneRect(-100, -100, 380, 380);
    mouseScene->setBackgroundBrush(QBrush(Qt::black, Qt::SolidPattern));
    drawMazeWalls();
    mouseScene->addItem(theMouse);
    ui->pathView->setScene(mouseScene);
    ui->pathView->setRenderHint(QPainter::Antialiasing);
    buildProfileButtons();

    // button ids are the Turn_t values

    turnGroup = new QButtonGroup(this);
    for (int t = 0; t < TURN_TYPE_COUNT; t++) {
        turnGroup->addButton(ui->turnButtons[t], t);
    }
    connect(turnGroup, &QButtonGroup::idClicked, this, &MainWindow::selectTurn);

    profileGroup->button(0)->click();
    ui->turnButtons[TURN_SS90F]->click();
    setWindowTitle(tr("Turn Render"));
    // the window fits its contents and cannot be resized
    layout()->setSizeConstraint(QLayout::SetFixedSize);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::showTurnSummary() const {
    float max_alpha = track.get_max_alpha() / loopInterval;
    qreal wheelAcceleration = theMouse->m_radius * radians(max_alpha);
    Pose exit_pose = track.get_exit_pose();

    ui->textEdit->clear();
    ui->textEdit->append("Turn Parameters");
    // equals the arc radius for segmented profiles; the tightest point for others
    float speed = ui->turnSpeedSpinBox->value();
    float minRadius = speed * speed / peakAcceleration(speed);
    ui->textEdit->append("   Min Radius: " + QString::number(minRadius, 'f', 0).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("   Turn Speed: " + QString::number(ui->turnSpeedSpinBox->value(), 'f', 0).rightJustified(6, ' ') + " mm/s");
    ui->textEdit->append("Cent'l Acceln: " + QString::number(track.get_max_acceleration(), 'f', 0).rightJustified(6, ' ') + " mm/s/s");
    ui->textEdit->append(" Wheel Acceln: " + QString::number(wheelAcceleration, 'f', 0).rightJustified(6, ' ') + " mm/s/s");
    ui->textEdit->append("        Alpha: " + QString::number(max_alpha, 'f', 0).rightJustified(6, ' ') + " deg/s/s");
    ui->textEdit->append("    Omega Max: " + QString::number(track.get_max_omega(), 'f', 0).rightJustified(6, ' ') + " deg/s");
    ui->textEdit->append("       Exit X: " + QString::number(exit_pose.x(), 'f', 1).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("       Exit Y: " + QString::number(exit_pose.y(), 'f', 1).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("        Theta: " + QString::number(exit_pose.theta(), 'f', 1).rightJustified(6, ' ') + " deg");
    ui->textEdit->append("     Distance: " + QString::number(track.turn_length(), 'f', 0).rightJustified(6, ' ') + " mm");
    ui->textEdit->append("         Time: " + QString::number(track.get_turn_time(), 'f', 3).rightJustified(6, ' ') + " s");
    ui->textEdit->append("   Exit Speed: " + QString::number(track.get_exit_speed(), 'f', 0).rightJustified(6, ' ') + " mm/s");
    for (const SummaryLine &line : profile().summary(turnSetup(), params().values)) {
        ui->textEdit->append(QString::fromStdString(line.label + ":").rightJustified(14, ' ') + " " +
                             QString::number(line.value, 'f', line.decimals).rightJustified(6, ' ') + " " +
                             QString::fromStdString(line.units));
    }
}

void MainWindow::rePaint() {
    constrainParams();
    profile().simulate(turnSetup(), ui->turnSpeedSpinBox->value(), params().values, track);
    showTurnSummary();
    ui->progressSlider->setMinimum(0);
    ui->progressSlider->setMaximum(track.finish_index());
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
    Pose pose = track.get_pose_at(progress);
    theMouse->setPose(pose);
    theMouse->setVelocity(track.get_speed_at(progress));
    ui->lblPathView->setText(
            QString::number(track.record_count(), 'f', 0).rightJustified(4, ' ') + "  " +
            QString::number(track.get_distance_at(progress), 'f', 0).rightJustified(3, ' ') + "mm " +
            "(" + QString::number(pose.x(), 'f', 0).rightJustified(3, ' ') +
            "," + QString::number(pose.y(), 'f', 0).rightJustified(3, ' ') +
            ") ang:" + QString::number(pose.theta(), 'f', 0).rightJustified(3, ' ') + "deg, " +
            "omega: " + QString::number(track.get_omega_at(progress), 'f', 0).rightJustified(4, ' ') + " deg/s");
}

namespace {

/**
 * @brief Round up to a tidy axis limit: the next multiple of half a power of ten.
 * @param value positive value to cover
 * @return smallest multiple of 10^floor(log10(value)) / 2 that is >= value
 */
double niceCeil(double value) {
    if (value <= 0) {
        return 1;
    }
    double step = std::pow(10.0, std::floor(std::log10(value))) / 2;
    return std::ceil(value / step) * step;
}

}  // namespace

/**
 * @brief Plot omega, speed and wheel speeds against time for the current track.
 *
 * Axes are fixed (0-0.5 s, 0-1600 deg/s, 0-3200 mm/s) so changes of speed
 * show against a constant scale; an axis is extended only if the data would
 * be clipped, e.g. for slow turns. With Auto Scale checked, the axes are
 * fitted to the data instead (from zero, with a little headroom).
 */
void MainWindow::drawProfileGraph() const {
    // create graph and assign data to it:
    ui->customPlot->clearGraphs();
    ui->customPlot->addGraph();
    QVector<qreal> xaxis;
    QVector<qreal> yaxis_omega;
    QVector<qreal> yaxis_speed;
    QVector<qreal> yaxis_left_wheel;
    QVector<qreal> yaxis_right_wheel;
    double maxOmega = 0;
    double maxSpeed = 0;
    for (int i = 1; i < track.record_count(); i++) {
        xaxis.push_back(track.get_time_at(i));
        float omega = track.get_omega_at(i);
        float speed = track.get_speed_at(i);
        yaxis_omega.push_back(omega);
        yaxis_speed.push_back(speed);
        yaxis_left_wheel.push_back(speed + theMouse->m_radius * radians(omega));
        yaxis_right_wheel.push_back(speed - theMouse->m_radius * radians(omega));
        maxOmega = std::max(maxOmega, double(omega));
        maxSpeed = std::max({maxSpeed, double(speed), yaxis_left_wheel.back(), yaxis_right_wheel.back()});
    }
    double maxTime = xaxis.isEmpty() ? 0 : xaxis.back();

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
    if (ui->btnAutoScale->isChecked()) {
        const double headroom = 1.05;
        ui->customPlot->xAxis->setRange(0, maxTime > 0 ? maxTime * headroom : 0.5);
        ui->customPlot->yAxis->setRange(0, maxOmega > 0 ? maxOmega * headroom : 1600);
        ui->customPlot->yAxis2->setRange(0, maxSpeed > 0 ? maxSpeed * headroom : 3200);
    } else {
        ui->customPlot->xAxis->setRange(0, maxTime > 0.5 ? niceCeil(maxTime) : 0.5);
        ui->customPlot->yAxis->setRange(0, maxOmega > 1600 ? niceCeil(maxOmega) : 1600);
        ui->customPlot->yAxis2->setRange(0, maxSpeed > 3200 ? niceCeil(maxSpeed) : 3200);
    }
    ui->customPlot->yAxis->setVisible(true);
    ui->customPlot->yAxis2->setVisible(true);

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
    for (int i = 0; i < track.record_count(); i += 5) {
        switch (track.get_phase_at(i)) {
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
        Pose pose = track.get_pose_at(i);// draw a breadcrumb
        mouseScene->addEllipse(pose.x(), pose.y(), 1, 1, markerPen)->setParentItem(breadcrumbs);
    }
}

/// Restore the profile defaults for the current turn and repaint
void MainWindow::resetTurn() {
    TurnDefaults d = profile().defaults(turn);
    params() = {d.offset, d.speed, d.values};
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

/**
 * @brief The acceleration box changed: derive the speed from it and repaint.
 * @param acceleration peak centripetal acceleration, mm/s/s
 */
void MainWindow::accelerationChanged(int acceleration) {
    float turnAcceleration = acceleration;
    float turnSpeed = speedForAcceleration(turnAcceleration);
    params().speed = turnSpeed;
    ui->turnSpeedSpinBox->blockSignals(true);
    ui->turnSpeedSpinBox->setValue(qRound(turnSpeed));
    ui->turnSpeedSpinBox->blockSignals(false);
    rePaint();
}

/**
 * @brief The speed box changed: store it, update the acceleration box and repaint.
 * @param speed turn speed, mm/s
 */
void MainWindow::speedChanged(int speed) {
    float turnSpeed = speed;
    params().speed = turnSpeed;
    updateAccelerationBox();
    rePaint();
}

/**
 * @brief The offset box changed: store it, move the start position and repaint.
 * @param offset start offset back from the turn base point, mm
 */
void MainWindow::offsetChanged(int offset) {
    params().offset = offset;
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
    const TurnGeometry &g = turnGeometry[turn];
    float offset = ui->offsetSpinBox->value();
    float start_x = g.base_x - offset * cos(PI / 2 + radians(g.start_angle));
    float start_y = g.base_y - offset * sin(PI / 2 + radians(g.start_angle));
    ui->startXSpinBox->blockSignals(true);
    ui->startXSpinBox->setValue(int(start_x));
    ui->startXSpinBox->blockSignals(false);
    ui->startYSpinBox->blockSignals(true);
    ui->startYSpinBox->setValue(int(start_y));
    ui->startYSpinBox->blockSignals(false);
}

/**
 * @brief The sensor angle box changed: redraw the mouse sensors.
 * @param angle sensor angle, degrees
 */
void MainWindow::sensorAngleChanged(int angle) {
    theMouse->setSensorAngle(angle);
    rePaint();
}

/**
 * @brief Load the stored parameters for a turn into the spinboxes.
 *
 * Signals are blocked while loading so the handlers do not repaint once per
 * value; their side effects (acceleration and start position) are done once
 * at the end. Does not repaint: the caller calls rePaint() once.
 *
 * @param turn_index turn type (Turn_t) in the current profile; the parameter
 *        spinboxes must already be built for the current profile
 */
void MainWindow::setParams(int turn_index) {
    const TurnValues &v = stored[turnProfile][turn_index];
    std::vector<QAbstractSpinBox *> boxes = {ui->offsetSpinBox, ui->turnSpeedSpinBox};
    boxes.insert(boxes.end(), paramBoxes.begin(), paramBoxes.end());
    for (QAbstractSpinBox *box : boxes) {
        box->blockSignals(true);
    }
    ui->offsetSpinBox->setValue(int(v.offset));
    ui->turnSpeedSpinBox->setValue(int(v.speed));
    for (size_t i = 0; i < paramBoxes.size(); i++) {
        paramBoxes[i]->setValue(v.values[i]);
    }
    for (QAbstractSpinBox *box : boxes) {
        box->blockSignals(false);
    }
    updateAccelerationBox();
    updateStartPosition();
}

/**
 * @brief Peak centripetal acceleration of the current turn, see TurnProfile::peak_acceleration().
 * @param speed turn speed in mm/s
 * @return peak centripetal acceleration in mm/s/s
 */
float MainWindow::peakAcceleration(float speed) const {
    return profile().peak_acceleration(turnSetup(), speed, params().values);
}

/**
 * @brief Turn speed for a peak centripetal acceleration, see TurnProfile::speed_for_acceleration().
 * @param acceleration peak centripetal acceleration in mm/s/s
 * @return turn speed in mm/s
 */
float MainWindow::speedForAcceleration(float acceleration) const {
    return profile().speed_for_acceleration(turnSetup(), acceleration, params().values);
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
 * @brief Collect the simulation setup from the UI and the turn geometry.
 *
 * Start Y and the start angle are negated into the simulation convention.
 *
 * @return setup for TurnProfile::simulate()
 */
TurnSetup MainWindow::turnSetup() const {
    TurnSetup setup;
    setup.start_x = ui->startXSpinBox->value();
    setup.start_y = -ui->startYSpinBox->value();
    setup.start_angle = -turnGeometry[turn].start_angle;
    setup.turn_angle = turnGeometry[turn].angle;
    setup.interval = loopInterval;
    return setup;
}

/**
 * @brief Apply the profile's constraints (e.g. lambda limit) to the current values.
 *
 * Changed values are stored and shown in their spinboxes without signals.
 */
void MainWindow::constrainParams() {
    std::vector<float> values = params().values;
    if (!profile().constrain(turnSetup(), values)) {
        return;
    }
    params().values = values;
    for (size_t i = 0; i < paramBoxes.size(); i++) {
        paramBoxes[i]->blockSignals(true);
        paramBoxes[i]->setValue(values[i]);
        paramBoxes[i]->blockSignals(false);
    }
}

/**
 * @brief Create one radio button per registered profile in the Turn Profile box.
 *
 * Button ids are registry indices.
 */
void MainWindow::buildProfileButtons() {
    profileGroup = new QButtonGroup(this);
    const auto &profiles = profile_registry();
    for (size_t i = 0; i < profiles.size(); i++) {
        auto *button = new QRadioButton(QString::fromStdString(profiles[i]->name()), ui->gbTurnProfile);
        button->setToolTip(QString::fromStdString(profiles[i]->description()));
        ui->profileLayout->addWidget(button);
        profileGroup->addButton(button, int(i));
    }
    connect(profileGroup, &QButtonGroup::idClicked, this, &MainWindow::selectProfile);
}

/**
 * @brief Replace the parameter rows with spinboxes for the current profile's parameters.
 *
 * Rows go after the speed and acceleration rows. Each spinbox stores its
 * value in params().values and repaints when changed.
 */
void MainWindow::buildParamRows() {
    for (QDoubleSpinBox *box : paramBoxes) {
        ui->paramLayout->removeRow(box);  // deletes the label too
    }
    paramBoxes.clear();
    const int firstRow = 2;  // after speed and acceleration
    std::vector<ParamDef> defs = profile().params();
    for (size_t i = 0; i < defs.size(); i++) {
        const ParamDef &def = defs[i];
        auto *box = new QDoubleSpinBox;
        box->setObjectName(QString::fromStdString("param_" + def.id));
        box->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        box->setAccelerated(true);
        box->setDecimals(def.decimals);
        box->setRange(def.min, def.max);
        box->setSingleStep(def.step);
        box->setToolTip(QString::fromStdString(def.label + " (" + def.units + ")"));
        connect(box, &QDoubleSpinBox::valueChanged, this, [this, i](double value) { paramChanged(int(i), value); });
        ui->paramLayout->insertRow(firstRow + int(i), QString::fromStdString(def.label + ":"), box);
        paramBoxes.push_back(box);
    }
}

/**
 * @brief A profile parameter spinbox changed: store it and repaint.
 * @param index parameter index in TurnProfile::params()
 * @param value new value
 */
void MainWindow::paramChanged(int index, double value) {
    params().values[index] = value;
    updateAccelerationBox();
    rePaint();
}

/**
 * @brief Switch to a turn profile and load its stored parameters for the current turn.
 *
 * Builds the profile's parameter spinboxes. The progress slider is left where it is.
 *
 * @param profile_index index in profile_registry(), the profileGroup button id
 */
void MainWindow::selectProfile(int profile_index) {
    turnProfile = profile_index;
    buildParamRows();
    setParams(turn);
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
