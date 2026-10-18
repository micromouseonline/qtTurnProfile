/**
 * @file main_window_ui.cpp
 * @brief Builds the main window widgets and layouts.
 */

#include "main_window_ui.h"
#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGraphicsView>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QPushButton>
#include <QRadioButton>
#include <QSlider>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>
#include "qcustomplot.h"

namespace {

/// Translatable text, in the MainWindow context
QString tr(const char *text) {
    return QCoreApplication::translate("MainWindow", text);
}

/// Turn type button labels, indexed by Turn_t
const char *turnLabels[TURN_TYPE_COUNT] = {"SS90F", "SS180", "SD45", "SD135", "DS45", "DS135", "DD90", "SS90E"};

/// Minimum size of the graph and the path view, pixels
const int viewSize = 400;

/**
 * @brief Graphics view that scales its scene rectangle to fit, keeping the aspect ratio.
 *
 * Lets the path view grow with the window while always showing the whole maze.
 */
class FitView : public QGraphicsView {
public:
    FitView() {
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }

protected:
    void resizeEvent(QResizeEvent *event) override {
        QGraphicsView::resizeEvent(event);
        fitInView(sceneRect(), Qt::KeepAspectRatio);
    }
};

/**
 * @brief Make a right aligned integer spinbox.
 *
 * Keyboard tracking is off: a typed value takes effect on Enter, focus loss or
 * the arrows, not on every keystroke. Partly typed values would otherwise be
 * acted on, and e.g. a partial acceleration clamps the speed and rewrites the
 * box while typing.
 * @param name object name
 * @param min minimum value
 * @param max maximum value
 * @param step single step
 * @param value initial value
 */
QSpinBox *makeSpinBox(const char *name, int min, int max, int step, int value) {
    auto *box = new QSpinBox;
    box->setObjectName(name);
    box->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    box->setRange(min, max);
    box->setSingleStep(step);
    box->setValue(value);
    box->setKeyboardTracking(false);
    return box;
}

/**
 * @brief Make a right aligned spinbox with one decimal; keyboard tracking off, as makeSpinBox().
 * @param name object name
 * @param min minimum value
 * @param max maximum value
 * @param value initial value
 */
QDoubleSpinBox *makeDoubleSpinBox(const char *name, double min, double max, double value) {
    auto *box = new QDoubleSpinBox;
    box->setObjectName(name);
    box->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    box->setDecimals(1);
    box->setRange(min, max);
    box->setSingleStep(0.1);
    box->setValue(value);
    box->setKeyboardTracking(false);
    return box;
}

/// Form layout with right aligned labels and fields at their natural width
QFormLayout *makeForm(QWidget *parent) {
    auto *form = new QFormLayout(parent);
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form->setFieldGrowthPolicy(QFormLayout::FieldsStayAtSizeHint);
    return form;
}

}  // namespace

void MainWindowUi::setupUi(QMainWindow *window) {
    if (window->objectName().isEmpty()) {
        window->setObjectName("MainWindow");
    }
    auto *central = new QWidget(window);
    central->setObjectName("centralWidget");
    window->setCentralWidget(central);
    const QFont fixedFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);

    // graph, left
    lblDataView = new QLabel(tr("Turn Profile"));
    lblDataView->setObjectName("lblDataView");
    customPlot = new QCustomPlot;
    customPlot->setObjectName("customPlot");
    customPlot->setMinimumSize(viewSize, viewSize);
    customPlot->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // path view and slider, right
    lblPathView = new QLabel(tr("Mouse Path"));
    lblPathView->setObjectName("lblPathView");
    lblPathView->setFont(fixedFont);
    lblPathView->setStyleSheet("color: blue");
    pathView = new FitView;
    pathView->setObjectName("pathView");
    pathView->setMinimumSize(viewSize, viewSize);
    pathView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    progressSlider = new QSlider(Qt::Horizontal);
    progressSlider->setObjectName("progressSlider");

    auto *top = new QGridLayout;
    top->addWidget(lblDataView, 0, 0);
    top->addWidget(customPlot, 1, 0);
    top->addWidget(lblPathView, 0, 1);
    top->addWidget(pathView, 1, 1);
    top->addWidget(progressSlider, 2, 1);
    top->setColumnStretch(0, 1);  // graph and path view share extra width equally
    top->setColumnStretch(1, 1);
    top->setRowStretch(1, 1);

    // turn summary
    textEdit = new QTextEdit;
    textEdit->setObjectName("textEdit");
    textEdit->setReadOnly(true);
    textEdit->setFont(fixedFont);
    textEdit->setMinimumWidth(QFontMetrics(fixedFont).horizontalAdvance(QString(30, 'M')));

    // parameters; MainWindow inserts the profile's rows after the acceleration row
    gbSpinners = new QGroupBox(tr("Parameters"));
    gbSpinners->setObjectName("gbSpinners");
    paramLayout = makeForm(gbSpinners);
    paramLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);  // all fields as wide as the widest
    turnSpeedSpinBox = makeSpinBox("turnSpeedSpinBox", 100, 3000, 10, 1000);
    // covers every reachable peak: speed 3000 with cubic length 50 on SS180 is about 848000
    accelerationSpinBox = makeSpinBox("accelerationSpinBox", 0, 1000000, 100, 9800);
    offsetSpinBox = makeSpinBox("offsetSpinBox", -270, 200, 1, 0);
    sensorAngleSpinBox = makeSpinBox("sensorAngleSpinBox", 0, 98, 1, 30);
    paramLayout->addRow(tr("Speed:"), turnSpeedSpinBox);
    paramLayout->addRow(tr("Acc:"), accelerationSpinBox);
    paramLayout->addRow(tr("Offset:"), offsetSpinBox);
    paramLayout->addRow(tr("Sensors:"), sensorAngleSpinBox);
    // tyre slip, shared by all profiles and turns
    slipGradientSpinBox = makeDoubleSpinBox("slipGradientSpinBox", 0, 20, 0);
    slipGradientSpinBox->setToolTip(tr("Slip angle per g of centripetal acceleration (deg/g), 0 = no slip.\n"
                                       "Measure across a slope, no fan: crab angle / tan(slope angle)"));
    loadFactorSpinBox = makeDoubleSpinBox("loadFactorSpinBox", 1, 10, 1);
    loadFactorSpinBox->setToolTip(tr("Normal load / weight: (weight + fan downforce) / weight. 1 = no fan"));
    paramLayout->addRow(tr("Slip:"), slipGradientSpinBox);
    paramLayout->addRow(tr("Load:"), loadFactorSpinBox);

    // turn types
    gbTurnType = new QGroupBox(tr("Turn Type"));
    gbTurnType->setObjectName("gbTurnType");
    auto *turnLayout = new QVBoxLayout(gbTurnType);
    for (int t = 0; t < TURN_TYPE_COUNT; t++) {
        turnButtons[t] = new QRadioButton(tr(turnLabels[t]));
        turnButtons[t]->setObjectName(QString("rb") + turnLabels[t]);
        turnLayout->addWidget(turnButtons[t]);
    }
    turnButtons[TURN_SS90F]->setChecked(true);

    // profiles; MainWindow adds one button per registered profile
    gbTurnProfile = new QGroupBox(tr("Turn Profile"));
    gbTurnProfile->setObjectName("gbTurnProfile");
    profileLayout = new QVBoxLayout(gbTurnProfile);

    // start position
    auto *gbStart = new QGroupBox(tr("Start"));
    gbStart->setObjectName("gbStart");
    auto *startLayout = makeForm(gbStart);
    startXSpinBox = makeSpinBox("startXSpinBox", -90, 90, 1, 0);
    startYSpinBox = makeSpinBox("startYSpinBox", -270, 180, 1, 0);
    startLayout->addRow(tr("X:"), startXSpinBox);
    startLayout->addRow(tr("Y:"), startYSpinBox);

    btnAutoScale = new QPushButton(tr("Auto Scale"));
    btnAutoScale->setObjectName("btnAutoScale");
    btnAutoScale->setCheckable(true);
    btnAutoScale->setToolTip(tr("Fit the graph axes to the data. Off: fixed axes, extended only if data would be clipped"));
    btnReset = new QPushButton(tr("Reset"));
    btnReset->setObjectName("btnReset");
    auto *buttons = new QHBoxLayout;
    buttons->addWidget(btnAutoScale);
    buttons->addWidget(btnReset);

    auto *rightColumn = new QVBoxLayout;
    rightColumn->addWidget(gbTurnProfile);
    rightColumn->addWidget(gbStart);
    rightColumn->addLayout(buttons);
    rightColumn->addStretch();

    auto *bottom = new QHBoxLayout;
    bottom->addWidget(textEdit, 1);
    bottom->addWidget(gbSpinners, 0, Qt::AlignTop);
    bottom->addWidget(gbTurnType, 0, Qt::AlignTop);
    bottom->addLayout(rightColumn);

    auto *main = new QVBoxLayout(central);
    main->addLayout(top, 1);  // extra height goes to the graph and path view
    main->addLayout(bottom, 0);
}
