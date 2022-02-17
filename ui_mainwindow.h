/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
//#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include <QObject>
#include <QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow {
public:
    QWidget *centralWidget;
    QGraphicsView *pathView;
    QLabel *lblDataView;
    QLabel *lblPathView;
    QPushButton *btnRefresh;
    QPushButton *btnReset;
    QLabel *lblStartY;
    QSpinBox *startYSpinBox;
    QLabel *lblStartX;
    QSpinBox *startXSpinBox;
    QCustomPlot *customPlot;
    QSlider *progressSlider;
    QTextEdit *textEdit;

    QGroupBox *gbSpinners;
    QSpinBox *turnSpeedSpinBox;
    QSpinBox *accelerationSpinBox;
    QSpinBox *radiusSpinBox;
    QSpinBox *lambdaSpinBox;
    QSpinBox *offsetSpinBox;
    QSpinBox *sensorAngleSpinBox;
    QSpinBox *cubicLengthSpinBox;
    QSpinBox *cubicGammaSpinBox;
    QLabel *lblSpeed;
    QLabel *lblAcc;
    QLabel *lblRadius;
    QLabel *lblLambda;
    QLabel *lblOffset;
    QLabel *lblSensorAngle;
    QLabel *lblCubicLength;
    QLabel *lblCubicGamma;

    QGroupBox *gbTurnProfile;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QRadioButton *rbTrapezoid;
    QRadioButton *rbQuadratic;
    QRadioButton *rbSinusoid;
    QRadioButton *rbCubic;
    QGroupBox *gbTurnType;
    QWidget *widget;
    QVBoxLayout *vlTurnTypes;
    QRadioButton *rbSS90F;
    QRadioButton *rbSS180;
    QRadioButton *rbSD45;
    QRadioButton *rbSD135;
    QRadioButton *rbDS45;
    QRadioButton *rbDS135;
    QRadioButton *rbDD90;
    QRadioButton *rbSS90E;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;

    void setupUi(QMainWindow *MainWindow) {
        if (MainWindow->objectName().isEmpty()) {
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        }
        MainWindow->resize(840, 750);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));

        pathView = new QGraphicsView(centralWidget);
        pathView->setObjectName(QStringLiteral("pathView"));
        pathView->setGeometry(QRect(420, 30, 400, 400));
        pathView->setAutoFillBackground(true);

        lblDataView = new QLabel(centralWidget);
        lblDataView->setObjectName(QStringLiteral("lblDataView"));
        lblDataView->setGeometry(QRect(10, 10, 391, 20));

        lblPathView = new QLabel(centralWidget);
        lblPathView->setFont(QFont("monospace", 10));
        lblPathView->setStyleSheet("font-weight: xbold; color: blue");
        lblPathView->setObjectName(QStringLiteral("lblPathView"));
        lblPathView->setGeometry(QRect(430, 10, 391, 20));

        btnRefresh = new QPushButton(centralWidget);
        btnRefresh->setObjectName(QStringLiteral("btnRefresh"));
        btnRefresh->setGeometry(QRect(745, 440, 65, 28));

        btnReset = new QPushButton(centralWidget);
        btnReset->setObjectName(QStringLiteral("btnRefresh"));
        btnReset->setGeometry(QRect(745, 475, 65, 28));

        lblStartY = new QLabel(centralWidget);
        lblStartY->setObjectName(QStringLiteral("label"));
        lblStartY->setGeometry(QRect(680, 450, 62, 16));
        startYSpinBox = new QSpinBox(centralWidget);
        startYSpinBox->setObjectName(QStringLiteral("startYSpinBox"));
        startYSpinBox->setGeometry(QRect(680, 470, 60, 25));
        startYSpinBox->setMinimum(-270);
        startYSpinBox->setMaximum(180);

        lblStartX = new QLabel(centralWidget);
        lblStartX->setObjectName(QStringLiteral("label_2"));
        lblStartX->setGeometry(QRect(610, 450, 62, 16));
        startXSpinBox = new QSpinBox(centralWidget);
        startXSpinBox->setObjectName(QStringLiteral("startXSpinBox"));
        startXSpinBox->setGeometry(QRect(610, 470, 60, 25));
        startXSpinBox->setMinimum(-90);
        startXSpinBox->setMaximum(90);

        customPlot = new QCustomPlot(centralWidget);
        customPlot->setObjectName(QStringLiteral("customPlot"));
        customPlot->setGeometry(QRect(10, 30, 400, 400));
        progressSlider = new QSlider(centralWidget);
        progressSlider->setObjectName(QStringLiteral("progressSlider"));
        progressSlider->setGeometry(QRect(420, 420, 401, 22));
        progressSlider->setOrientation(Qt::Horizontal);

        textEdit = new QTextEdit(centralWidget);
        textEdit->setObjectName(QStringLiteral("textEdit"));
        textEdit->setGeometry(QRect(10, 450, 241, 330));
        QFont font;
        font.setFamily(QStringLiteral("Consolas"));
        font.setPointSize(10);
        textEdit->setFont(font);

        //========================
        gbSpinners = new QGroupBox(centralWidget);
        gbSpinners->setObjectName(QStringLiteral("gbSpinners"));
        gbSpinners->setGeometry(QRect(260, 440, 150, 290));

        turnSpeedSpinBox = new QSpinBox(gbSpinners);
        turnSpeedSpinBox->setObjectName(QStringLiteral("turnSpeedSpinBox"));
        turnSpeedSpinBox->setGeometry(QRect(70, 30, 70, 24));
        turnSpeedSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        turnSpeedSpinBox->setMinimum(100);
        turnSpeedSpinBox->setMaximum(3000);
        turnSpeedSpinBox->setSingleStep(10);
        turnSpeedSpinBox->setValue(1000);

        accelerationSpinBox = new QSpinBox(gbSpinners);
        accelerationSpinBox->setObjectName(QStringLiteral("accelerationSpinBox"));
        accelerationSpinBox->setGeometry(QRect(70, 60, 70, 24));
        accelerationSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        accelerationSpinBox->setMinimum(1000);
        accelerationSpinBox->setMaximum(70000);
        accelerationSpinBox->setSingleStep(100);
        accelerationSpinBox->setValue(9800);

        radiusSpinBox = new QSpinBox(gbSpinners);
        radiusSpinBox->setObjectName(QStringLiteral("radiusSpinBox"));
        radiusSpinBox->setGeometry(QRect(70, 90, 70, 24));
        radiusSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        radiusSpinBox->setAccelerated(true);
        radiusSpinBox->setMinimum(15);
        radiusSpinBox->setMaximum(270);
        radiusSpinBox->setValue(90);

        lambdaSpinBox = new QSpinBox(gbSpinners);
        lambdaSpinBox->setObjectName(QStringLiteral("lambdaSpinBox"));
        lambdaSpinBox->setGeometry(QRect(70, 120, 70, 24));
        lambdaSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        lambdaSpinBox->setMinimum(0);
        lambdaSpinBox->setMaximum(200);
        lambdaSpinBox->setValue(50);

        offsetSpinBox = new QSpinBox(gbSpinners);
        offsetSpinBox->setObjectName(QStringLiteral("offsetSpinBox"));
        offsetSpinBox->setGeometry(QRect(70, 150, 70, 24));
        offsetSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        offsetSpinBox->setMinimum(-270);
        offsetSpinBox->setMaximum(200);
        offsetSpinBox->setSingleStep(1);

        sensorAngleSpinBox = new QSpinBox(gbSpinners);
        sensorAngleSpinBox->setObjectName(QStringLiteral("sensorAngleSpinBox"));
        sensorAngleSpinBox->setGeometry(QRect(70, 180, 70, 24));
        sensorAngleSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        sensorAngleSpinBox->setMaximum(98);
        sensorAngleSpinBox->setValue(30);

        cubicLengthSpinBox = new QSpinBox(gbSpinners);
        cubicLengthSpinBox->setObjectName(QStringLiteral("cubicLengthSpinBox"));
        cubicLengthSpinBox->setGeometry(QRect(70, 210, 70, 24));
        cubicLengthSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        cubicLengthSpinBox->setMaximum(500);
        cubicLengthSpinBox->setMinimum(50);
        cubicLengthSpinBox->setValue(100);

        cubicGammaSpinBox = new QSpinBox(gbSpinners);
        cubicGammaSpinBox->setObjectName(QStringLiteral("cubicGammaSpinBox"));
        cubicGammaSpinBox->setGeometry(QRect(70, 240, 70, 24));
        cubicGammaSpinBox->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);
        cubicGammaSpinBox->setMaximum(50);
        cubicGammaSpinBox->setMinimum(0);
        cubicGammaSpinBox->setValue(0);

        lblSpeed = new QLabel(gbSpinners);
        lblSpeed->setObjectName(QStringLiteral("label_4"));
        lblSpeed->setGeometry(QRect(10, 30, 55, 24));
        lblSpeed->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblAcc = new QLabel(gbSpinners);
        lblAcc->setObjectName(QStringLiteral("label_5"));
        lblAcc->setGeometry(QRect(10, 60, 55, 24));
        lblAcc->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblRadius = new QLabel(gbSpinners);
        lblRadius->setObjectName(QStringLiteral("label_7"));
        lblRadius->setGeometry(QRect(10, 90, 55, 24));
        lblRadius->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblLambda = new QLabel(gbSpinners);
        lblLambda->setObjectName(QStringLiteral("label_6"));
        lblLambda->setGeometry(QRect(10, 120, 55, 24));
        lblLambda->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblOffset = new QLabel(gbSpinners);
        lblOffset->setObjectName(QStringLiteral("label_8"));
        lblOffset->setGeometry(QRect(10, 150, 55, 24));
        lblOffset->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblSensorAngle = new QLabel(gbSpinners);
        lblSensorAngle->setObjectName(QStringLiteral("label_9"));
        lblSensorAngle->setGeometry(QRect(10, 180, 55, 20));
        lblSensorAngle->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblCubicLength = new QLabel(gbSpinners);
        lblCubicLength->setObjectName(QStringLiteral("lblCubicLength"));
        lblCubicLength->setGeometry(QRect(10, 210, 55, 20));
        lblCubicLength->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        lblCubicGamma = new QLabel(gbSpinners);
        lblCubicGamma->setObjectName(QStringLiteral("lblCubicGamma"));
        lblCubicGamma->setGeometry(QRect(10, 240, 55, 20));
        lblCubicGamma->setAlignment(Qt::AlignRight | Qt::AlignTrailing | Qt::AlignVCenter);

        //===============================
        gbTurnProfile = new QGroupBox(centralWidget);
        gbTurnProfile->setObjectName(QStringLiteral("gbTurnProfile"));
        gbTurnProfile->setGeometry(QRect(610, 500, 200, 130));
        layoutWidget = new QWidget(gbTurnProfile);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 20, 150, 100));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);

        rbSinusoid = new QRadioButton(layoutWidget);
        rbSinusoid->setObjectName(QStringLiteral("rbSinusoid"));
        verticalLayout->addWidget(rbSinusoid);

        rbTrapezoid = new QRadioButton(layoutWidget);
        rbTrapezoid->setObjectName(QStringLiteral("rbTrapezoid"));
        verticalLayout->addWidget(rbTrapezoid);

        rbCubic = new QRadioButton(layoutWidget);
        rbCubic->setObjectName(QStringLiteral("rbCubic"));
        verticalLayout->addWidget(rbCubic);

        rbQuadratic = new QRadioButton(layoutWidget);
        rbQuadratic->setObjectName(QStringLiteral("rbQuadratic"));
        verticalLayout->addWidget(rbQuadratic);

        gbTurnType = new QGroupBox(centralWidget);
        gbTurnType->setObjectName(QStringLiteral("gpTurnType"));
        gbTurnType->setGeometry(QRect(430, 440, 91, 190));
        widget = new QWidget(gbTurnType);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 20, 71, 160));
        rbSS90F = new QRadioButton(widget);
        rbSS90F->setObjectName(QStringLiteral("rbSS90F"));
        rbSS90F->setChecked(true);

        rbSS180 = new QRadioButton(widget);
        rbSS180->setObjectName(QStringLiteral("rbSS180"));

        rbSS90E = new QRadioButton(widget);
        rbSS90E->setObjectName(QStringLiteral("rbSS90E"));

        rbSD45 = new QRadioButton(widget);
        rbSD45->setObjectName(QStringLiteral("rbSD45"));

        rbSD135 = new QRadioButton(widget);
        rbSD135->setObjectName(QStringLiteral("rbSD135"));

        rbDS45 = new QRadioButton(widget);
        rbDS45->setObjectName(QStringLiteral("rbDS45"));

        rbDS135 = new QRadioButton(widget);
        rbDS135->setObjectName(QStringLiteral("rbDS135"));

        rbDD90 = new QRadioButton(widget);
        rbDD90->setObjectName(QStringLiteral("rbDD90"));

        vlTurnTypes = new QVBoxLayout(widget);
        vlTurnTypes->setSpacing(6);
        vlTurnTypes->setContentsMargins(11, 11, 11, 11);
        vlTurnTypes->setObjectName(QStringLiteral("verticalLayout_2"));
        vlTurnTypes->setContentsMargins(0, 0, 0, 0);
        vlTurnTypes->addWidget(rbSS90F);
        vlTurnTypes->addWidget(rbSS180);
        vlTurnTypes->addWidget(rbSD45);
        vlTurnTypes->addWidget(rbSD135);
        vlTurnTypes->addWidget(rbDS45);
        vlTurnTypes->addWidget(rbDS135);
        vlTurnTypes->addWidget(rbDD90);
        vlTurnTypes->addWidget(rbSS90E);

        //========================
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 840, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);

        retranslateUi(MainWindow);
        QMetaObject::connectSlotsByName(MainWindow);
    }  // setupUi

    void retranslateUi(QMainWindow *MainWindow) {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        lblDataView->setText(QApplication::translate("MainWindow", "Turn Profile", 0));
        lblPathView->setText(QApplication::translate("MainWindow", "Mouse Path", 0));
        btnRefresh->setText(QApplication::translate("MainWindow", "Refresh", 0));
        btnReset->setText(QApplication::translate("MainWindow", "Reset", 0));
        lblStartY->setText(QApplication::translate("MainWindow", "Start Y", 0));
        lblStartX->setText(QApplication::translate("MainWindow", "Start X", 0));
        lblRadius->setText(QApplication::translate("MainWindow", "Radius:", 0));
        lblSpeed->setText(QApplication::translate("MainWindow", "Speed:", 0));
        lblAcc->setText(QApplication::translate("MainWindow", "Acc:", 0));
        lblLambda->setText(QApplication::translate("MainWindow", "Lambda:", 0));
        lblOffset->setText(QApplication::translate("MainWindow", "Offset:", 0));
        lblSensorAngle->setText(QApplication::translate("MainWindow", "Sensors:", 0));
        lblCubicLength->setText(QApplication::translate("MainWindow", "Length:", 0));
        lblCubicGamma->setText(QApplication::translate("MainWindow", "Gamma:", 0));
        gbSpinners->setTitle(QApplication::translate("MainWindow", "Parameters", 0));
        gbTurnProfile->setTitle(QApplication::translate("MainWindow", "Turn Profile", 0));
        rbTrapezoid->setText(QApplication::translate("MainWindow", "Trapezoid", 0));
        rbQuadratic->setText(QApplication::translate("MainWindow", "Quadratic", 0));
        rbSinusoid->setText(QApplication::translate("MainWindow", "Sinusoid", 0));
        rbCubic->setText(QApplication::translate("MainWindow", "Cubic Spiral", 0));
        gbTurnType->setTitle(QApplication::translate("MainWindow", "Turn Type", 0));
        rbSS90F->setText(QApplication::translate("MainWindow", "SS90F", 0));
        rbSS180->setText(QApplication::translate("MainWindow", "SS180", 0));
        rbSD45->setText(QApplication::translate("MainWindow", "SD45", 0));
        rbSD135->setText(QApplication::translate("MainWindow", "SD135", 0));
        rbDS45->setText(QApplication::translate("MainWindow", "DS45", 0));
        rbDS135->setText(QApplication::translate("MainWindow", "DS135", 0));
        rbDD90->setText(QApplication::translate("MainWindow", "DD90", 0));
        rbSS90E->setText(QApplication::translate("MainWindow", "SS90E", 0));
    }  // retranslateUi
};

namespace Ui {
    class MainWindow : public Ui_MainWindow {
    };
}  // namespace Ui

QT_END_NAMESPACE

#endif  // UI_MAINWINDOW_H
