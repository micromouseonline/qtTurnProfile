#-------------------------------------------------
#
# Project created by QtCreator 2012-04-24T12:29:59
#
#-------------------------------------------------

QT       += core gui
QMAKE_MAC_SDK = macosx10.13
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

TARGET = qtTurnProfile
CONFIG += console
CONFIG += static
TEMPLATE = app

SOURCES += main.cpp\
        mainwindow.cpp \
        graphicmouse.cpp \
        qcustomplot.cpp

HEADERS  += mainwindow.h \
        graphicmouse.h \
    qcustomplot.h \
    ui_mainwindow.h

FORMS    +=

RESOURCES += \
    resources.qrc
