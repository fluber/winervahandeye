#-------------------------------------------------
#
# Project created by QtCreator 2018-07-04T18:54:07
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = WinervaHandEye
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
DEFINES += QTHREAD_HAS_VARIADIC_CREATE
DEFINES += __WINDOWS__
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

win32:DEFINES += WIN32

#includes
INCLUDEPATH   += "$$(ENSENSO_INSTALL)/Development/C/include"

#libs
QMAKE_LIBDIR  += "$$(ENSENSO_INSTALL)/Development/C/lib"

LIBS    += "$$(ENSENSO_INSTALL)/Development/C/lib/nxlib64.lib"
LIBS += -L$$(WINERVA)/lib -lWinerva3DLib

INCLUDEPATH += $$(WINERVA)/include
DEPENDPATH += $$(WINERVA)/lib

SOURCES += \
        main.cpp \
    handeyecalibratedialog.cpp \
    hiwinrobot.cpp \
    socketclient.cpp \
    nxlibtransformationhelper.cpp \
    qtColormaps.cpp

HEADERS += \
    handeyecalibratedialog.h \
    hiwinrobot.h \
    socketclient.h \
    nxlibtransformationhelper.h \
    qtColormaps.h

FORMS += \
    handeyecalibratedialog.ui
