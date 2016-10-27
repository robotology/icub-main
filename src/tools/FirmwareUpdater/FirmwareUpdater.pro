#-------------------------------------------------
#
# Project created by QtCreator 2016-07-26T09:35:04
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FirmwareUpdater
TEMPLATE = app



SOURCES += main.cpp\
        mainwindow.cpp \
    firmwareupdatercore.cpp \
    selectioncheckbox.cpp \
    changeinfodialog.cpp \
    changeaddressdialog.cpp

HEADERS  += mainwindow.h \
    firmwareupdatercore.h \
    selectioncheckbox.h \
    changeinfodialog.h \
    changeaddressdialog.h

FORMS    += mainwindow.ui \
    changeinfodialog.ui \
    changeaddressdialog.ui


win32-msvc2010{

    INCLUDEPATH += $$(YARP_DIR)/include
    INCLUDEPATH += $$(ICUB_ROOT)/src/tools/ethLoader/ethLoaderLib
    INCLUDEPATH += $$(ICUB_ROOT)/src/tools/canLoader/canLoaderLib

    CONFIG(debug, debug|release) {
        LIBS += -L$$(YARP_ROOT)/build/lib/Debug
        LIBS += -L$$(ICUB_ROOT)/build/lib/Debug


        LIBS += -lYARP_OSd
        LIBS += -lYARP_sigd
        LIBS += -lYARP_mathd
        LIBS += -lYARP_devdDebug
        LIBS += -lYARP_named
        LIBS += -lYARP_initd
        LIBS += -lethLoaderLibds


    }else{

        LIBS += -L$$(YARP_ROOT)/build/lib/Release
        LIBS += -L$$(ICUB_ROOT)/build/lib/Release


        LIBS += -lYARP_OS
        LIBS += -lYARP_sig
        LIBS += -lYARP_math
        LIBS += -lYARP_dev
        LIBS += -lYARP_name
        LIBS += -lYARP_init
        LIBS += -lethLoaderLib
        LIBS += -lcanLoaderLib
        DEFINES += UPDATER_RELEASE
    }
}

linux{

    INCLUDEPATH += $$(YARP_DIR)/include
    INCLUDEPATH += $$(ICUB_ROOT)/src/tools/ethLoader/ethLoaderLib
    INCLUDEPATH += $$(ICUB_ROOT)/src/tools/canLoader/canLoaderLib
    INCLUDEPATH += $$(ICUB_ROOT)/../icub-firmware-shared/eth/embobj/plus/comm-v2/icub
    INCLUDEPATH += $$(ICUB_ROOT)/../icub-firmware-shared/eth/embobj/core/core
    INCLUDEPATH += $$(ICUB_ROOT)/../icub-firmware-shared/can/canProtocolLib
    INCLUDEPATH += $$(ICUB_ROOT)/../icub-firmware-shared/can

    CONFIG(debug, debug|release) {



    }else{
        DEFINES += UPDATER_RELEASE
    }

    LIBS += -L$$(YARP_ROOT)/build/lib
    LIBS += -L$$(ICUB_ROOT)/build/lib

    LIBS += -lethLoaderLib
    LIBS += -lcanLoaderLib
    LIBS += -lYARP_math
    LIBS += -lYARP_dev
    LIBS += -lYARP_init
    LIBS += -lYARP_name
    LIBS += -lACE

    LIBS += -lYARP_sig
    LIBS += -lYARP_OS
}

DISTFILES +=

RESOURCES += \
    res.qrc

