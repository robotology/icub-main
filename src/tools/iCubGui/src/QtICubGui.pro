#-------------------------------------------------
#
# Project created by QtCreator 2014-11-19T09:26:43
#
#-------------------------------------------------

QT       += core gui opengl

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = QtICubGui
TEMPLATE = app


SOURCES += main.cpp\
    animationview.cpp \
    bvh.cpp \
    bvhnode.cpp \
    camera.cpp \
    settings.cpp \
    qavimator.cpp \
    settingsdialog.cpp

HEADERS  += \
    animationview.h \
    bvh.h \
    bvhnode.h \
    bvhnodedh.h \
    bvhnodeend.h \
    bvhnodeeye.h \
    bvhnodeforcetorque.h \
    bvhnodeinertial.h \
    bvhnodelefthand.h \
    bvhnoderighthand.h \
    bvhnoderoot.h \
    bvhnoderpy_xyz.h \
    camera.h \
    mesh.h \
    objectsthread.h \
    playstate.h \
    settings.h \
    visionobj.h \
    qavimator.h \
    settingsdialog.h \
    subtitilessthread.h

FORMS    += \
    qavimator.ui \
    settingsdialog.ui



win32-msvc2010{
    INCLUDEPATH += $$(ICUB_ROOT)/src/libraries/skinDynLib/include
    INCLUDEPATH += $$(GLUT_DIR)
    INCLUDEPATH += $$(YARP_ROOT)/src/libYARP_OS/include
    INCLUDEPATH += $$(YARP_ROOT)/src/libYARP_dev/include
    INCLUDEPATH += $$(YARP_ROOT)/src/libYARP_sig/include
    INCLUDEPATH += $$(YARP_ROOT)/build/generated_include/
    INCLUDEPATH += $$(YARP_ROOT)/src/yarpmanager/libymanager/include

    CONFIG(debug, debug|release) {

        DEFINES += WIN32
        DEFINES += _WINDOWS
        DEFINES += _DEBUG
        DEFINES += _REENTRANT
        DEFINES += _CRT_SECURE_NO_DEPRECATE
        DEFINES += _CRT_NONSTDC_NO_DEPRECATE
        DEFINES += QT_DLL
        DEFINES += QT_THREAD_SUPPORT
        DEFINES += NO_DEBUG
        DEFINES += _MBCS


        LIBS += -L"$$(GLUT_DIR)"
        LIBS += -L$$(YARP_ROOT)/build/lib/Debug
        LIBS += -L"$$(ACE_ROOT)/lib"
        LIBS += -L"$$(GSL_DIR)/lib"
        LIBS += -L$$(ICUB_ROOT)/build/lib/Debug

        LIBS += -lYARP_mathd
        LIBS += -lYARP_managerd
        #LIBS += -lYARP_tinyxmld
        LIBS += -lYARP_OSd
        LIBS += -lYARP_sigd
        LIBS += -lYARP_initd
        LIBS += -lgsl
        LIBS += -lgslcblas
        LIBS += -lyarpcard
        #LIBS += -lYARP_wire_rep_utilsd
        #LIBS += -lyarp_tcprosd
        LIBS += -lyarp_bayerd
        #LIBS += -lyarp_humand
        LIBS += -lACEd
        LIBS += -lWinmm
        LIBS += -lAdvapi32
        LIBS += -lShell32
        LIBS += -lglut32
        LIBS += -lskinDynLibd
    }else{

        DEFINES += WIN32
        DEFINES += _WINDOWS
        DEFINES += NDEBUG
        DEFINES += _REENTRANT
        DEFINES += _CRT_SECURE_NO_DEPRECATE
        DEFINES += _CRT_NONSTDC_NO_DEPRECATE
        DEFINES += QT_DLL
        DEFINES += QT_THREAD_SUPPORT
        DEFINES += NO_DEBUG
        DEFINES += _MBCS

        LIBS += -L"$$(GLUT_DIR)"
        LIBS += -L$$(YARP_ROOT)/build/lib/Release
        LIBS += -L"$$(ACE_ROOT)/lib"
        LIBS += -L"$$(GSL_DIR)/lib"
        LIBS += -L$$(ICUB_ROOT)/build/lib/Release

        LIBS += -lYARP_math
        LIBS += -lYARP_manager
        #LIBS += -lYARP_tinyxmld
        LIBS += -lYARP_OS
        LIBS += -lYARP_sig
        LIBS += -lYARP_init
        LIBS += -lgsl
        LIBS += -lgslcblas
        LIBS += -lyarpcar
        #LIBS += -lYARP_wire_rep_utilsd
        #LIBS += -lyarp_tcprosd
        LIBS += -lyarp_bayer
        #LIBS += -lyarp_humand
        LIBS += -lACE
        LIBS += -lWinmm
        LIBS += -lAdvapi32
        LIBS += -lShell32
        LIBS += -lglut32
        LIBS += -lskinDynLib
    }


}

RESOURCES += \
    res.qrc
