SET(KEYWORD "libYARP_QWidgets")
PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ACE REQUIRED)
FIND_PACKAGE(Qt3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/yarp/gui/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${QT_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${ACE_INCLUDE_DIRS}
)

ADD_DEFINITIONS(
    ${QT_DEFINITIONS}
)

IF(WIN32)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x0400)
ENDIF(WIN32)

#Qt MOC Headers
SET(libYARP_QWidgets_MOCS
    #${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetSalience.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetViewer.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QGLVideo.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetConnection.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetConnections.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetOutput.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetRPC.h
	${PROJECT_SOURCE_DIR}/include/yarp/gui/ControlboardAxis.h
	${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetControlboard.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetModuleDefault.h
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/QWidgetApplication.h
)   

#Qt User Interface Files
SET(libYARP_QWidgets_UIS
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetviewerbase.ui
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetconnectionbase.ui
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetconnectionsbase.ui
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetoutputbase.ui
    ${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetrpcbase.ui
	${PROJECT_SOURCE_DIR}/include/yarp/gui/controlboardaxisbase.ui
	${PROJECT_SOURCE_DIR}/include/yarp/gui/qwidgetcontrolboardbase.ui
)

QT_WRAP_UI(${KEYWORD}
    ui_headers
    ui_sources
    ${libYARP_QWidgets_UIS}
)

IF( QT_WRAP_CPP )
    QT_WRAP_CPP(${KEYWORD}
        moc_sources
        ${libYARP_QWidgets_MOCS}
    )
ENDIF(QT_WRAP_CPP)

SET(libYARP_QWidgets_SRCS
   ${folder_source}
   ${folder_header}
)

LINK_LIBRARIES(
    ${QT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${ACE_LIBRARIES}
)

ADD_LIBRARY(${KEYWORD}
   ${libYARP_QWidgets_SRCS}
   ${moc_sources}
   ${ui_sources}
)
           
INSTALL_TARGETS(/lib ${KEYWORD})





