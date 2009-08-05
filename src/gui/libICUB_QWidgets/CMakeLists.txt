SET(KEYWORD "libICUB_QWidgets")
PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(Qt3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(SALIENCELIB REQUIRED)
FIND_PACKAGE(YARPQWIDGETS REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${SALIENCELIB_INCLUDE_DIRS}
    ${QT_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${YARPQWIDGETS_INCLUDE_DIRS}
)

ADD_DEFINITIONS(
    ${QT_DEFINITIONS}
)
             
IF(WIN32)
    ADD_DEFINITIONS(-D_WIN32_WINNT=0x0400)
ENDIF(WIN32)

#Qt MOC Headers
SET(libICUB_QWidgets_MOCS
    ${PROJECT_SOURCE_DIR}/include/iCub/GuiSalience.h
    ${PROJECT_SOURCE_DIR}/include/iCub/QWidgetSalience.h
    ${PROJECT_SOURCE_DIR}/include/iCub/GuiSalienceFilter.h
)   

#Qt User Interface Files
SET(libICUB_QWidgets_UIS
    ${PROJECT_SOURCE_DIR}/include/iCub/guisaliencebase.ui
    ${PROJECT_SOURCE_DIR}/include/iCub/guisaliencefilterbase.ui
)

QT_WRAP_UI(${KEYWORD}
    ui_headers
    ui_sources
    ${libICUB_QWidgets_UIS}
)

IF( QT_WRAP_CPP )
    QT_WRAP_CPP(${KEYWORD}
        moc_sources
        ${libICUB_QWidgets_MOCS}
    )
ENDIF(QT_WRAP_CPP)

SET(libICUB_QWidgets_SRCS
   ${folder_source}
   ${folder_header}
)

LINK_LIBRARIES(
    ${QT_LIBRARIES}
    ${SALIENCELIB_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YARPQWIDGETS_LIBRARIES}
)

ADD_LIBRARY(${KEYWORD}
   ${libICUB_QWidgets_SRCS}
   ${moc_sources}
   ${ui_sources}
)
           
INSTALL_TARGETS(/lib ${KEYWORD})





