SET(KEYWORD "qViewerGui")
PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Qt3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(YARPQWIDGETS REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${QT_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${YARPQWIDGETS_INCLUDE_DIRS}
)

ADD_DEFINITIONS(
    ${QT_DEFINITIONS}
)
             
#Qt MOC Headers
SET(qViewerGui_MOCS
    ${PROJECT_SOURCE_DIR}/include/iCub/QViewer.h
)   

#Qt User Interface Files
SET(qViewerGui_UIS
    ${PROJECT_SOURCE_DIR}/include/iCub/qviewerbase.ui
)

QT_WRAP_UI(qViewerGui
    ui_headers
    ui_sources
    ${qViewerGui_UIS}
)

IF( QT_WRAP_CPP )
    QT_WRAP_CPP(qViewerGui
        moc_sources
        ${qViewerGui_MOCS}
    )
ENDIF(QT_WRAP_CPP)

### qViewerGui ###
SET(qViewerGui_SRCS
   ${folder_source}
   ${folder_header}
)

LINK_LIBRARIES(
    ${QT_LIBRARIES}
    ${OPENGL_LIBRARIES}
    ${YARPQWIDGETS_LIBRARIES}
)
    
ADD_EXECUTABLE(${KEYWORD}
   ${qViewerGui_SRCS}
   ${moc_sources}
   ${ui_sources}
)
           
INSTALL_TARGETS(/bin ${KEYWORD})





