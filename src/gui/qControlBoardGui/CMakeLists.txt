PROJECT(qControlBoardGui)

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Qt3 REQUIRED)

SET(PROJECT_NAME qControlBoardGui)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${QT_INCLUDE_DIR}
)

ADD_DEFINITIONS(
    ${QT_DEFINITIONS}
)
             
#Qt MOC Headers
SET(qControlBoardGui_MOCS
    ${PROJECT_SOURCE_DIR}/include/iCub/GuiControlboard.h
    ${PROJECT_SOURCE_DIR}/include/iCub/GuiControlboardAxis.h
    ${PROJECT_SOURCE_DIR}/include/iCub/GuiAbout.h
)   

#Qt User Interface Files
SET(qControlBoardGui_UIS
    ${PROJECT_SOURCE_DIR}/include/iCub/guicontrolboardbase.ui
    ${PROJECT_SOURCE_DIR}/include/iCub/guicontrolboardaxisbase.ui
    ${PROJECT_SOURCE_DIR}/include/iCub/guiaboutbase.ui
)

QT_WRAP_UI(qControlBoardGui
    ui_headers
    ui_sources
    ${qControlBoardGui_UIS}
)

IF( QT_WRAP_CPP )
    QT_WRAP_CPP(qControlBoardGui
        moc_sources
        ${qControlBoardGui_MOCS}
    )
ENDIF(QT_WRAP_CPP)

### qControlBoardGui ###
SET(qControlBoardGui_SRCS
   ${folder_source}
   ${folder_header}
)

LINK_LIBRARIES(
    ${QT_LIBRARIES}
)
    
ADD_EXECUTABLE(qControlBoardGui
   ${qControlBoardGui_SRCS}
   ${moc_sources}
   ${ui_sources}
)
           
INSTALL_TARGETS(/bin qControlBoardGui)





