SET(KEYWORD "guiDemo")
PROJECT(${KEYWORD})

#FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(Qt3 REQUIRED)

# Search for source code.
FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
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
SET(guiDemo_MOCS
    ${PROJECT_SOURCE_DIR}/maindlg.ui.h
)   

#Qt User Interface Files
SET(guiDemo_UIS
    ${PROJECT_SOURCE_DIR}/maindlg.ui
)

QT_WRAP_UI(guiDemo
    ui_headers
    ui_sources
    ${guiDemo_UIS}
)

IF( QT_WRAP_CPP )
    QT_WRAP_CPP(guiDemo
        moc_sources
        ${guiDemo_MOCS}
    )
ENDIF(QT_WRAP_CPP)

### guiDemo ###
SET(guiDemo_SRCS
   ${folder_source}
   ${folder_header}
)

LINK_LIBRARIES(
    ${QT_LIBRARIES}
)
    
ADD_EXECUTABLE(${KEYWORD}
   ${guiDemo_SRCS}
   ${moc_sources}
   ${ui_sources}
)
           
INSTALL_TARGETS(/bin ${KEYWORD})





