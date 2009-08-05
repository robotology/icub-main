SET(KEYWORD "qControlBoardAnalysisGui")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
FIND_PACKAGE(Qt3 REQUIRED)
FIND_PACKAGE(Qwt REQUIRED)

IF(QWT_FOUND)

    # Search for source code.
    FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
    FILE(GLOB folder_include include/iCub/*.h)
    SOURCE_GROUP("Source Files" FILES ${folder_source})
    SOURCE_GROUP("Header Files" FILES ${folder_include})

    INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${QT_INCLUDE_DIR}
        ${QWT_INCLUDE_DIR}/qwt
    )

    ADD_DEFINITIONS(
        ${QT_DEFINITIONS}
    )
                 
    #Qt MOC Headers
    SET(qControlBoardAnalysisGui_MOCS
        ${PROJECT_SOURCE_DIR}/include/iCub/GuiControlboardAnalysis.h
        ${PROJECT_SOURCE_DIR}/include/iCub/GuiControlboardAnalysisAxis.h
        ${PROJECT_SOURCE_DIR}/include/iCub/GuiCBAAbout.h
    )   

    #Qt User Interface Files
    SET(qControlBoardAnalysisGui_UIS
        ${PROJECT_SOURCE_DIR}/include/iCub/guicontrolboardanalysisbase.ui
        ${PROJECT_SOURCE_DIR}/include/iCub/guicontrolboardanalysisaxisbase.ui
        ${PROJECT_SOURCE_DIR}/include/iCub/guicbaaboutbase.ui
    )

    QT_WRAP_UI(${KEYWORD}
        ui_headers
        ui_sources
        ${qControlBoardAnalysisGui_UIS}
    )

    IF( QT_WRAP_CPP )
        QT_WRAP_CPP(${KEYWORD}
            moc_sources
            ${qControlBoardAnalysisGui_MOCS}
        )
    ENDIF(QT_WRAP_CPP)

    SET(qControlBoardAnalysisGui_SRCS
       ${folder_source}
       ${folder_include}
    )

    LINK_LIBRARIES(
        ${QT_LIBRARIES}
        ${QWT_LIBRARY}
    )
        
    ADD_EXECUTABLE(${KEYWORD}
       ${qControlBoardAnalysisGui_SRCS}
       ${moc_sources}
       ${ui_sources}
    )

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE(QWT_FOUND)
    MESSAGE("Qwt not found: ${KEYWORD} will not be built.")
ENDIF(QWT_FOUND)




