SET(KEYWORD "CrawlManager")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)



FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF(folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

SOURCE_GROUP("Source Files" FILES ${folder_source})

TARGET_LINK_LIBRARIES(${KEYWORD} )
INSTALL(TARGETS ${KEYWORD} DESTINATION ${ICUB_DIR}/bin)

