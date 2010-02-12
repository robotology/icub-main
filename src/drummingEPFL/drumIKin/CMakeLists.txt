SET(PROJECTNAME drumIKin)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(IKIN REQUIRED)
INCLUDE_DIRECTORIES(${IKIN_INCLUDE_DIRS})

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF(folder_source)
    ADD_EXECUTABLE(${PROJECTNAME} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${IKIN_LIBRARIES})

