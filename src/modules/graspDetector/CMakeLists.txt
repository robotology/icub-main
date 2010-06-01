SET(PROJECTNAME graspDetector)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp
                  graspDetector.cpp
                  fingerDetector.cpp)

SET(folder_header graspDetector.h
                  fingerDetector.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
