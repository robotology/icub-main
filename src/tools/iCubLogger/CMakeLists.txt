# Copyright: (C) 2014 RobotCub Consortium
# Authors: Marco Randazzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
SET(PROJECTNAME iCubLogger)
PROJECT(${PROJECTNAME})

FILE(GLOB source *.cpp)
FILE(GLOB header *.h)

SOURCE_GROUP("Source Files" FILES ${source})
SOURCE_GROUP("Header Files" FILES ${header})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${source} ${header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


