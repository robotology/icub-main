# Copyright (C) 2010 RobotCub Consortium
# Author: Matteo Fumagalli
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME gravityCompensator)

PROJECT(${PROJECTNAME})
SET(folder_source main.cpp gravityThread.cpp)
SET(folder_header gravityThread.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${skinDynLib_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} skinDynLib iDyn ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
