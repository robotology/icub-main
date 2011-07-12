
# Copyright (C) 2010 RobotCub Consortium
# Author: Matteo Fumagalli
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

 
SET(PROJECTNAME wholeBodyTorqueObserver)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source main.cpp observerThread.cpp)
FILE(GLOB folder_header observerThread.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iDyn
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)