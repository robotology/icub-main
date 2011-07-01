# Author: Alexandre Bernardino
# Copyright (C) 2008 RobotCub Consortium
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME camExtrinsicsCalib)

PROJECT(${PROJECTNAME})

SET(folder_source camExtrinsicsCalib.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
