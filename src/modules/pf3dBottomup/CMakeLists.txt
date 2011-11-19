# Copyright (C) 2010 RobotCub Consortium
# Author: Martim Brandao
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME pf3dBottomup)

PROJECT(${PROJECTNAME})

FILE(GLOB folder_source src/*.cpp)
FILE(GLOB folder_header include/iCub/*.hpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iCubVis_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iCubVis
                                     ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

