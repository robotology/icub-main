# Copyright: (C) 2010 RobotCub Consortium
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME actionPrimitives)

PROJECT(${PROJECTNAME})

SET(folder_source src/actionPrimitives.cpp)
SET(folder_header include/iCub/action/actionPrimitives.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${perceptiveModels_INCLUDE_DIRS}
                    ${iDyn_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} perceptiveModels iDyn icubmod
                                      ${YARP_LIBRARIES})

icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   EXTERNAL_INCLUDE_DIRS ${perceptiveModels_INCLUDE_DIRS}
                                   DEPENDS perceptiveModels iDyn
                                   DESTINATION include/iCub/action
                                   FILES ${folder_header})
