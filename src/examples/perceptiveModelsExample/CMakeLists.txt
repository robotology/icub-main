# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME perceptiveModelsExample)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${perceptiveModels_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} perceptiveModels
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

