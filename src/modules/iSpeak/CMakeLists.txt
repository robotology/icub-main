# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME iSpeak)
PROJECT(${PROJECTNAME})

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS} ${thriftGenerated_INCLUDE_DIRS})

SET(folder_source main.cpp)
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} thriftGenerated)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)