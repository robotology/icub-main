# Copyright: 2011 RobotCub Consortium
# Author: Carlo Ciliberto
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECTNAME actionsRenderingEngine)

project(${PROJECTNAME})

set(folder_source src/main.cpp
                  src/utils.cpp
                  src/MotorThread.cpp
                  src/VisuoThread.cpp)

set(folder_header include/iCub/utils.h 
                  include/iCub/VisuoThread.h
                  include/iCub/MotorThread.h)
                  
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS} ${actionPrimitives_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} actionPrimitives ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)