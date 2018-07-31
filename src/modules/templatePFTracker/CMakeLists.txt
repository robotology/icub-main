# Copyright: 2009 RobotCub Consortium
# Author: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME templatePFTracker)
PROJECT(${PROJECTNAME})

FILE(GLOB folder_header include/iCub/*.h)
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS} ${GSL_INCLUDE_DIRS})
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${GSL_LIBRARIES} ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
