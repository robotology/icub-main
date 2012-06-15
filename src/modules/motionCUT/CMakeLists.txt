# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME motionCUT)
PROJECT(${PROJECTNAME})

SET(folder_header main.h)
IF(OpenCV_VERSION_MAJOR GREATER 2 OR OpenCV_VERSION_MAJOR EQUAL 2 AND OpenCV_VERSION_MINOR GREATER 2)
    SET(folder_source main_cv_cpp.cpp)
ELSE()
    SET(folder_source main_cv_c.cpp)
ENDIF()

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

