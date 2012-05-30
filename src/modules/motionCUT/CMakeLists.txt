# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME motionCUT)

PROJECT(${PROJECTNAME})

SET(folder_source main.cpp)
IF(OpenCV_VERSION_MAJOR LESS 3 AND OpenCV_VERSION_MINOR LESS 3)
    SOURCE_GROUP("Source Files" FILES ${folder_source} main_cv_c.cpp)
ELSE()
    SOURCE_GROUP("Source Files" FILES ${folder_source} main_cv_cpp.cpp)
ENDIF()

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

