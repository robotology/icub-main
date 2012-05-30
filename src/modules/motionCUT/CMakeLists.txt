# Copyright: 2010 RobotCub Consortium
# Author: Carlo Ciliberto, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME motionCUT)

PROJECT(${PROJECTNAME})

SET(OpenCV_USE_CPP FALSE)
IF(OpenCV_VERSION_MAJOR GREATER 2)
    SET(OpenCV_USE_CPP TRUE)
ENDIF()
IF(OpenCV_VERSION_MAJOR EQUAL 2 AND OpenCV_VERSION_MINOR GREATER 2)
    SET(OpenCV_USE_CPP TRUE)
ENDIF()

SET(folder_source main.cpp)
IF(OpenCV_USE_CPP)
    ADD_DEFINITIONS(-DOpenCV_USE_CPP)
    SOURCE_GROUP("Source Files" FILES ${folder_source} main_cv_cpp.cpp)
ELSE()
    SOURCE_GROUP("Source Files" FILES ${folder_source} main_cv_c.cpp)
ENDIF()

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

