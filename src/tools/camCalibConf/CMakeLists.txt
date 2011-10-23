# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME camCalibConf)

PROJECT(${PROJECTNAME})

SET(folder_source src/main.cpp
                  src/CamCalibConfModule.cpp)

SET(folder_header include/iCub/CamCalibConfModule.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES}
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

