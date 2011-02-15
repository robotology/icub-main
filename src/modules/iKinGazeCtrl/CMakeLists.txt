# Copyright: 2010 RobotCub Consortium
# Author: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

SET(PROJECTNAME iKinGazeCtrl)

PROJECT(${PROJECTNAME})

SET(folder_header include/iCub/gazeNlp.h
                  include/iCub/utils.h
                  include/iCub/solver.h
                  include/iCub/controller.h
                  include/iCub/localizer.h)

SET(folder_source src/gazeNlp.cpp
                  src/utils.cpp
                  src/solver.cpp
                  src/controller.cpp
                  src/localizer.cpp
                  src/main.cpp)

SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${iKin_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} iKin
                                     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)




