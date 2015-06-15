# Copyright (C) 2010 RobotCub Consortium
# Author: Martim Brandao
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME pf3dBottomup)

PROJECT(${PROJECTNAME})

SET(folder_source src/main.cpp
                  src/cvfloodfill2.cpp
                  src/pf3dBottomup.cpp
                  src/ScaleSpace.cpp
                  src/FastGauss.cpp
                  src/IIRFilt.cpp
                  src/IIRGausDeriv.cpp)
SET(folder_header include/iCub/FastGauss.h
                  include/iCub/ScaleSpace.h
                  include/iCub/IIRFilt.h
                  include/iCub/IIRGausDeriv.h
                  include/iCub/pf3dBottomup.hpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
                    ${OpenCV_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

