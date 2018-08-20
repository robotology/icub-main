# Copyright (C) 2015
# Author: Marco Randazzo <marco.randazzo@iit.it>
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME dualCamCalib)

PROJECT(${PROJECTNAME})

SET(folder_source src/main.cpp
                  src/spherical_projection.cpp
                  src/DualCamCalibModule.cpp
                  src/CalibToolFactory.cpp
                  src/PinholeCalibTool.cpp
                  src/SphericalCalibTool.cpp)

SET(folder_header include/iCub/spherical_projection.h
                  include/iCub/DualCamCalibModule.h
                  include/iCub/CalibToolFactory.h
                  include/iCub/ICalibTool.h
                  include/iCub/PinholeCalibTool.h
                  include/iCub/SphericalCalibTool.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${OpenCV_LIBRARIES} ${YARP_LIBRARIES})
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

