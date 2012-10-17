# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME debugStream)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
add_library(${PROJECTNAME} Debug.h Debug.cpp)
#TARGET_LINK_LIBRARIES(${PROJECTNAME}  ${ACE_LIBRARIES})
icub_export_library(${PROJECTNAME})

