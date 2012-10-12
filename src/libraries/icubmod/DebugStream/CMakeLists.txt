# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
add_library(DebugStream Debug.h Debug.cpp)
#TARGET_LINK_LIBRARIES(DebugStream  ${ACE_LIBRARIES})
icub_export_library(DebugStream)

