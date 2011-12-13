#
# Copyright (C) 2010 RobotCub Consortium
# Author: Alessandro Scalzo alessandro.scalzo@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
set(PROJECTNAME iCubInterfaceGuiLib)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${iCubDev_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${iCubDev_LIBRARIES})