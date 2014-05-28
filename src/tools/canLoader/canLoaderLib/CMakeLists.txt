# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME canLoaderLib)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ACE_INCLUDE_DIRS})
include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})




add_library(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ACE_LIBRARIES})

icub_export_library(canLoaderLib)
