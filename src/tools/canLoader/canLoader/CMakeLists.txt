# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(PROJECTNAME canLoader)

file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../canLoaderLib)
include_directories(${GTK2_INCLUDE_DIRS})
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ACE_INCLUDE_DIRS})
include_directories(${icub_firmware_shared_canProtocolLib_INCLUDE_DIR})



add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} icubmod
                                     canLoaderLib
                                     ${GTK2_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     ${ACE_LIBRARIES})

install(TARGETS ${PROJECTNAME} DESTINATION bin)

