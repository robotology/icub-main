# Copyright: (C) 2013 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME robolli_lib)
PROJECT(${PROJECTNAME})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../debugStream)
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ../ ${YARP_INCLUDE_DIRS}  ${iCubDev_INCLUDE_DIRS})

set(ThisLib_SOURCES     Boards_iface.cpp
                        CommProtocol.cpp
                        DSP_board.cpp
                        ../comanDevicesHandler.cpp)

set(ThisLib_HEADERS     Boards_iface.h
                        CommProtocol.hpp
                        definitions.h
                        DSP_board.h
                        ../comanDevicesHandler.hpp)

add_library(robolli_lib ${ThisLib_SOURCES} ${ThisLib_HEADERS})
target_link_libraries(robolli_lib debugStream yaml-cpp  pthread )
icub_export_library(robolli_lib )
