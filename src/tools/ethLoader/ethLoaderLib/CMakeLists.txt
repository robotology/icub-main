# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(ethLoaderLib)

file(GLOB folder_header *.h)
file(GLOB folder_source *.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
add_library(${PROJECT_NAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECT_NAME} YARP::YARP_os
                                      YARP::YARP_dev
                                      ACE::ACE
                                      icub_firmware_shared::embobj)
target_include_directories(${PROJECT_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                                                  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

icub_export_library(${PROJECT_NAME})

