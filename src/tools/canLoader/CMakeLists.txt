# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(canLoader)

if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling canLoader")
  return()
endif()

message(STATUS " +++ tool compiling canLoaderLib")
add_subdirectory(canLoaderLib)

message(STATUS " +++ tool compiling canLoader-console")
add_subdirectory(canLoader-console)
