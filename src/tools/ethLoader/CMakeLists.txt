# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Alessandro Scalzo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PROJECT(ethLoader)

if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling ethLoader")
  return()
endif()

message(STATUS " +++ tool compiling ethLoaderLib")
add_subdirectory(ethLoaderLib)
