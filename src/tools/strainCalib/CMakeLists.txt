# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(strainCalib)

if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling strainCalib")
  return()
endif()


message(STATUS " +++ tool compiling strainCalibLib")
add_subdirectory(strainCalibLib)

if(NOT ICUB_USE_GTK2)
  message(STATUS "GTK2 not selected, skipping strainCalibGUI")
  return()
endif()

message(STATUS " +++ tool compiling strainCalibGUI")
add_subdirectory(strainCalibGUI)


