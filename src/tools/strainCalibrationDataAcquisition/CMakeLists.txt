# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

project(strainCalibrationDataAcquisition)

if(NOT ICUB_USE_icub_firmware_shared)
  message(STATUS "icub-firmware-shared not selected, disabling strainCalibrationDataAcquisitionGUI")
  return()
endif()


if(NOT ICUB_USE_GTK2)
  message(STATUS "GTK2 not selected, skipping strainCalibrationDataAcquisitionGUI")
  return()
endif()

message(STATUS " +++ tool compiling strainCalibrationGUI")
add_subdirectory(strainCalibrationDataAcquisitionGUI)


