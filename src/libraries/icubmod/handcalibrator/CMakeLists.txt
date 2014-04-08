# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(icubhandcalibrator TYPE iCubHandCalibrator INCLUDE iCubHandCalibrator.h)

if (NOT SKIP_icubhandcalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
yarp_add_plugin(handcalibrator iCubHandCalibrator.cpp iCubHandCalibrator.h)
target_link_libraries(handcalibrator ${YARP_LIBRARIES})

icub_export_library(handcalibrator)
    
endif(NOT SKIP_icubhandcalibrator)
