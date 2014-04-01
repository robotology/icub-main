# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(icubheadcalibrator TYPE iCubHeadCalibrator INCLUDE iCubHeadCalibrator.h)

if (NOT SKIP_icubheadcalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
yarp_add_plugin(headcalibrator iCubHeadCalibrator.cpp iCubHeadCalibrator.h)
target_link_libraries(headcalibrator ${YARP_LIBRARIES})

icub_export_library(headcalibrator)

endif(NOT SKIP_icubheadcalibrator)
