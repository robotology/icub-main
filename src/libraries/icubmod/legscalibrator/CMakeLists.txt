# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(icublegscalibrator TYPE iCubLegsCalibrator INCLUDE iCubLegsCalibrator.h)


if (NOT SKIP_icublegscalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
yarp_add_plugin(legscalibrator iCubLegsCalibrator.cpp iCubLegsCalibrator.h)

target_link_libraries(legscalibrator ${YARP_LIBRARIES})

icub_export_library(legscalibrator)

endif (NOT SKIP_icublegscalibrator)
