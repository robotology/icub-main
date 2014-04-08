# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(icubtorsoonlycalibrator TYPE iCubTorsoOnlyCalibrator INCLUDE iCubTorsoOnlyCalibrator.h)

if (NOT SKIP_icubtorsoonlycalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS})

yarp_add_plugin(torsoonlycalibrator iCubTorsoOnlyCalibrator.cpp iCubTorsoOnlyCalibrator.h)
target_link_libraries(torsoonlycalibrator ${YARP_LIBRARIES})

icub_export_library(torsoonlycalibrator)

endif (NOT SKIP_icubtorsoonlycalibrator)
