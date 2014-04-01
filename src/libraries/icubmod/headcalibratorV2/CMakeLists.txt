# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(icubheadcalibratorV2 TYPE iCubHeadCalibratorV2 INCLUDE iCubHeadCalibratorV2.h)

if (NOT SKIP_icubheadcalibratorV2)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
yarp_add_plugin(headcalibratorV2 iCubHeadCalibratorV2.cpp iCubHeadCalibratorV2.h)
target_link_libraries(headcalibratorV2 ${YARP_LIBRARIES})

icub_export_library(headcalibratorV2)

endif(NOT SKIP_icubheadcalibratorV2)
