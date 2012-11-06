# Copyright: (C) 2010 RobotCub Consortium
# Authors: Marco Maggiali
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(icubtalkingheadcalibrator TYPE iCubTalkingHeadCalibrator INCLUDE iCubTalkingHeadCalibrator.h)

if (NOT SKIP_icubTalkingHeadcalibrator)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(TalkingHeadcalibrator iCubTalkingHeadCalibrator.cpp iCubTalkingHeadCalibrator.h)
target_link_libraries(TalkingHeadcalibrator ${YARP_LIBRARIES})

icub_export_library(TalkingHeadcalibrator)

endif(NOT SKIP_icubTalkingHeadcalibrator)
