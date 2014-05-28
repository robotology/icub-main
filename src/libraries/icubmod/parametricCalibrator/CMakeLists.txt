# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


PREPARE_DEVICE(parametricCalibrator TYPE parametricCalibrator INCLUDE parametricCalibrator.h)

if(ENABLE_parametricCalibrator)
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS}
                       ${iCubDev_INCLUDE_DIRS}
                       ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream/)
    
    yarp_add_plugin(parametricCalibrator parametricCalibrator.h parametricCalibrator.cpp)
    target_link_libraries(parametricCalibrator debugStream ${YARP_LIBRARIES})
    icub_export_library(parametricCalibrator)
endif()
