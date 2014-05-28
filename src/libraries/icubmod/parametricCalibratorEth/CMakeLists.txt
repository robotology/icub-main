# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(parametricCalibratorEth TYPE parametricCalibratorEth INCLUDE parametricCalibratorEth.h)
          
if(ENABLE_parametricCalibratorEth)     
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                       ${YARP_INCLUDE_DIRS}
                       ${iCubDev_INCLUDE_DIRS}
                       ${CMAKE_SOURCE_DIR}/src/libraries/icubmod/debugStream/)
    
    yarp_add_plugin(parametricCalibratorEth parametricCalibratorEth.h parametricCalibratorEth.cpp)
    target_link_libraries(parametricCalibratorEth debugStream ${YARP_LIBRARIES})
    icub_export_library(parametricCalibratorEth)
endif(ENABLE_parametricCalibratorEth)
