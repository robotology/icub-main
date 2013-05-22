# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(embObjVirtualAnalogSensor TYPE embObjVirtualAnalogSensor   INCLUDE embObjVirtualAnalogSensor.h   WRAPPER virtualAnalogServer)

IF (NOT SKIP_embObjVirtualAnalogSensor)
  INCLUDE(../embObjLib/embObjLib.conf)
#   message(INFO " embObjVirtualAnalogSensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  ADD_LIBRARY(embObjVirtualAnalogSensor embObjVirtualAnalogSensor.cpp embObjVirtualAnalogSensor.h)
  
  TARGET_LINK_LIBRARIES(embObjVirtualAnalogSensor debugStream  ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  
 icub_export_library(embObjVirtualAnalogSensor)
ENDIF (NOT SKIP_embObjVirtualAnalogSensor)

