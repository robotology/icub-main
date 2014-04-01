# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(embObjAnalogSensor TYPE embObjAnalogSensor   INCLUDE embObjAnalogSensor.h   WRAPPER )

IF (NOT SKIP_embObjAnalogSensor)

  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)

#   message(INFO " embObjAnalogSensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjAnalogSensor embObjAnalogSensor.cpp embObjAnalogSensor.h)
  TARGET_LINK_LIBRARIES(embObjAnalogSensor debugStream  ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_library(embObjAnalogSensor)
ENDIF (NOT SKIP_embObjAnalogSensor)

