# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

PREPARE_DEVICE(embObjVirtualAnalogSensor TYPE embObjVirtualAnalogSensor   INCLUDE embObjVirtualAnalogSensor.h   WRAPPER virtualAnalogServer)

IF (NOT SKIP_embObjVirtualAnalogSensor)
  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjVirtualAnalogSensor embObjVirtualAnalogSensor.cpp embObjVirtualAnalogSensor.h)
  
  TARGET_LINK_LIBRARIES(embObjVirtualAnalogSensor debugStream  ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
 icub_export_library(embObjVirtualAnalogSensor)
ENDIF (NOT SKIP_embObjVirtualAnalogSensor)

