# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(embObjAnalogSensor TYPE yarp::dev::embObjAnalogSensor
                                       INCLUDE embObjAnalogSensor.h
                                       WRAPPER AnalogServer)

IF (NOT SKIP_embObjAnalogSensor)

    if(ETHRES_DEBUG_DONTREADBACK)
        add_definitions( -DETHRES_DEBUG_DONTREADBACK)
    endif(ETHRES_DEBUG_DONTREADBACK)

  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)

#   message(INFO " embObjAnalogSensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjAnalogSensor embObjAnalogSensor.cpp embObjAnalogSensor.h)
  TARGET_LINK_LIBRARIES(embObjAnalogSensor ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(embObjAnalogSensor)

  yarp_install(FILES embObjAnalogSensor.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjAnalogSensor)

