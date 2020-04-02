# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjAnalogSensor CATEGORY device
                                       TYPE yarp::dev::embObjAnalogSensor
                                       INCLUDE embObjAnalogSensor.h
                                       EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjAnalogSensor)

  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjAnalogSensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjAnalogSensor embObjAnalogSensor.cpp embObjAnalogSensor.h)
  TARGET_LINK_LIBRARIES(embObjAnalogSensor ethResources iCubDev)
  icub_export_plugin(embObjAnalogSensor)

          yarp_install(TARGETS embObjAnalogSensor
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjAnalogSensor)

