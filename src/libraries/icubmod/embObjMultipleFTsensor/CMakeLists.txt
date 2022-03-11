# Copyright: (C) 2012 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjMultipleFTsensor CATEGORY device
                                 TYPE yarp::dev::embObjMultipleFTsensor
                                 INCLUDE embObjMultipleFTsensor.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjMultipleFTsensor)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjMultipleFTsensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjMultipleFTsensor embObjMultipleFTsensor.cpp embObjMultipleFTsensor.h eo_ftsens_privData.cpp eo_ftsens_privData.h AnalogData.cpp AnalogData.h)
  TARGET_LINK_LIBRARIES(embObjMultipleFTsensor ethResources)
  icub_export_plugin(embObjMultipleFTsensor)

          yarp_install(TARGETS embObjMultipleFTsensor
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjMultipleFTsensor)

