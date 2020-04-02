# Copyright: (C) 2012 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjFTsensor CATEGORY device
                                 TYPE yarp::dev::embObjFTsensor
                                 INCLUDE embObjFTsensor.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjFTsensor)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjFTsensor - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjFTsensor embObjFTsensor.cpp embObjFTsensor.h eo_ftsens_privData.cpp eo_ftsens_privData.h)
  TARGET_LINK_LIBRARIES(embObjFTsensor ethResources)
  icub_export_plugin(embObjFTsensor)

          yarp_install(TARGETS embObjFTsensor
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjFTsensor)

