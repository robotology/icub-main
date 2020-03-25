# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjStrain CATEGORY device
                                 TYPE yarp::dev::embObjStrain
                                 INCLUDE embObjStrain.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjStrain)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjStrain - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjStrain embObjStrain.cpp embObjStrain.h)
  TARGET_LINK_LIBRARIES(embObjStrain ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(embObjStrain)

  yarp_install(TARGETS embObjStrain
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjStrain)

