# Copyright: (C) 2012 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjMultiEnc CATEGORY device
                                   TYPE yarp::dev::embObjMultiEnc
                                   INCLUDE embObjMultiEnc.h
                                   EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjMultiEnc)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

#   message(INFO " embObjMultiEnc - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjMultiEnc embObjMultiEnc.cpp embObjMultiEnc.h)
  TARGET_LINK_LIBRARIES(embObjMultiEnc ethResources iCubDev)
  icub_export_plugin(embObjMultiEnc)

   yarp_install(TARGETS embObjMultiEnc
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})


ENDIF ()

