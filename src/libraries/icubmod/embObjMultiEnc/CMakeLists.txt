# Copyright: (C) 2012 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjMultiEnc CATEGORY device
                                   TYPE yarp::dev::embObjMultiEnc
                                   INCLUDE embObjMultiEnc.h
                                   EXTRA_CONFIG WRAPPER=AnalogServer)

IF (NOT SKIP_embObjMultiEnc)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)

#   message(INFO " embObjMultiEnc - embObj_includes: ${embObj_includes}, ${CMAKE_CURRENT_SOURCE_DIR}/")
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

  yarp_add_plugin(embObjMultiEnc embObjMultiEnc.cpp embObjMultiEnc.h)
  TARGET_LINK_LIBRARIES(embObjMultiEnc ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(embObjMultiEnc)

  yarp_install(FILES embObjMultiEnc.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjMultiEnc)

