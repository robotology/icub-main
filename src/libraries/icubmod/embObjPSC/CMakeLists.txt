# Copyright: (C) 2019 RobotCub Consortium
# Authors: Valentina Gaggero
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjPSC CATEGORY device
                               TYPE yarp::dev::embObjPSC
                               INCLUDE embObjPSC.h
                               EXTRA_CONFIG WRAPPER=AnalogServer)

IF (NOT SKIP_embObjPSC)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${iCubDev_INCLUDE_DIRS})

  yarp_add_plugin(embObjPSC embObjPSC.cpp embObjPSC.h)
  TARGET_LINK_LIBRARIES(embObjPSC ethResources ${YARP_LIBRARIES} ${ACE_LIBRARIES})
  icub_export_plugin(embObjPSC)

  yarp_install(FILES embObjPSC.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjPSC)

