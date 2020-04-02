# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_plugin(embObjMotionControl
    CATEGORY device
    TYPE yarp::dev::embObjMotionControl
    INCLUDE embObjMotionControl.h
    EXTRA_CONFIG WRAPPER=controlboardwrapper2)

 IF (NOT SKIP_embObjMotionControl)

  if (ICUB_ASK_REFERENCE_TO_FIRMWARE)
      add_definitions(-DASK_REFERENCE_TO_FIRMWARE)
  endif()

  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

#    yarp_add_plugin(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h usrcbk/eOcfg_nvsEP_mc_usrcbk_pippo.c )
    yarp_add_plugin(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h eomcParser.cpp eomcParser.h measuresConverter.cpp measuresConverter.h eomcUtils.h)
    TARGET_LINK_LIBRARIES(embObjMotionControl ethResources iCubDev)
    icub_export_plugin(embObjMotionControl)
    
      yarp_install(TARGETS embObjMotionControl
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
        
ENDIF ()

