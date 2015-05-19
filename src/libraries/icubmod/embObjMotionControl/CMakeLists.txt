# Copyright: (C) 2012 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_prepare_device(embObjMotionControl TYPE yarp::dev::embObjMotionControl
    INCLUDE embObjMotionControl.h
    WRAPPER controlboardwrapper2)

 IF (NOT SKIP_embObjMotionControl)

    if(ETHRES_DEBUG_DONTREADBACK)
        add_definitions( -DETHRES_DEBUG_DONTREADBACK)
    endif(ETHRES_DEBUG_DONTREADBACK)

  if (ICUB_AUTOMATIC_MODE_SWITCHING)
	add_definitions(-DICUB_AUTOMATIC_MODE_SWITCHING)
  endif()

  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")
  INCLUDE(../embObjLib/embObjLib.cmake)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${iCubDev_INCLUDE_DIRS} )

#    yarp_add_plugin(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h usrcbk/eOcfg_nvsEP_mc_usrcbk_pippo.c )
    yarp_add_plugin(embObjMotionControl embObjMotionControl.cpp embObjMotionControl.h)
    TARGET_LINK_LIBRARIES(embObjMotionControl ethResources iCubDev ${YARP_LIBRARIES} ${ACE_LIBRARIES})
    icub_export_plugin(embObjMotionControl)
    
    yarp_install(FILES embObjMotionControl.ini  DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})
        
ENDIF (NOT SKIP_embObjMotionControl)

