

# Copyright (C) 2020 iCub Tech - Istituto Italiano di Tecnologia
# Author:  Marco Accame
# email:   marco.accame@iit.it


yarp_prepare_plugin(embObjPOS CATEGORY device
                               TYPE yarp::dev::embObjPOS
                               INCLUDE embObjPOS.h
                               EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjPOS)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjPOS embObjPOS.cpp embObjPOS.h)
  TARGET_LINK_LIBRARIES(embObjPOS ethResources iCubDev)
  icub_export_plugin(embObjPOS)

 yarp_install(TARGETS embObjPOS
               COMPONENT Runtime
               LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

ENDIF (NOT SKIP_embObjPOS)

