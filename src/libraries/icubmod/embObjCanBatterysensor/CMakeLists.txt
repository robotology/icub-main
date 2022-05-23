# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Luca Tricerri
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(embObjCanBatterysensor CATEGORY device
                                 TYPE yarp::dev::embObjCanBatterysensor
                                 INCLUDE embObjCanBatterysensor.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjCanBatterysensor)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")


  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjCanBatterysensor embObjCanBatterysensor.cpp embObjCanBatterysensor.h)
  TARGET_LINK_LIBRARIES(embObjCanBatterysensor ethResources)
  icub_export_plugin(embObjCanBatterysensor)

  yarp_install(TARGETS embObjCanBatterysensor
        COMPONENT Runtime
        LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
        ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
        YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  if (BUILD_TESTING)
   add_library(embObjCanBatterysensorUT STATIC embObjCanBatterysensor.cpp embObjCanBatterysensor.h)    
   target_link_libraries(embObjCanBatterysensorUT ethResources
                                    YARP::YARP_dev
                                    icub_firmware_shared::embobj)
                                        
   target_include_directories(embObjCanBatterysensorUT PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  endif()

ENDIF (NOT SKIP_embObjCanBatterysensor)

