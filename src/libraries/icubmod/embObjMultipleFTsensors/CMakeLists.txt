# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Luca Tricerri
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(embObjMultipleFTsensors CATEGORY device
                                 TYPE yarp::dev::embObjMultipleFTsensors
                                 INCLUDE embObjMultipleFTsensors.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

if (NOT SKIP_embObjMultipleFTsensors)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")


  include_directories(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjMultipleFTsensors embObjMultipleFTsensors.cpp embObjMultipleFTsensors.h)
  target_link_libraries(embObjMultipleFTsensors ethResources
                        YARP::YARP_os YARP::YARP_dev YARP::YARP_sig)
  icub_export_plugin(embObjMultipleFTsensors)

  yarp_install(TARGETS embObjMultipleFTsensors
        COMPONENT Runtime
        LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
        ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
        YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  if (BUILD_TESTING)
   add_library(embObjMultipleFTsensorsUT STATIC embObjMultipleFTsensors.cpp embObjMultipleFTsensors.h)
    target_link_libraries(embObjMultipleFTsensorsUT ethResources
                          YARP::YARP_os YARP::YARP_dev YARP::YARP_sig
                          icub_firmware_shared::embobj)
                                        
   target_include_directories(embObjMultipleFTsensorsUT PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  endif()

endif()

