# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Luca Tricerri
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(embObjBattery CATEGORY device
                                 TYPE yarp::dev::embObjBattery
                                 INCLUDE embObjBattery.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

if (NOT SKIP_embObjBattery)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")


  include_directories(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjBattery embObjBattery.cpp embObjBattery.h)
  target_link_libraries(embObjBattery ethResources
                        YARP::YARP_os YARP::YARP_dev YARP::YARP_sig
                        icub_firmware_shared::embot)
  target_include_directories(embObjBattery PUBLIC "$<BUILD_INTERFACE:${YARP_COMPAT_SHIM_INCLUDE_DIR}>")
  icub_export_plugin(embObjBattery)

  yarp_install(TARGETS embObjBattery
        COMPONENT Runtime
        LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
        ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
        YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  if (BUILD_TESTING)
   add_library(embObjBatteryUT STATIC embObjBattery.cpp embObjBattery.h)
    target_link_libraries(embObjBatteryUT ethResources
                          YARP::YARP_os YARP::YARP_dev YARP::YARP_sig
                          icub_firmware_shared::embobj
                          icub_firmware_shared::embot)
   target_include_directories(embObjBatteryUT PUBLIC "$<BUILD_INTERFACE:${YARP_COMPAT_SHIM_INCLUDE_DIR}>")
   target_include_directories(embObjBatteryUT PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")
  endif()

endif()

