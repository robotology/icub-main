# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Luca Tricerri
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(embObjMultipleFTsensor CATEGORY device
                                 TYPE yarp::dev::embObjMultipleFTsensor
                                 INCLUDE embObjMultipleFTsensor.h
                                 EXTRA_CONFIG WRAPPER=analogServer)

IF (NOT SKIP_embObjMultipleFTsensor)


  set(ICUB_COMPILE_EMBOBJ_LIBRARY ON CACHE INTERNAL "use the embObjLib lib")


  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

  yarp_add_plugin(embObjMultipleFTsensor embObjMultipleFTsensor.cpp embObjMultipleFTsensor.h)
  TARGET_LINK_LIBRARIES(embObjMultipleFTsensor ethResources)
  icub_export_plugin(embObjMultipleFTsensor)

  yarp_install(TARGETS embObjMultipleFTsensor
        COMPONENT Runtime
        LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
        ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR}
        YARP_INI DESTINATION ${ICUB_PLUGIN_MANIFESTS_INSTALL_DIR})

  if (UNITTEST_COMPILE)
   add_library(embObjMultipleFTsensorUT STATIC embObjMultipleFTsensor.cpp embObjMultipleFTsensor.h)    
   target_link_libraries(embObjMultipleFTsensorUT ethResources
                                    YARP::YARP_dev
                                    icub_firmware_shared::embobj
                                    ACE::ACE)    
   target_include_directories(embObjMultipleFTsensorUT PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${LIBRARY_TARGET_NAME}>")
  endif()

ENDIF (NOT SKIP_embObjMultipleFTsensor)

