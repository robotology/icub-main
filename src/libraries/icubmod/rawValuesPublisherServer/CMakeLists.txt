# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.


yarp_prepare_plugin(rawValuesPublisherServer
                    CATEGORY device
                    TYPE RawValuesPublisherServer
                    INCLUDE RawValuesPublisherServer.h
                    EXTRA_CONFIG
                    WRAPPER=rawValuesPublisherServer
                    DEFAULT OFF
                    GENERATE_PARSER)

if(ENABLE_rawValuesPublisherServer)
  option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Generate the param parser for rawValuesPublisherServer device" OFF)
  yarp_add_plugin(rawValuesPublisherServer)

  target_sources(rawValuesPublisherServer PRIVATE RawValuesPublisherServer.cpp
                                              RawValuesPublisherServer.h
                                              RawValuesPublisherServer_ParamsParser.cpp
                                              RawValuesPublisherServer_ParamsParser.h)

  target_link_libraries(rawValuesPublisherServer PRIVATE YARP::YARP_os
                                                      YARP::YARP_dev
                                                      ICUB::iCubDev)
                                                      
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                      YARP_dev)

  yarp_install(TARGETS rawValuesPublisherServer 
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET rawValuesPublisherServer PROPERTY FOLDER "Plugins/Device")
endif()