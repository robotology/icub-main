# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(couplingICubHandMk2
                    CATEGORY device
                    TYPE CouplingICubHandMk2
                    INCLUDE CouplingICubHandMk2.h
                    GENERATE_PARSER
                    DEFAULT ON)

if(ENABLE_couplingICubHandMk2)
  option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Generate the param parser for couplingICubHandMk2 device" OFF)
  yarp_add_plugin(yarp_couplingICubHandMk2)

  if(MSVC)
    add_definitions(-D_USE_MATH_DEFINES)
  endif()

  target_sources(yarp_couplingICubHandMk2 PRIVATE CouplingICubHandMk2.cpp
                                                  CouplingICubHandMk2.h
                                                  CouplingICubHandMk2_ParamsParser.cpp
                                                  CouplingICubHandMk2_ParamsParser.h)

  target_link_libraries(yarp_couplingICubHandMk2 PRIVATE YARP::YARP_os
                                                         YARP::YARP_dev)
  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                      YARP_dev)

  yarp_install(TARGETS yarp_couplingICubHandMk2
               EXPORT YARP_${YARP_PLUGIN_MASTER}
               COMPONENT ${YARP_PLUGIN_MASTER}
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
               YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET yarp_couplingICubHandMk2 PROPERTY FOLDER "Plugins/Device")
endif()
