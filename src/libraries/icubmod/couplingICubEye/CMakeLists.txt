# Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

yarp_prepare_plugin(couplingICubEye
                    CATEGORY device
                    TYPE CouplingICubEye
                    INCLUDE CouplingICubEye.h
                    GENERATE_PARSER
                    DEFAULT ON)

if(ENABLE_couplingICubEye)
    option(ALLOW_DEVICE_PARAM_PARSER_GENERATION "Generate the param parser for couplingICubEye device" OFF)
    yarp_add_plugin(yarp_couplingICubEye)

    if(MSVC)
        add_definitions(-D_USE_MATH_DEFINES)
    endif()

    target_sources(yarp_couplingICubEye PRIVATE CouplingICubEye.cpp
                                                CouplingICubEye.h
                                                CouplingICubEye_ParamsParser.cpp
                                                CouplingICubEye_ParamsParser.h)

    target_link_libraries(yarp_couplingICubEye PRIVATE   YARP::YARP_os
                                                            YARP::YARP_dev)
    list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
                                                        YARP_dev)

    yarp_install(TARGETS yarp_couplingICubEye
                EXPORT YARP_${YARP_PLUGIN_MASTER}
                COMPONENT ${YARP_PLUGIN_MASTER}
                LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
                YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

    set_property(TARGET yarp_couplingICubEye PROPERTY FOLDER "Plugins/Device")
endif()