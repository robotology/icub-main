# SPDX-FileCopyrightText: 2006-2025 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(rawValuesPublisherRemapper
  CATEGORY device
  TYPE RawValuesPublisherRemapper
  INCLUDE RawValuesPublisherRemapper.h
  EXTRA_CONFIG
  DEFAULT OFF
  GENERATE_PARSER
)

if(ENABLE_rawValuesPublisherRemapper)
  yarp_add_plugin(rawValuesPublisherRemapper)

  target_sources(rawValuesPublisherRemapper
    PRIVATE
    RawValuesPublisherRemapper.cpp
    RawValuesPublisherRemapper.h
    RawValuesPublisherRemapper_ParamsParser.cpp
    RawValuesPublisherRemapper_ParamsParser.h
  )

  target_link_libraries(rawValuesPublisherRemapper
    PRIVATE
    YARP::YARP_os
    YARP::YARP_dev
    ICUB::iCubDev
  )

  list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS
    YARP_os
    YARP_dev
  )

  yarp_install(
    TARGETS rawValuesPublisherRemapper
    EXPORT YARP_${YARP_PLUGIN_MASTER}
    COMPONENT ${YARP_PLUGIN_MASTER}
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
  )

  set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)

  set_property(TARGET rawValuesPublisherRemapper PROPERTY FOLDER "Plugins/Device")

  if(YARP_COMPILE_TESTS)
    add_subdirectory(tests)
  endif()

endif()