# Copyright (C) 2006-2020 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

include(InstallBasicPackageFiles)

macro(ICUB_INSTALL_BASIC_PACKAGE_FILES _export)
  set(_options )
  set(_oneValueArgs FIRST_TARGET
                    STATIC_CONFIG_TEMPLATE
                    COMPONENT)
  set(_multiValueArgs DEPENDENCIES
                      PRIVATE_DEPENDENCIES)
  cmake_parse_arguments(_YIBPF "${_options}" "${_oneValueArgs}" "${_multiValueArgs}" ${ARGN})

  unset(_component)
  if(NOT DEFINED _YIBPF_COMPONENT)
    set(_YIBPF_COMPONENT ${_export}-dev)
  endif()

  unset(_deps)
  foreach(_dep ${_YIBPF_DEPENDENCIES})
      list(APPEND _deps "${_dep}")
  endforeach()

  unset(_priv_deps)
  foreach(_dep ${_YIBPF_PRIVATE_DEPENDENCIES})
      list(APPEND _priv_deps "${_dep}")
  endforeach()

  unset(_config_template)
  if(NOT BUILD_SHARED_LIBS AND DEFINED _YIBPF_STATIC_CONFIG_TEMPLATE)
    set(_config_template CONFIG_TEMPLATE "${_YIBPF_STATIC_CONFIG_TEMPLATE}")
  endif()

  install_basic_package_files(${_export}
                              ${_config_template}
                              VERSION ${${ROOT_PROJECT_NAME}_VERSION}
                              COMPATIBILITY SameMajorVersion
                              EXPORT_DESTINATION "${CMAKE_BINARY_DIR}/${_export}"
                              INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/${_export}"
                              DEPENDENCIES ${_deps}
                              PRIVATE_DEPENDENCIES ${_priv_deps}
                              NO_CHECK_REQUIRED_COMPONENTS_MACRO
                              NO_SET_AND_CHECK_MACRO
                              COMPONENT ${_YIBPF_COMPONENT}
                              NAMESPACE ICUB::
                              UPPERCASE_FILENAMES
                              ${_YIBPF_UNPARSED_ARGUMENTS})
endmacro()
