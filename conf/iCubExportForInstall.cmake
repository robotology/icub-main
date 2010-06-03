# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Code to export the installd project
# 
# Files: 
# EXPORT_CONFIG_FILE: in build directory, store libraries and dependenecies
# automatically generated with CMake export command.
# INSTALL_CONFIG_FILE: tmp file, it is generated from a template and will 
# become the real icub-config.cmake that is installed in CMAKE_INSTALL_PREFIX
# INSTALL_CONFIG_TAMPLATEE: INSTALL_CONFIG_FILE template

#### prepare config file for installation
get_property_fix(ICUB_TARGETS GLOBAL PROPERTY ICUB_TARGETS)

set(EXPORT_CONFIG_FILE icub-export-install.cmake)
set(INSTALL_CONFIG_FILE icub-config-for-install.cmake)
set(INSTALL_CONFIG_TEMPLATE "conf/template/icub-config-install.cmake.in")

set(ICUB_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)

install(EXPORT icub-targets DESTINATION ./ FILE ${EXPORT_CONFIG_FILE})
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/${INSTALL_CONFIG_TEMPLATE}
  ${CMAKE_BINARY_DIR}/${INSTALL_CONFIG_FILE} @ONLY IMMEDIATE)

install(FILES ${CMAKE_BINARY_DIR}/${INSTALL_CONFIG_FILE} DESTINATION ./ RENAME icub-config.cmake)




