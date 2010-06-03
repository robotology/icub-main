# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Code to export the project build tree. Note this is similar to the code 
# that export the installed project but is also different in important aspects.
# For this reason the code is more complicated and produces separate files.
#
# This produces a file called icub-config.cmake in ${CMAKE_BINARY_DIR} that 
# other projects should execute (with find_package) to use the project.
#
# This files assumes that each target you want to export has previously 
# called icub_export_library.
#
# Files: 
# EXPORT_INCLUDE_FILE: this file contains a list of all the targets in the 
# build, and a corresponding include directories. This is generated from 
# information in property (mainly ICUB_TARGETS); this is populated
# by each target by calling for example icub_export_library, see iCubHelpers.cmake).
# 
# EXPORT_CONFIG_FILE: in build directory, store libraries and dependenecies
# automatically generated with CMake export command.
# 
# BUILD_CONFIG_TEMPLATE: template for the real config file (icub-config.cmake)
# that exports the build tree
#

###################################
get_property(ICUB_TARGETS GLOBAL PROPERTY ICUB_TARGETS)

message(STATUS "Now exporting targets: ${ICUB_TARGETS}")
set(EXPORT_INCLUDE_FILE icub-export-build-includes.cmake)
set(EXPORT_CONFIG_FILE icub-export-build.cmake)
set(BUILD_CONFIG_TEMPLATE "conf/template/icub-config-build-tree.cmake.in")
file(APPEND ${CMAKE_BINARY_DIR}/${EXPORT_INCLUDE_FILE} "###################\n")
file(APPEND ${CMAKE_BINARY_DIR}/${EXPORT_INCLUDE_FILE} "# List of include directories for exported targets\n\n")
set(include_dirs "")
foreach (t ${ICUB_TARGETS})
  get_property_fix(target_INCLUDE_DIRS TARGET ${t} PROPERTY INCLUDE_DIRS)
  file(APPEND ${CMAKE_BINARY_DIR}/${EXPORT_INCLUDE_FILE} "set(${t}_INCLUDE_DIRS ${target_INCLUDE_DIRS} CACHE STRING \"include dir for target ${t}\")\n")

  set(include_dirs ${include_dirs} ${target_INCLUDE_DIRS})
endforeach(t)

file(APPEND ${CMAKE_BINARY_DIR}/${EXPORT_INCLUDE_FILE} "set(ICUB_INCLUDE_DIRS \"${include_dirs}\" CACHE STRING \"list of include directories, all exported targets\")\n")

CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/${BUILD_CONFIG_TEMPLATE}
  ${CMAKE_BINARY_DIR}/icub-config.cmake @ONLY IMMEDIATE)
