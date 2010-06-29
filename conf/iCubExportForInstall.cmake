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
#
# In the template the variable $ICUB_INCLUDE_DIRS is substituted with the 
# correct value which is a combination of:
# - ${CMAKE_INSTALL_PREFIX}/include (this is the general include directory where header files 
# are installed 
# - a list of external directories, as specified in the target's EXTERNAL_INCLUDE_DIRS property
# set in icub_export_library inside iCubHelpers.cmake
#
#
#### prepare config file for installation
message(STATUS "Now exporting targets for installed builds: ${ICUB_TARGETS}")

get_property(ICUB_TARGETS GLOBAL PROPERTY ICUB_TARGETS)

set(EXPORT_CONFIG_FILE icub-export-install.cmake)
set(INSTALL_CONFIG_FILE icub-config-for-install.cmake)
set(INSTALL_CONFIG_TEMPLATE "conf/template/icub-config-install.cmake.in")

set(ICUB_MODULE_PATH ${CMAKE_INSTALL_PREFIX}/share/iCub/cmake)

set(include_dirs "")
foreach (t ${ICUB_TARGETS})
  get_property(target_INCLUDE_DIRS TARGET ${t} PROPERTY EXTERNAL_INCLUDE_DIRS)
  get_property(target_INCLUDE_DIRS TARGET ${t} PROPERTY INCLUDE_DIRS)
  
   set(include_dirs ${include_dirs} ${target_INCLUDE_DIRS}) 

   if (ICUB_VERBOSE)
   message(STATUS "Header files for ${t}: ${target_INCLUDE_DIRS}")
   endif()
endforeach(t)

if(include_dirs)
   LIST(REMOVE_DUPLICATES include_dirs)
endif(include_dirs)

set(ICUB_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include ${include_dirs})
#message(STATUS "Header files global directory: ${ICUB_INCLUDE_DIRS}")

install(EXPORT icub-targets DESTINATION lib/ICUB FILE ${EXPORT_CONFIG_FILE})
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/${INSTALL_CONFIG_TEMPLATE}
  ${CMAKE_BINARY_DIR}/${INSTALL_CONFIG_FILE} @ONLY IMMEDIATE)

install(FILES ${CMAKE_BINARY_DIR}/${INSTALL_CONFIG_FILE} DESTINATION lib/ICUB RENAME icub-config.cmake)




