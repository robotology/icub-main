# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Macros to automate common tasks.
# 

# Export library to be used from external programs
# @param name: name of the target to be export
# @param private_inc_dirs: directory that contains private header files
# @param public_inc_dirs: a list of header files that will be copied at 
# install time to INSTALL_PREFIX/include/iCub
#
# Creates a cache variable and property to store the target include directory.
#
# -add ${name} to group icub_targets
# -append ${name} to global property ICUB_TARGETS
# -create property INCLUDE_DIRS for target ${name} 
# -install rule to copy all header files to include/iCub
# -append export rules to a file in ${PROJECT_BINARY_DIR}

macro(icub_export_library name private_inc_dirs dest_dir)

set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)


set_property_fix(TARGET ${name} PROPERTY INCLUDE_DIRS  "${private_inc_dirs}")
set(${name}_INCLUDE_DIRS "${private_inc_dirs}" CACHE STRING "include directories")
#set(${name}_LIBRARIES ${name} CACHE STRING "library")
set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${name})

#set(ICUB_TARGETS ${ICUB_TARGETS} "${name}" CACHE STRING "" FORCE)

install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
install(FILES ${ARGN} DESTINATION ${dest_dir})

export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})

endmacro(icub_export_library)

### From yarp.
# Helper macro to work around a bug in set_property in cmake 2.6.0
MACRO(get_property_fix localname _global _property varname)
  set(_exists_chk)
  get_property(_exists_chk GLOBAL PROPERTY ${varname})
  if (_exists_chk)
    set(${localname} ${_exists_chk})
  else (_exists_chk)
    set(${localname})
  endif (_exists_chk)
ENDMACRO(get_property_fix)

# Helper macro to work around a bug in set_property in cmake 2.6.0
MACRO(set_property_fix _global _property _append varname)
  get_property(_append_chk GLOBAL PROPERTY ${varname})
  if (_append_chk)
    set_property(GLOBAL APPEND PROPERTY ${varname} ${ARGN})
  else (_append_chk)
    set_property(GLOBAL PROPERTY ${varname} ${ARGN})
  endif (_append_chk)
ENDMACRO(set_property_fix)
