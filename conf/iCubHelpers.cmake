# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Macros to automate common tasks.
# 

# Export library to be used from external programs
# @param name: name of the target to be export
# @param public_inc_dirs: the directory that contains public header files, copied verbatim at 
# install time to INSTALL_PREFIX/include/iCub
# 
# Variables created: 
# ${name}_INCLUDE_DIRS: cache variable containing header files directory
# ${name}_LIBRARIES: name of the target library to be linked against
#
# -add ${name} to group icub_targets
# -append ${name} to our own local list of exported targets ICUB_TARGETS
# -install rule to copy all header files in public_include_dir
# -append export rules to a file in ${PROJECT_BINARY_DIR}
macro(icub_export_library name public_inc_dirs)

set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

#set(${name}_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include" CACHE STRING "include directories")
#set(${name}_LIBRARIES ${name} CACHE STRING "library")

set_property(GLOBAL PROPERTY ${name}_INCLUDE_DIRS  ${PROJECT_SOURCE_DIR}/include)
set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${name})

#set(ICUB_TARGETS ${ICUB_TARGETS} "${name}" CACHE STRING "" FORCE)

install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
install(DIRECTORY "${PROJECT_SOURCE_DIR}/include/${public_inc_dirs}" DESTINATION include/iCub
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})

endmacro(icub_export_library)
