# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 
# Macros to automate common tasks.
# 

# Export library to be used from external programs
# @param name: name of the target to be export
# @param headers: directory that contains public include files
# @param header_files: public header files (to be installed)
# 
# Variables created: 
# ${name}_INCLUDE_DIRS: cache variable containing header files directory
# ${name}_LIBRARIES: name of the target library to be linked against
#
# -add ${name} to group icub_targets
# -append ${name} to our own local list of exported targets ICUB_TARGETS
# -install rule to copy the list of ${header_files} to include/iCub postfix
# -append export rules to a file in ${PROJECT_BINARY_DIR}
macro(icub_export_library name headers header_files)

set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

set(${name}_INCLUDE_DIRS ${headers} CACHE STRING "include directories")
set(${name}_LIBRARIES ${name} CACHE STRING "library")

set(ICUB_TARGETS ${ICUB_TARGETS} "${name}" CACHE STRING "" FORCE)

install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
install(FILES ${header_files} DESTINATION include/iCub)
export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})

endmacro(icub_export_library)
