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


set_property(TARGET ${name} PROPERTY INCLUDE_DIRS  "${private_inc_dirs}")
set(${name}_INCLUDE_DIRS "${private_inc_dirs}" CACHE STRING "include directories")
#set(${name}_LIBRARIES ${name} CACHE STRING "library")
icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${name})

#set(ICUB_TARGETS ${ICUB_TARGETS} "${name}" CACHE STRING "" FORCE)

install(TARGETS ${name} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
install(FILES ${ARGN} DESTINATION ${dest_dir})

export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})

endmacro(icub_export_library)

#icub_export_library2(target INTREE_INCLUDES dir EXTERNAL_INCLUDES dir DESTINATION dest FILES files)
macro(icub_export_library2 param)

#message("Running with ${param}")

if (NOT expect)
    #Reset variables
    set(target ${param})
    set(expect EXPECT_INTREE_INCLUDES)
    set_target_properties(${target} PROPERTIES 
                        INTERNAL_INCLUDE_DIRS "" EXTERNAL_INCLUDE_DIRS "" HEADERS_DESTINATION "" FILES "")
elseif (expect STREQUAL "EXPECT_INTREE_INCLUDES")
    if (${param} STREQUAL "INTREE_INCLUDES")
        set(expect INTREE_INCLUDES)
    else()
        #skip optional parameter INTREE_INCLUDES
        message(STATUS "Skipping optional parameter INTREE_INCLUDES")
        set(expect EXPECT_EXTERNAL_INCLUDES)
    endif()
elseif (expect STREQUAL "INTREE_INCLUDES")
    set(expect EXPECT_EXTERNAL_INCLUDES)
    set_target_properties(${target} PROPERTIES 
                        INTERNAL_INCLUDE_DIRS
                        ${param})
elseif (expect STREQUAL "EXPECT_EXTERNAL_INCLUDES")
    if (${param} STREQUAL "EXTERNAL_INCLUDES")
        set(expect EXTERNAL_INCLUDES)
    else()
        #skipping optional parameter EXTERNAL_INCLUDES
        message(STATUS "Skipping optional parameter EXTERNAL_INCLUDES")
        set(expect EXPECT_DESTINATION)
    endif()
    
elseif (expect STREQUAL "EXTERNAL_INCLUDES")
    set_target_properties(${target} PROPERTIES 
                        EXTERNAL_INCLUDE_DIRS
                        ${param})
    set(expect EXPECT_DESTINATION)
elseif (expect STREQUAL "EXPECT_DESTINATION")
    if (${param} STREQUAL "DESTINATION")
        set(expect DESTINATION)
    else()
        message(STATUS  "Skipping optional parameter DESTINATION")
        set(expect "END")
    endif()
elseif (expect STREQUAL "DESTINATION")     
    set(expect EXPECT_FILES)
    set_target_properties(${target} PROPERTIES 
                        HEADERS_DESTINATION
                        ${param})
elseif (expect STREQUAL "EXPECT_FILES")
    if (${param} STREQUAL "FILES")
        #set(expect FILES)
        message(STATUS ${ARGN})
        set_target_properties(${target} PROPERTIES 
                        FILES "${ARGN}")
                        
        set(expect "END")

    else()
       message(FATAL "ERROR: mandatory field FILES")
    endif()
    
endif(NOT expect)

if (expect STREQUAL "END")

    set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

    icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${target})
    install(TARGETS ${target} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
    export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})
    
    get_target_property(internal_includes ${target} INTERNAL_INCLUDE_DIRS)
    get_target_property(external_includes ${target} EXTERNAL_INCLUDE_DIRS)
    get_target_property(header_files ${target} FILES)
    get_target_property(destination ${target} HEADERS_DESTINATION)
    
    if (internal_includes)
       set(include_dirs ${include_dirs} ${internal_includes})
    endif()
    if (external_includes)
       set(include_dirs ${include_dirs} ${external_includes})
    endif()

    if (include_dirs)
        icub_set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
        message(STATUS "Target ${target} exporting: ${include_dirs}")
        set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories")
    endif()

    if (header_files)
        message(STATUS "Target ${target} installing ${header_files} to ${destination}")
        install(FILES ${header_files} DESTINATION ${destination})
    endif()

else(expect STREQUAL "END")
    #pass call forward
    icub_export_library2(${ARGN})
endif(expect STREQUAL "END")
                     
endmacro(icub_export_library2)

### From yarp.
# Helper macro to work around a bug in set_property in cmake 2.6.0
MACRO(icub_get_property localname _global _property varname)
  set(_icub_exists_chk)
  get_property(_icub_exists_chk GLOBAL PROPERTY ${varname})
  if (_icub_exists_chk)
    set(${localname} ${_icub_exists_chk})
  else (_icub_exists_chk)
    set(${localname})
  endif (_icub_exists_chk)
ENDMACRO(icub_get_property)

# Helper macro to work around a bug in set_property in cmake 2.6.0
MACRO(icub_set_property _global _property _append varname)
  get_property(_icub_append_chk GLOBAL PROPERTY ${varname})
  if (_icub_append_chk)
    set_property(GLOBAL APPEND PROPERTY ${varname} ${ARGN})
  else (_icub_append_chk)
    set_property(GLOBAL PROPERTY ${varname} ${ARGN})
  endif (_icub_append_chk)
ENDMACRO(icub_set_property)

