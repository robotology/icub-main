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

#icub_export_library2(target INTERNAL_INCLUDE_DIRS dir EXTERNAL_INCLUDE_DIRS dir DEPENDS targets DESTINATION dest FILES files)
MACRO(icub_export_library2 target)
  PARSE_ARGUMENTS(${target}
    "INTERNAL_INCLUDE_DIRS;EXTERNAL_INCLUDE_DIRS;DEPENDS;DESTINATION;FILES"
    "VERBOSE"
    ${ARGN}
    )
 
  set(VERBOSE ${${target}_VERBOSE})
  if(VERBOSE)
    MESSAGE(STATUS "*** Arguments for ${target}")
    MESSAGE(STATUS "Internal directories: ${${target}_INTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "External directories: ${${target}_EXTERNAL_INCLUDE_DIRS}")
    MESSAGE(STATUS "Dependencies: ${${target}_DEPENDS}")
    MESSAGE(STATUS "Destination: ${${target}_DESTINATION}")
    MESSAGE(STATUS "Header files: ${${target}_FILES}")
    MESSAGE(STATUS "Option verbosity: ${${target}_VERBOSE}")
  endif()
  
  set(internal_includes ${${target}_INTERNAL_INCLUDE_DIRS})
  set(external_includes ${${target}_EXTERNAL_INCLUDE_DIRS})
  set(dependencies ${${target}_DEPENDS})
  set(files ${${target}_FILES})
  set(destination ${${target}_DESTINATION})
  
  set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

  icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${target})
  install(TARGETS ${target} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
  export(TARGETS ${target} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})
           
  ############ Parsing dependencies
  if (dependencies)
    set_target_properties(${target} PROPERTIES 
                        DEPENDS
                        ${dependencies})                           
    
    foreach (d ${dependencies})
        get_target_property(in_dirs ${d} INTERNAL_INCLUDE_DIRS)
        get_target_property(ext_dirs ${d} EXTERNAL_INCLUDE_DIRS)
        
        if (VERBOSE)
            message(STATUS "FROM ${d}:")
            message(STATUS "${in_dirs}")
            message(STATUS "${ext_dirs}")
        endif()
        
        set(internal_includes ${internal_includes} ${in_dirs})
        set(external_includes ${external_includes} ${ext_dirs})
    endforeach(d)
    
    endif()
  
  set(include_dirs "")
  
  if (internal_includes)
    set(include_dirs ${include_dirs} ${internal_includes})
    set_target_properties(${target} PROPERTIES 
                        INTERNAL_INCLUDE_DIRS
                        "${internal_includes}")
  endif()
  if (external_includes)
    set(include_dirs ${include_dirs} ${external_includes})
    set_target_properties(${target} PROPERTIES 
                        EXTERNAL_INCLUDE_DIRS
                        "${external_includes}")   
  endif()
  
  if (include_dirs)
    set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
    message(STATUS "Target ${target} exporting: ${include_dirs}")
    set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories")
  endif()

  if (files AND destination)
    message(STATUS "Target ${target} installing ${files} to ${destination}")
    install(FILES ${files} DESTINATION ${destination})

    set_target_properties(${target} PROPERTIES 
                        HEADERFILES
                        "${files}")
                        
    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()
 
ENDMACRO(icub_export_library2)

MACRO(PARSE_ARGUMENTS prefix arg_names option_names)
  SET(DEFAULT_ARGS)
  FOREACH(arg_name ${arg_names})    
    SET(${prefix}_${arg_name})
  ENDFOREACH(arg_name)
  FOREACH(option ${option_names})
    SET(${prefix}_${option} FALSE)
  ENDFOREACH(option)

  SET(current_arg_name DEFAULT_ARGS)
  SET(current_arg_list)
  FOREACH(arg ${ARGN})            
    SET(larg_names ${arg_names})    
    LIST(FIND larg_names "${arg}" is_arg_name)                   
    IF (is_arg_name GREATER -1)
      SET(${prefix}_${current_arg_name} ${current_arg_list})
      SET(current_arg_name ${arg})
      SET(current_arg_list)
    ELSE (is_arg_name GREATER -1)
      SET(loption_names ${option_names})    
      LIST(FIND loption_names "${arg}" is_option)            
      IF (is_option GREATER -1)
	     SET(${prefix}_${arg} TRUE)
      ELSE (is_option GREATER -1)
	     SET(current_arg_list ${current_arg_list} ${arg})
      ENDIF (is_option GREATER -1)
    ENDIF (is_arg_name GREATER -1)
  ENDFOREACH(arg)
  SET(${prefix}_${current_arg_name} ${current_arg_list})
ENDMACRO(PARSE_ARGUMENTS)

# macro(icub_export_library2 param)

# message("Debugging icub_export_library2: ${expect} head: ${param} tail: ${ARGV}")

# if (NOT expect)
    # #Reset variables
    # set(target ${param})
    # set(expect EXPECT_INTERNAL_INCLUDE_DIRS)
    # set_target_properties(${target} PROPERTIES 
                        # INTERNAL_INCLUDE_DIRS "" EXTERNAL_INCLUDE_DIRS "" HEADERS_DESTINATION "" FILES "")
 
    # ## check if called with no other parameters
    # if (${ARGC} EQUAL 1)
         # set(expect "END")
    # endif()
    
# elseif (expect STREQUAL "EXPECT_INTERNAL_INCLUDE_DIRS")
    # if (${param} STREQUAL "INTERNAL_INCLUDE_DIRS")
        # set(expect INTERNAL_INCLUDE_DIRS)
    # else()
        # #skip optional parameter INTERNAL_INCLUDE_DIRS
        # message(STATUS "Skipping optional parameter INTERNAL_INCLUDE_DIRS")
        # set(expect EXPECT_EXTERNAL_INCLUDE_DIRS)
    # endif()
# elseif (expect STREQUAL "INTERNAL_INCLUDE_DIRS")
    # set(expect EXPECT_EXTERNAL_INCLUDE_DIRS)
    # set_target_properties(${target} PROPERTIES 
                        # INTERNAL_INCLUDE_DIRS
                        # ${param})
# elseif (expect STREQUAL "EXPECT_EXTERNAL_INCLUDE_DIRS")
    # if (${param} STREQUAL "EXTERNAL_INCLUDE_DIRS")
        # set(expect EXTERNAL_INCLUDE_DIRS)
    # else()
        # #skipping optional parameter EXTERNAL_INCLUDE_DIRS
        # message(STATUS "Skipping optional parameter EXTERNAL_INCLUDE_DIRS")
        # set(expect EXPECT_DESTINATION)
    # endif()
    
# elseif (expect STREQUAL "EXTERNAL_INCLUDE_DIRS")
    # set_target_properties(${target} PROPERTIES 
                        # EXTERNAL_INCLUDE_DIRS
                        # ${param})
    # set(expect EXPECT_DESTINATION)
# elseif (expect STREQUAL "EXPECT_DESTINATION")
    # if (${param} STREQUAL "DESTINATION")
        # set(expect DESTINATION)
    # else()
        # message(STATUS  "Skipping optional parameter DESTINATION")
        # set(expect "END")
    # endif()
# elseif (expect STREQUAL "DESTINATION")     
    # set(expect EXPECT_FILES)
    # set_target_properties(${target} PROPERTIES 
                        # HEADERS_DESTINATION
                        # ${param})
# elseif (expect STREQUAL "EXPECT_FILES")
    # if (${param} STREQUAL "FILES")
        # #message(STATUS "--> ${ARGC}")
        # #if (${ARGC} EQUAL 2)
        # #    set(expect FILES)
        # #else()
            # #set(expect FILES)
            # message(STATUS ${ARGN})
            # set_target_properties(${target} PROPERTIES 
                        # FILES "${ARGN}")
                        
            # set(expect "END")
        # #endif()
    # else()
       # message(FATAL "ERROR: mandatory field FILES")
    # endif()
# elseif (expect STREQUAL "FILES")
    # set_target_properties(${target} PROPERTIES FILES ${param})
    # set(expect "END")  
# endif(NOT expect)

# if (expect STREQUAL "END")

    # set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

    # icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${target})
    # install(TARGETS ${target} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib EXPORT icub-targets)
    # export(TARGETS ${name} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})
    
    # get_target_property(internal_includes ${target} INTERNAL_INCLUDE_DIRS)
    # get_target_property(external_includes ${target} EXTERNAL_INCLUDE_DIRS)
    # get_target_property(header_files ${target} FILES)
    # get_target_property(destination ${target} HEADERS_DESTINATION)
    
    # if (internal_includes)
       # set(include_dirs ${include_dirs} ${internal_includes})
     # endif()
    # if (external_includes)
       # set(include_dirs ${include_dirs} ${external_includes})
    # endif()

    # if (include_dirs)
        # set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
        # message(STATUS "Target ${target} exporting: ${include_dirs}")
        # set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories")
    # endif()

    # if (header_files)
        # message(STATUS "Target ${target} installing ${header_files} to ${destination}")
        # install(FILES ${header_files} DESTINATION ${destination})
    # endif()

# else(expect STREQUAL "END")
    # #pass call forward
    # icub_export_library2(${ARGN})
# endif(expect STREQUAL "END")
                     
# endmacro(icub_export_library2)

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

