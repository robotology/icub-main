# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Macros to automate common tasks.
#

# Better export library function. Export a target to be used from external programs
#
# icub_export_library(target
#                       [INTERNAL_INCLUDE_DIRS dir1 dir2 ...]
#                       [EXTERNAL_INCLUDE_DIRS dir1 dir2 ...]
#                       [DEPENDS target1 target2 ...]
#                       [DESTINATION dest]
#                       [VERBOSE]
#                       [FILES file1 file2 ...])
# - target: target name
# - INTERNAL_INCLUDE_DIRS a list of directories that contain header files when building in-source
# - EXTERNAL_INCLUDE_DIRS a list of directories that contain header files external of the repository
# - DEPENDS a list of dependencies; these are targets built within the repository. Important CMake should
#   parse these targets *before* the current target (check sub_directories(...)).
# - VERBOSE: ask to print parameters (for debugging)
# - DESTINATION: destination directory to which header files will be copied (relative w.r.t. install prefix)
# - FILES: a list of files that will be copied to destination (header files)
#
# The function does a bunch of things:
#
# -append ${target} to the list of targetes built within the project (global property ICUB_TARGETS)
# -retrieve INTERNAL_INCLUDE_DIRS/EXTERNAL_INCLUDE_DIRS properties for each dependency
# -build INTERNAL_INCLUDE_DIRS by merging INTERNAL_INCLUDE_DIRS and the property INTERNAL_INCLUDE_DIRS of each
#  dependency target -- store it as a property for the current target
# -creates a DEPENDS property for the target, this contains the list of dependencies
# -similarly as above for EXTERNAL_INCLUDE_DIRS
# -merge EXTERNAL/INTERNAL_INCLUDE_DIRS into INCLUDE_DIRS for the current target, store it as property and cache
#  variable
# -set up install rule for copying all FILES to DESTINATION
# -append export rules for target to a ICUB_EXPORTBUILD_FILE in ${PROJECT_BINARY_DIR}
#
# Note: this function has to be called by all targets. cmake  requires that INSTALL
# is called when targets are created (or in the same CMake list file) so it was not possible
# to just callect target names and call INSTALL for each of them at the end of the build.
#

MACRO(icub_export_library target)
  PARSE_ARGUMENTS(${target}
    "INTERNAL_INCLUDE_DIRS;EXTERNAL_INCLUDE_DIRS;DEPENDS;DESTINATION;FILES;FILES_WITH_PATH;PATH_TO_EXCLUDE"
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
    MESSAGE(STATUS "Header files for which we keep the relative path: ${${target}_FILES_WITH_PATH}")
    MESSAGE(STATUS "Part of the relative path to strip off: ${${target}_PATH_TO_EXCLUDE}")
    MESSAGE(STATUS "Option verbosity: ${${target}_VERBOSE}")
  endif()

  set(internal_includes ${${target}_INTERNAL_INCLUDE_DIRS})
  set(external_includes ${${target}_EXTERNAL_INCLUDE_DIRS})
  set(dependencies ${${target}_DEPENDS})
  set(files ${${target}_FILES})
  set(files_with_path ${${target}_FILES_WITH_PATH})
  set(path_to_exclude ${${target}_PATH_TO_EXCLUDE})
  set(destination ${${target}_DESTINATION})

  set(ICUB_EXPORTBUILD_FILE icub-export-build.cmake)

  ##### Append target to global list.
  icub_set_property(GLOBAL APPEND PROPERTY ICUB_TARGETS ${target})
  # Install/export rules
  install(TARGETS ${target}
          EXPORT icub-targets
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
          COMPONENT Development)
  if (MSVC) 
    install ( FILES ${CMAKE_BINARY_DIR}/lib/Debug/${target}d.pdb
              DESTINATION lib
              CONFIGURATIONS Debug
              COMPONENT Development
              OPTIONAL)
  endif()
  export(TARGETS ${target} APPEND FILE ${CMAKE_BINARY_DIR}/${ICUB_EXPORTBUILD_FILE})
    #important wrap ${dependencies} with "" to allows storing a list of dependencies
  set_target_properties(${target} PROPERTIES DEPENDS "${dependencies}")

  ##### Handle include directories
  # Parsing dependencies
  if (dependencies)
    foreach (d ${dependencies})
        get_target_property(in_dirs ${d} INTERNAL_INCLUDE_DIRS)
        get_target_property(ext_dirs ${d} EXTERNAL_INCLUDE_DIRS)

        if (VERBOSE)
            message(STATUS "Getting from target ${d}:")
            message(STATUS "${in_dirs}")
            message(STATUS "${ext_dirs}")
        endif()

        if (in_dirs)
            set(internal_includes ${internal_includes} ${in_dirs})
        endif (in_dirs)

        if (ext_dirs)
            set(external_includes ${external_includes} ${ext_dirs})
        endif(ext_dirs)
    endforeach(d)
  endif(dependencies)
  ############################

  ################ Build unique variable with internal and external include directories
  ## Set corresponding target's properties
  set(include_dirs "")

  if (internal_includes)
    list(REMOVE_DUPLICATES internal_includes)
    set_target_properties(${target} PROPERTIES
                        INTERNAL_INCLUDE_DIRS
                        "${internal_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting internal headers: ${internal_includes}")
    endif()
    set(include_dirs ${include_dirs} ${internal_includes})
  endif()

  if (external_includes)
    list(REMOVE_DUPLICATES external_includes)
    set_target_properties(${target} PROPERTIES
                        EXTERNAL_INCLUDE_DIRS
                        "${external_includes}")
    if(VERBOSE)
        message(STATUS "Target ${target} exporting external headers: ${external_includes}")
    endif()
    set(include_dirs ${include_dirs} ${external_includes})
  endif()

  if (include_dirs)
    list(REMOVE_DUPLICATES include_dirs)
    set_property(TARGET ${target} PROPERTY INCLUDE_DIRS  "${include_dirs}")
    if (VERBOSE)
        message(STATUS "Target ${target} exporting: ${include_dirs}")
    endif()
    set(${target}_INCLUDE_DIRS "${include_dirs}" CACHE STRING "include directories target" FORCE)
  endif()
  ##############################################

  # Compile libraries using -fPIC to produce position independent code
  # For CMAKE_VERSION >= 2.8.10 this is handled in iCubOptions.cmake
  # using the CMAKE_POSITION_INDEPENDENT_CODE flag
  if(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS)
    if(CMAKE_VERSION VERSION_EQUAL "2.8.9")
      set_target_properties(${target} PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
    elseif(CMAKE_VERSION VERSION_LESS "2.8.9")
      set_target_properties(${target} PROPERTIES COMPILE_FLAGS -fPIC)
    endif()
  endif()

  #### Export rules
  if (files AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files} to ${destination}")
    endif()
    install(FILES ${files} DESTINATION ${destination} COMPONENT Development)

    set_target_properties(${target} PROPERTIES
                        HEADERFILES
                        "${files}")

    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()

  #### Export rules for files_with_path case
  if (files_with_path AND destination)
    if (VERBOSE)
        message(STATUS "Target ${target} installing ${files_with_path} to ${destination}")
    endif()
   

    if(path_to_exclude)
        # strip off the trailing slash
        string(REGEX REPLACE "/+$" "" path_to_exclude ${path_to_exclude})
    endif()

    foreach(cur_file  ${files_with_path})
        get_filename_component(file_rel_dir ${cur_file} PATH)
        if(path_to_exclude)
            string(REPLACE "${path_to_exclude}" "" file_rel_dir ${file_rel_dir})
        endif()
        install(FILES ${cur_file} DESTINATION ${destination}/${file_rel_dir} COMPONENT Development)
    endforeach()
    set_target_properties(${target} PROPERTIES 
                        HEADERFILES
                        "${files_with_path}")
                        
    set_target_properties(${target} PROPERTIES
                        HEADERS_DESTINATION
                        ${destination})
  endif()

ENDMACRO(icub_export_library)


macro(icub_export_plugin target)
    get_target_property(_type ${target} TYPE)
    if("${_type}" STREQUAL "MODULE_LIBRARY")
        yarp_install(TARGETS ${target}
                     EXPORT icub-targets
                     COMPONENT Runtime
                     LIBRARY DESTINATION ${ICUB_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ICUB_STATIC_PLUGINS_INSTALL_DIR})
    else()
        icub_export_library(${target})
    endif()
endmacro()


#
# Taken from kitware wiki, easy support for macro with variable parameters.
# See icub_export_library2 for usage.
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

##
# Declare a new application rule.
#
# icub_app(target)
# - target: target name this name, prefixed with app will be the target for
# make or the vs gui.
macro(icub_app target)
    set(dummy ${CMAKE_BINARY_DIR}/f-${target})
	file(TO_CMAKE_PATH "${ICUB_APPLICATIONS_PREFIX}" app_prefix)

	set(cmakefile ${CMAKE_BINARY_DIR}/app-${target}.cmake)
    
    set(BUFFER "###### This file is automatically generated by CMake.\n")

    set(BUFFER "${BUFFER}## see macro icub_app defined in conf/iCubHelpers.cmake\n\n")
	set(BUFFER "${BUFFER}execute_process(COMMAND \"\${CMAKE_COMMAND}\" -E make_directory \"${app_prefix}\")\n")
	set(BUFFER "${BUFFER}execute_process(COMMAND \"\${CMAKE_COMMAND}\" -E make_directory \"${app_prefix}/app\")\n")
	
    file(WRITE ${cmakefile} "${BUFFER}")
    
    add_custom_target(app-${target} DEPENDS ${dummy})

    icub_set_property(GLOBAL APPEND PROPERTY ICUB_APPLICATIONS app-${target})
    icub_set_property(GLOBAL APPEND PROPERTY ICUB_APPLICATIONS_FILES ${dummy})

	add_custom_command(OUTPUT ${dummy}
					   COMMAND ${CMAKE_COMMAND} -P ${cmakefile}
					   COMMENT "Setting up app-${target}")

#    add_custom_command(OUTPUT ${dummy}
#                    COMMAND ${CMAKE_COMMAND} -E make_directory ${dapp}
#                    COMMENT "Creating directory ${target}")
endmacro(icub_app)

# Add install rule for applications.
#
# icub_app_install(target
#                  [DESTINATION dest]
#                  [VERBOSE]
#                  [ROOT]
#                  [FILES file1 file2 ...])
# - target: name of the command, previously defined with the icub_app command
# - DESTINATION: destination directory for files
# - VERBOSE: print verbose
# - ROOT: whether to use app/${target} as prefix or not
# - FILES: files to be copied
#
# By default FILES will be copied to ${ICUB_INSTALL_PREFIX}/app/${target}/DESTINATION
# if you specify ROOT, files will be copied to ${ICUB_INSTALL_PREFIX}.
#
# Jan 2011: converted the macro to be a function, to avoid clashing with cmake variables defined outside.
# Hopefully functions have local scope and will prevent clashes to happen in the future.
#
# April 2011: added FORCE_UPDATE flag to force update of files

function(icub_app_install target)
  PARSE_ARGUMENTS(${target}
	"FILES;DESTINATION"
    "VERBOSE;ROOT;FORCE_UPDATE"
    ${ARGN}
    )

  set(dummy ${CMAKE_BINARY_DIR}/f-${target})

  set(VERBOSE ${${target}_VERBOSE})

  if(VERBOSE)
    MESSAGE(STATUS "*** Arguments for ${target}")
    MESSAGE(STATUS "FILES: ${${target}_FILES}")
    MESSAGE(STATUS "DESTINATION: ${${target}_DESTINATION}")
    MESSAGE(STATUS "Option VERBOSE: ${${target}_VERBOSE}")
	MESSAGE(STATUS "Option ROOT: ${${target}_ROOT}")
    MESSAGE(STATUS "Option FORCE_UPDATE ${${target}_FORCE_UPDATE}")
  endif(VERBOSE)

  set(files ${${target}_FILES})
  set(destination ${${target}_DESTINATION})
  set(root ${${target}_ROOT})
  set(force ${${target}_FORCE_UPDATE})

  set(cmakefile ${CMAKE_BINARY_DIR}/app-${target}.cmake)

  if (root)
	file(TO_CMAKE_PATH "${ICUB_APPLICATIONS_PREFIX}" dapp)
  else(root)
	file(TO_CMAKE_PATH "${ICUB_APPLICATIONS_PREFIX}/app/${target}" dapp)
  endif(root)

  if (destination)
		set(dapp ${dapp}/${destination})
  endif (destination)

  unset(BUFFER)

  set(BUFFER "${BUFFER}execute_process(COMMAND \"\${CMAKE_COMMAND}\" -E make_directory \"${dapp}\")\n")
  set(BUFFER "${BUFFER}\nset(files \"${files}\")\n")
  set(BUFFER "${BUFFER}foreach(f \${files})\n")
  set(BUFFER "${BUFFER}\tget_filename_component(fname \"\${f}\" NAME)\n")
  set(BUFFER "${BUFFER}\tif (NOT IS_DIRECTORY \"\${f}\")\n")
  
  if (force)
    set(BUFFER "${BUFFER}\t\t\tmessage(\"Forcing update of: ${dapp}/\${fname}\")\n")
    set(BUFFER "${BUFFER}\t\t\texecute_process(COMMAND \"\${CMAKE_COMMAND}\" -E copy \"\${f}\" \"${dapp}\")\n")
  else(force)

    set(BUFFER "${BUFFER}\t\tif (NOT EXISTS \"${dapp}/\${fname}\")\n")
    set(BUFFER "${BUFFER}\t\t\texecute_process(COMMAND \"\${CMAKE_COMMAND}\" -E copy \"\${f}\" \"${dapp}\")\n")
    set(BUFFER "${BUFFER}\t\telse()\n")
    set(BUFFER "${BUFFER}\t\t\texecute_process(COMMAND \"\${CMAKE_COMMAND}\" -E compare_files \"\${f}\" \"${dapp}/\${fname}\" RESULT_VARIABLE test_not_successful OUTPUT_QUIET ERROR_QUIET)\n")
    set(BUFFER "${BUFFER}\t\t\tif ( test_not_successful )\n")
    set(BUFFER "${BUFFER}\t\t\t\tmessage(\"Preserving locally modified file: ${dapp}/\${fname}\")\n")
    set(BUFFER "${BUFFER}\t\t\t\tmessage(\"--> saving new file to: ${dapp}/\${fname}.new\")\n")
    set(BUFFER "${BUFFER}\t\t\t\texecute_process(COMMAND \"\${CMAKE_COMMAND}\" -E copy \"\${f}\" \"${dapp}/\${fname}.new\")\n")
    set(BUFFER "${BUFFER}\t\t\tendif()\n")
    set(BUFFER "${BUFFER}\t\tendif()\n")

  endif(force)

  set(BUFFER "${BUFFER}\tendif()\n")
  set(BUFFER "${BUFFER}endforeach(f \"\${files}\")\n\n")
  
  file(APPEND ${cmakefile} "${BUFFER}")
endfunction(icub_app_install)

##
# This function is now obsolete.
# Leaving it empty for backward compatibility.
macro(icub_add_target target)
## obsolete...
endmacro(icub_add_target)

## gather all applications from global list ICUB_APPLICATIONS
# declare a target called install_applications which depends
# on all the previously defined applications.
macro(icub_app_all)
    ## add custom targets
    icub_get_property(applications GLOBAL PROPERTY ICUB_APPLICATIONS)
    message(STATUS "Exporting applications: ${applications}")
    add_custom_target(install_applications)
    add_dependencies(install_applications ${applications})
endmacro(icub_app_all)

### From yarp.
# Helper macro to work around a bug in set_property in cmake 2.6.0
# We use icub_ prefix to avoid name clashes with yarp.
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
