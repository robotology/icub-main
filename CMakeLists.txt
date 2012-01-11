# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

# New build system. Exploit new cmake 2.6 features (export). 

#reduce warning level with cmake 2.6
cmake_minimum_required(VERSION 2.6)
#cmake policies
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(iCub)

#### Find YARP, require that YARP version is at least ICUB_REQYARP_VERSION
SET(ICUB_REQYARP_VERSION_MAJOR "2")
SET(ICUB_REQYARP_VERSION_MINOR "3")
SET(ICUB_REQYARP_VERSION_PATCH "13")
SET(ICUB_REQYARP_VERSION ${ICUB_REQYARP_VERSION_MAJOR}.${ICUB_REQYARP_VERSION_MINOR}.${ICUB_REQYARP_VERSION_PATCH})

find_package(YARP REQUIRED)

message(STATUS "YARP is version: ${YARP_VERSION_MAJOR}.${YARP_VERSION_MINOR}.${YARP_VERSION_PATCH}")
 
# unfortunately VERSION_LESS is unavailable with cmake 2.6.? 
if (CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION EQUAL 6 AND CMAKE_PATCH_VERSION LESS 4)
        # # find yarp, accept all versions, then manually check YARP version
       
        if (YARP_VERSION_MAJOR LESS ICUB_REQYARP_VERSION_MAJOR OR YARP_VERSION_MINOR LESS ICUB_REQYARP_VERSION)
           message(FATAL_ERROR "YARP version is not compatible, please update YARP")
        else()
            if(YARP_VERSION_PATCH LESS ICUB_REQYARP_VERSION_PATCH)
               message(FATAL_ERROR "YARP version is not compatible, please update YARP")
            else()
               message(STATUS "found compatible YARP version")
            endif()
        endif()
else()
    if (YARP_VERSION VERSION_LESS ICUB_REQYARP_VERSION)
        message(FATAL_ERROR "YARP version is not compatible, please update YARP")
    else()
        message(STATUS "found compatible YARP version")
    endif()
endif()

if (YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
     message(STATUS "found libYARP_math")
else(YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
     message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
endif(YARP_HAS_MATH_LIB) 


############# Options and definitions
# Important: all these have to come before all calls to add_subdirectory()
# otherwise code inside these directory will not be aware of the options.
#
# add yarp definitions
add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT

# pick up yarp's cmake scripts
set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${CMAKE_MODULE_PATH})

# Pick up our scripts - they are all in the conf subdirectory
set(ICUB_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

# get the current version of the iCub build
include(${PROJECT_SOURCE_DIR}/conf/iCubVersion.cmake)

# Import common options (disable flags in windows, rpath, etc..)
# These options are shared with projects in contrib.
include(${PROJECT_SOURCE_DIR}/conf/iCubOptions.cmake)

# add functionalities like icub_add_library/icub_add_executable
include(${PROJECT_SOURCE_DIR}/conf/iCubHelpers.cmake)

# find dependencies and set variables
include(${PROJECT_SOURCE_DIR}/conf/iCubFindDependencies.cmake)

#### options that are iCub specific
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${IPOPT_LINK_FLAGS}")

# check for libraries that are strictly required
if (NOT ICUB_HAS_GSL)
  message(FATAL_ERROR "Cannot find GSL")
else (NOT ICUB_HAS_GSL)
  if (NOT ICUB_USE_GSL)
    message(STATUS "GSL is strictly required: GSL enabled")
    set(ICUB_USE_GSL TRUE CACHE BOOL "Use package GSL" FORCE)
  endif (NOT ICUB_USE_GSL)
endif (NOT ICUB_HAS_GSL)

### enable testing
include (CTest)

### this makes everything go in $ICUB_DIR/lib and $ICUB_DIR/bin
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
message(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
message(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
# this doesn't happen automatically for makefiles
make_directory(${LIBRARY_OUTPUT_PATH})
make_directory(${EXECUTABLE_OUTPUT_PATH})
mark_as_advanced(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)
mark_as_advanced(CMAKE_BACKWARDS_COMPATIBILITY)

set_property(GLOBAL PROPERTY ICUB_INCLUDE_DIRS)
set_property(GLOBAL PROPERTY ICUB_TARGETS)

# clean up:
# icub-export-build.cmake, icub-export-build-includes.cmake and icub-export-install-includes.cmake
# this is a required step because we will append data to these files during the build
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build-includes.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-install-includes.cmake "")

# Some options: mostly cache variables
set(ICUB_INSTALL_APPLICATIONS TRUE CACHE BOOL "Turn on/off install for applications")
file(TO_CMAKE_PATH "$ENV{ICUB_ROOT}" env_icub_root)

set(__ICUB_INSTALL_APPLICATIONS__ TRUE)

if (ICUB_INSTALL_APPLICATIONS)
   if (NOT env_icub_root)
       message("No ICUB_ROOT environment variable set, disabling application install rules.")
       set(__ICUB_INSTALL_APPLICATIONS__ FALSE)
   endif()
endif()

set(ICUB_APPLICATIONS_PREFIX ${env_icub_root} CACHE PATH "Prefix directory for installing applications. Read only, set ICUB_ROOT environment variable to affect this setting." FORCE)

# temporary options for debugging/testing new functionalities
set(ICUB_USE_INTERFACEGUI TRUE CACHE BOOL "enable interface gui")
set(ICUB_CANPROTOCOL_STRICT TRUE CACHE BOOL "Enable strict check of canbus protocol and firmware versions" FORCE)

###################### End options and definitions

########### populate applications
if (__ICUB_INSTALL_APPLICATIONS__)
    add_subdirectory(app)
endif(__ICUB_INSTALL_APPLICATIONS__)

# Create everything needed to build our executables.
add_subdirectory(src)

############ generate custom target that exports all applications
if (__ICUB_INSTALL_APPLICATIONS__)
    icub_app_all()
endif(__ICUB_INSTALL_APPLICATIONS__)

########### propagate version info to targets
get_property(ICUB_TARGETS GLOBAL PROPERTY ICUB_TARGETS)
foreach(t ${ICUB_TARGETS})
  set_target_properties(${t} PROPERTIES VERSION ${ICUB_GENERIC_VERSION}
                        SOVERSION ${ICUB_GENERIC_SOVERSION})
endforeach()
######################

# export the build tree
message(STATUS "Now exporting build tree")
include(${PROJECT_SOURCE_DIR}/conf/iCubExportBuildTree.cmake)

################ copying python scripts
message(STATUS "Installing manager.py")
install(FILES  app/default/scripts/manager.py app/default/scripts/icub-cluster-server.sh app/default/scripts/icub-cluster-run.sh 
        DESTINATION bin COMPONENT Core PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# add target to copy manager.py to bin -- notice that in windows this can be bin/debug or bin/release
# so we need to use ${CMAKE_CFG_INTDIR}
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/python-scripts-cmd
      COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/app/default/scripts/manager.py ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/app/default/scripts/icub-cluster-server.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/app/default/scripts/icub-cluster-run.sh ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}
      COMMENT "Copying manager.py, icub-cluster scripts to ${CMAKE_BINARY_DIR}/bin/${CMAKE_CFG_INTDIR}")
add_custom_target(python-scripts ALL DEPENDS ${CMAKE_BINARY_DIR}/python-scripts-cmd)
if (ICUB_EXPERIMENTAL_APP_INSTALL)
    add_dependencies(install_applications python-scripts)
endif(ICUB_EXPERIMENTAL_APP_INSTALL)
###############################
    
# export the installed project 
message(STATUS "Now exporting for install")
include(${PROJECT_SOURCE_DIR}/conf/iCubExportForInstall.cmake)

## install cmake scripts
message(STATUS "Installing cmake scripts")
set(ICUB_CMAKE_SCRIPTS FindPLXCANAPI.cmake
                        FindCFW2CANAPI.cmake
                        FindESDCANAPI.cmake
                        FindIPOPT.cmake
                        FindODE.cmake
                        FindIPP.cmake
                        FindQt3.cmake
                        FindGLUT.cmake
                        FindOpenGL.cmake
                        iCubHelpers.cmake
                        iCubOptions.cmake
                        iCubFindDependencies.cmake)

foreach(conf ${ICUB_CMAKE_SCRIPTS})
  # in YARP we used to copy Find* scripts to the build directory
  # this as changed in YARP but NOT in iCub.
  configure_file(${CMAKE_SOURCE_DIR}/conf/${conf}
                 ${CMAKE_BINARY_DIR}/conf/${conf} COPYONLY IMMEDIATE)

  # on the other hand we do need to install Find* script at install time
  install(FILES  conf/${conf} DESTINATION share/iCub/cmake/ COMPONENT Development)

endforeach(conf)

## add the "uninstall" target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/conf/template/icub-config-uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/icub-config-uninstall.cmake"
  IMMEDIATE @ONLY)

include(${PROJECT_SOURCE_DIR}/conf/iCubPackage.cmake)
  
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/icub-config-uninstall.cmake")

#########

