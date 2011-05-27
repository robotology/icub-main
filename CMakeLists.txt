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

#### Find YARP

SET(ICUB_REQYARP_VERSION_MAJOR "2")
SET(ICUB_REQYARP_VERSION_MINOR "3")
SET(ICUB_REQYARP_VERSION_PATCH "4")

## avoid checking specific versions of YARP with cmake 2.6.0
## find_package does not work as expected in version 2.6.0

if (CMAKE_MAJOR_VERSION EQUAL 2 AND CMAKE_MINOR_VERSION EQUAL 6 AND CMAKE_PATCH_VERSION LESS 4)
        # find yarp, accept all versions
        find_package(YARP REQUIRED)       
        if (YARP_VERSION_MAJOR LESS ICUB_REQYARP_VERSION_MAJOR OR YARP_VERSION_MINOR LESS ICUB_REQYARP_VERSION OR YARP_VERSION_PATCH LESS ICUB_REQYARP_VERSION_PATCH)
           message(FATAL_ERROR "YARP version is not compatible, please update YARP")
        endif()
else()
        ## find yarp request particular version
        find_package(YARP ${ICUB_REQYARP_VERSION_MAJOR}.${ICUB_REQYARP_VERSION_MINOR}.${ICUB_REQYARP_VERSION_PATH} REQUIRED)
endif()

message(STATUS "YARP is version: ${YARP_VERSION_MAJOR}.${YARP_VERSION_MINOR}.${YARP_VERSION_PATCH}")
 
if (YARP_FOUND)
   if (YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
     message(STATUS "found YARP")
   else(YARP_HAS_MATH_LIB)
     set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
     message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
   endif(YARP_HAS_MATH_LIB) 
else (YARP_FOUND)
  set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
  message(FATAL_ERROR "Cannot find YARP")
endif (YARP_FOUND)

########## add yarp definitions
add_definitions(${YARP_DEFINES})   #this contains also -D_REENTRANT

### pick up yarp's cmake scripts
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

# find matlab libraries
set(BUILD_MATLAB_BASED_TOOLS FALSE CACHE BOOL "Dou you want to build MATLAB-based tools?")
if(BUILD_MATLAB_BASED_TOOLS)
    find_path(MATLAB_INCLUDE_DIRS engine.h NO_DEFAULT_PATH)
    if(WIN32)
       find_path(MATLAB_LIBRARIES_DIRS libeng.lib NO_DEFAULT_PATH)
    else(WIN32)
       find_path(MATLAB_LIBRARIES_DIRS eng NO_DEFAULT_PATH)
    endif(WIN32)
    
    if(WIN32)
       set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmx.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmex.lib
                            ${MATLAB_LIBRARIES_DIRS}/libmat.lib)
    else(WIN32)
       set(MATLAB_LIBRARIES ${MATLAB_LIBRARIES_DIRS}/libeng.so
                            ${MATLAB_LIBRARIES_DIRS}/libmx.so
                            ${MATLAB_LIBRARIES_DIRS}/libmex.so
                            ${MATLAB_LIBRARIES_DIRS}/libmat.so)
    endif(WIN32)
else(BUILD_MATLAB_BASED_TOOLS)
    set(MATLAB_INCLUDE_DIRS "")
    set(MATLAB_LIBRARIES_DIRS "")
    set(MATLAB_LIBRARIES "")
endif(BUILD_MATLAB_BASED_TOOLS)

#### enable testing
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
##########################################

set_property(GLOBAL PROPERTY ICUB_INCLUDE_DIRS)
set_property(GLOBAL PROPERTY ICUB_TARGETS)

# clean up:
# icub-export-build.cmake, icub-export-build-includes.cmake and icub-export-install-includes.cmake
# this is a required step because we will append data to these files during the build
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build-includes.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-install-includes.cmake "")

set(ICUB_EXPERIMENTAL_APP_INSTALL TRUE CACHE BOOL "Turn on app install experimental feature")
file(TO_CMAKE_PATH "$ENV{ICUB_ROOT}" env_icub_root)
SET(ICUB_APPLICATIONS_PREFIX ${env_icub_root} CACHE TYPE PATH)

########### populate applications
if (ICUB_EXPERIMENTAL_APP_INSTALL)
    add_subdirectory(app)
endif(ICUB_EXPERIMENTAL_APP_INSTALL)

# Create everything needed to build our executable.
add_subdirectory(src)

############ generate custom target that exports all applications
if (ICUB_EXPERIMENTAL_APP_INSTALL)
    icub_app_all()
endif(ICUB_EXPERIMENTAL_APP_INSTALL)

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
				DESTINATION bin COMPONENT Core) 

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
                        FindGtkMM.cmake
                        FindGtkMMUnix.cmake
                        FindGtkMMWin32.cmake
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

# temporary options for debugging/testing new functionalities
set(ICUB_USE_INTERFACEGUI TRUE CACHE BOOL "enable interface gui")
set(ICUB_CANPROTOCOL_STRICT TRUE CACHE BOOL "Enable strict check of canbus protocol and firmware versions" FORCE)

######

