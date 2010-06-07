# Copyright: (C) 2009 RobotCub Consortium
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

if(MSVC)
    MESSAGE(STATUS "Running on windows")
    
    # These used to be convenient in the old build, now caused a lot of troubles. Are they still needed?
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib;libcmtd.lib;msvcrt.lib")
    
    # ACE uses a bunch of functions MSVC warns about.
    # The warnings make sense in general, but not in this case.
    # this gets rids of deprecated unsafe crt functions
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    # this gets rid of warning about deprecated POSIX names
    add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
    # Traditionally, we add "d" postfix to debug libraries
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING 
        "Choose the type of build, recommanded options are: Debug or Release")
endif(NOT CMAKE_BUILD_TYPE)
# Hide variable to MSVC users, since it is not needed
 if (MSVC)
        mark_as_advanced(CMAKE_BUILD_TYPE)
endif(MSVC)

#### set EXPERIMENTAL  VARIABLE
set(ICUB_EXPERIMANTAL FALSE CACHE BOOL "experimental")

#### Find YARP
find_package(YARP)
if (YARP_FOUND)
  set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
  message(STATUS "found YARP")
else (YARP_FOUND)
  set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
  message(FATAL "Cannot find YARP")
endif (YARP_FOUND)

### pick up yarp's cmake scripts
set(CMAKE_MODULE_PATH ${YARP_DIR}/conf ${CMAKE_MODULE_PATH})

# Pick up our scripts - they are all in the conf subdirectory
set(ICUB_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

# Set up some large-scale compile options - do you want devices? guis?
# INCLUDE(${PROJECT_SOURCE_DIR}/ICUBOptions.cmake)

# add functionalities like icub_add_library/icub_add_executable
include(${PROJECT_SOURCE_DIR}/conf/iCubHelpers.cmake)

# find dependencies and set variables
include(${PROJECT_SOURCE_DIR}/conf/iCubFindDependencies.cmake)

# check for libraries that are strictly required
if (NOT ICUB_HAS_GSL)
  message(FATAL "Cannot find GSL")
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

# clean up icub-export-build.cmake and icub-export-build-includes.cmake(required because targets append to it)
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build-includes.cmake "")

# Create everything needed to build our executable.
add_subdirectory(src)

# export the installed project 
include(${PROJECT_SOURCE_DIR}/conf/iCubExportForInstall.cmake)

# export the build tree
include(${PROJECT_SOURCE_DIR}/conf/iCubExportBuildTree.cmake)

## install cmake scripts
set(ICUB_CMAKE_SCRIPTS FindPLXCANAPI.cmake FindCFW2CANAPI.cmake FindESDCANAPI.cmake FindGtkMM.cmake FindGtkMMUnix.cmake FindGtkMMWin32.cmake FindIPOPT.cmake FindODE.cmake)
foreach(conf ${ICUB_CMAKE_SCRIPTS})
  configure_file(${CMAKE_SOURCE_DIR}/conf/${conf}
                 ${CMAKE_BINARY_DIR}/conf/${conf} COPYONLY IMMEDIATE)
  install(FILES ${CMAKE_BINARY_DIR}/conf/${conf} COMPONENT scripts 
	  DESTINATION conf)
endforeach(conf)






