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

IF(win32)
#disable annoying warnings about use of deprecated c functions
ADD_DEFINITIONS(-D_CRT_SECURE_NO_DEPRECATE)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:libcmt.lib;libcmtd.lib")
ENDIF(win32)

PROJECT(iCub)

# Pick up our scripts - they are all in the conf subdirectory
SET(ICUB_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)
SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

# Set up some large-scale compile options - do you want devices? guis?
# INCLUDE(${PROJECT_SOURCE_DIR}/ICUBOptions.cmake)

#add functionalities like icub_add_library/icub_add_executable
include(${PROJECT_SOURCE_DIR}/conf/iCubHelpers.cmake)

# find dependencies and set variables
include(${PROJECT_SOURCE_DIR}/dependencies.cmake)


### this makes everything go in $ICUB_DIR/lib and $ICUB_DIR/bin
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
MESSAGE(STATUS "Libraries go to ${LIBRARY_OUTPUT_PATH}")
MESSAGE(STATUS "Executables go to ${EXECUTABLE_OUTPUT_PATH}")
# this doesn't happen automatically for makefiles
MAKE_DIRECTORY(${LIBRARY_OUTPUT_PATH})
MAKE_DIRECTORY(${EXECUTABLE_OUTPUT_PATH})
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(CMAKE_BACKWARDS_COMPATIBILITY)
##########################################

set(ICUB_INCLUDE_DIRS "")
set(ICUB_LIBRARIES "")
set(ICUB_TARGETS "" CACHE STRING "list of all targets" FORCE) #cleanup directory

# clean up icub-export-build.cmake and icub-export-build-includes.cmake(required because targets append to it)
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build.cmake "")
file(WRITE ${CMAKE_BINARY_DIR}/icub-export-build-includes.cmake "")

# Create everything needed to build our executable.
add_subdirectory(src)

# export the installed project 
include(${PROJECT_SOURCE_DIR}/conf/iCubExportForInstall.cmake)

# export the build tree
include(${PROJECT_SOURCE_DIR}/conf/iCubExportBuildTree.cmake)




