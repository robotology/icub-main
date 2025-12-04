# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12...3.28)

set(ROOT_PROJECT_NAME ICUB)
project(${ROOT_PROJECT_NAME} LANGUAGES C CXX
                             VERSION 2.11.0)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

option(ICUB_COMPILE_BINDINGS "Compile optional language bindings"  FALSE)

set(YCM_REQUIRED_VERSION 0.18.2)
set(YARP_REQUIRED_VERSION 3.12.1)

find_package(YCM ${YCM_REQUIRED_VERSION} REQUIRED)
find_package(YARP ${YARP_REQUIRED_VERSION} REQUIRED)

enable_testing()

if(YARP_math_FOUND)
  set(ICUB_HAS_YARP TRUE CACHE BOOL "" FORCE)
  message(STATUS "found libYARP_math")
else()
  set(ICUB_HAS_YARP FALSE CACHE BOOL "" FORCE)
  message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
endif()

set_property(GLOBAL PROPERTY ICUB_INCLUDE_DIRS)
set_property(GLOBAL PROPERTY ICUB_TARGETS)
set_property(GLOBAL PROPERTY ICUB_DEPENDENCIES_FLAGS) # this is populated iCubFindDependencies.cmake

############# Options and definitions
# Important: all these have to come before all calls to add_subdirectory()
# otherwise code inside these directory will not be aware of the options.

# Pick up our scripts - they are all in the conf subdirectory
set(ICUB_MODULE_PATH ${PROJECT_SOURCE_DIR}/conf)
set(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

# add functionalities like icub_add_library/icub_add_executable
include(${PROJECT_SOURCE_DIR}/conf/iCubHelpers.cmake)

# Import common options (disable flags in windows, rpath, etc..)
# These options are shared with projects in contrib.
include(${PROJECT_SOURCE_DIR}/conf/iCubOptions.cmake)

# find dependencies and set variables
include(${PROJECT_SOURCE_DIR}/conf/iCubFindDependencies.cmake)

### enable testing
option(ICUB_DASHBOARD_SUBMIT "Submit compile tests to cdash" FALSE)

if(ICUB_DASHBOARD_SUBMIT)
    include (CTest)
endif()

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Build position independent code.
# Position Independent Code (PIC) is commonly used for shared libraries so that
# the same shared library code can be loaded in each program address space in a
# location where it will not overlap with any other uses of such memory.
# In particular, this option avoids problems occurring when a process wants to
# load more than one shared library at the same virtual address.
# Since shared libraries cannot predict where other shared libraries could be
# loaded, this is an unavoidable problem with the traditional shared library
# concept.
# Generating position-independent code is often the default behavior for most
# modern compilers.
# Moreover linking a static library that is not built with PIC from a shared
# library will fail on some compiler/architecture combinations.
# Further details on PIC can be found here:
# https://eli.thegreenplace.net/2011/11/03/position-independent-code-pic-in-shared-libraries/
option(CMAKE_POSITION_INDEPENDENT_CODE "When compiling static libraries generate position independent code" TRUE)
# Enable RPATH support for installed binaries and libraries
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_FULL_BINDIR}"
                          LIB_DIRS "${CMAKE_INSTALL_FULL_LIBDIR}"
                          INSTALL_NAME_DIR "${CMAKE_INSTALL_FULL_LIBDIR}"
                          USE_LINK_PATH)

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
# Some options: mostly cache variables
file(TO_CMAKE_PATH "$ENV{ICUB_ROOT}" env_icub_root)
set(ICUB_APPLICATIONS_PREFIX ${env_icub_root} CACHE PATH "Prefix directory for installing applications. Read only, set ICUB_ROOT environment variable to affect this setting." FORCE)
mark_as_advanced(ICUB_APPLICATIONS_PREFIX)

# Force dynamic plugins. Must be set before
# yarp_configure_external_installation to ensure that iCub.ini is
# generated correctly
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "iCub is always built with dynamic plugins")
yarp_configure_external_installation(iCub WITH_PLUGINS)
# Hide YARP_FORCE_DYNAMIC_PLUGINS from gui
# (yarp_configure_external_installation changes TYPE to BOOL)
set_property(CACHE YARP_FORCE_DYNAMIC_PLUGINS PROPERTY TYPE INTERNAL)

# temporary options for debugging/testing new functionalities
set(ICUB_CANPROTOCOL_STRICT TRUE CACHE BOOL "Enable strict check of canbus protocol and firmware versions" FORCE)
set(ICUB_CANMASKS_STRICT_FILTER TRUE CACHE BOOL "Enable strict filtering on can address. Needed with sharedcan.")
set(ICUB_ASK_REFERENCE_TO_FIRMWARE TRUE CACHE BOOL "Pos/Vel/Trq references will be asked to the firmware instead of being cached by motion control")

###################### End options and definitions

########### populate applications
add_subdirectory(app)

# Create everything needed to build our executables.
add_subdirectory(src)

## install cmake scripts
message(STATUS "Installing cmake scripts")
set(ICUB_CMAKE_SCRIPTS iCubHelpers.cmake iCubOptions.cmake)

foreach(conf ${ICUB_CMAKE_SCRIPTS})
  # in YARP we used to copy Find* scripts to the build directory
  # this has changed in YARP but NOT in iCub.
  configure_file(${CMAKE_SOURCE_DIR}/conf/${conf}
                 ${CMAKE_BINARY_DIR}/conf/${conf} COPYONLY IMMEDIATE)

  # on the other hand we do need to install Find* script at install time
  install(FILES  conf/${conf} DESTINATION share/iCub/cmake/ COMPONENT Development)
endforeach(conf)

# Create the dummy target ICUB and connect all the COMPONENTS
include(${PROJECT_SOURCE_DIR}/conf/iCubDescribe.cmake)

if(ICUB_COMPILE_BINDINGS)
  add_subdirectory(bindings)
endif()

include(AddUninstallTarget)
