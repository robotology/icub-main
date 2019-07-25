# Copyright: (C) 2009 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code from yarp.

macro(checkandset_dependency package)
    if (${package}_FOUND)
        set(ICUB_HAS_${package} TRUE CACHE BOOL "Package ${package} found" FORCE)
        set(ICUB_USE_${package} TRUE CACHE BOOL "Use package ${package}")
        message(STATUS "${package}: found")
    else (${package}_FOUND)
        set(ICUB_HAS_${package} FALSE CACHE BOOL "" FORCE)
        set(ICUB_USE_${package} FALSE CACHE BOOL "Use package ${package}")
        message(STATUS "${package}: NOT found")
    endif (${package}_FOUND)
    mark_as_advanced(ICUB_HAS_${package})

    if (NOT ${package}_FOUND AND ICUB_USE_${package})
        message(WARNING "You requested to use the package ${package}, but it is unavailable (or was not found). This might lead to compile errors, we recommend you turn off the ICUB_USE_${package} flag.")
    endif (NOT ${package}_FOUND AND ICUB_USE_${package})

    #store all dependency flags for later export
    set_property(GLOBAL APPEND PROPERTY ICUB_DEPENDENCIES_FLAGS ICUB_USE_${package})

endmacro (checkandset_dependency)


message(STATUS "Detecting required libraries")
message(STATUS "CMake modules directory: ${CMAKE_MODULE_PATH}")

# This is a workaround to enable the warning only if
# icub_firmware_shared was found, but the version is not compatible with
# the requested one.
find_package(icub_firmware_shared COMPONENTS canProtocolLib QUIET)
if(icub_firmware_shared_FOUND)
  find_package(icub_firmware_shared 4.0.6 COMPONENTS canProtocolLib)
endif()

find_package(GSL)
find_package(GLUT)
find_package(OpenCV)

find_package(OpenGL)
find_package(ODE)
find_package(SDL)

set(GTK2_USE_IMPORTED_TARGETS ON)
find_package(GTK2 COMPONENTS gtk gtkmm)
if(WIN32 AND GTK2_FOUND)
    list(REMOVE_ITEM GTK2_LIBRARIES ${FREETYPE_LIBRARY} ${GTK2_PANGOXFT_LIBRARY})
endif()
find_package(ACE)
find_package(IPOPT)
find_package(IPP)
find_package(Qt5 COMPONENTS Core Widgets OpenGL Quick Qml Concurrent PrintSupport QUIET)

message(STATUS "OpenCV version is ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")

if (OpenCV_FOUND)
  # check version of openCV
  if (OpenCV_VERSION_MAJOR GREATER 1)
    message(STATUS "OpenCV is at least version 2")
    set(ICUB_OpenCV_LEGACY false CACHE BOOL "Legacy version of OpenCV detected" FORCE)
    mark_as_advanced(ICUB_OpenCV_LEGACY)
  else()
    set(ICUB_OpenCV_LEGACY true CACHE BOOL "Legacy version of OpenCV detected" FORCE)
    message(STATUS "OpenCV is previous 2.0 (some modules will be skipped)")  
    message(STATUS "Setting ICUB_OpenCV_LEGACY true")
  endif()

  set(ICUB_LINK_DIRECTORIES ${ICUB_LINK_DIRECTORIES} ${OpenCV_LIB_DIR})
endif()

if (GTK2_FOUND)

    message(STATUS "GTK2 version is ${GTK2_MAJOR_VERSION}.${GTK2_MINOR_VERSION}")

    if (NOT GTK2_MAJOR_VERSION)
        message (STATUS "GTK2 version unknown, assuming 2.8")
        set(GTK2_MAJOR_VERSION 2)
        set(GTK2_MINOR_VERSION 8)
    endif()

   if (GTK2_MAJOR_VERSION GREATER 2 OR GTK2_MAJOR_VERSION EQUAL 2)
    if (GTK2_MINOR_VERSION GREATER 8 OR GTK2_MINOR_VERSION EQUAL 8)

        if (GTK2_MINOR_VERSION LESS 12)
                set(ICUB_GTK2_LEGACY true CACHE BOOL "Legacy version of GTK2 detected" FORCE)
        else()
                set(ICUB_GTK2_LEGACY false CACHE BOOL "Legacy version of GTK2 detected" FORCE)
                mark_as_advanced(ICUB_GTK2_LEGACY)
        endif()
     endif()
    endif()

    # check version of GtkMM
    if (NOT ICUB_GTK2_LEGACY)
        message(STATUS "GTK2 is at least 2.12")
   else()
        message(STATUS "GTK2 is prior to 2.12 (some modules will be skipped)")  
        message(STATUS "Setting ICUB_GTK2_LEGACY true")
   endif()

endif()

message(STATUS "I have found the following libraries:")

checkandset_dependency(icub_firmware_shared)
checkandset_dependency(GSL)
checkandset_dependency(GLUT)
checkandset_dependency(OpenGL)
checkandset_dependency(ODE)
checkandset_dependency(SDL)
checkandset_dependency(GTK2)
checkandset_dependency(ACE)
checkandset_dependency(IPOPT)
checkandset_dependency(IPP)
checkandset_dependency(OpenCV)
checkandset_dependency(Qt5)

if (YARP_HAS_LIBMATH)
    set(ICUB_HAS_YARPMATH true)
    message(STATUS "found yarp math library")
endif (YARP_HAS_LIBMATH)
