# Copyright: (C) 2009 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code from yarp.

macro(checkandset_dependency package)
    if (${package}_FOUND)
        set(ICUB_HAS_${package} TRUE CACHE BOOL "Package ${package} found" FORCE)
        set(ICUB_USE_${package} TRUE CACHE BOOL "Use package ${package}")
        message(STATUS "found ${package}")
    else (${package}_FOUND)
    	  set(ICUB_HAS_${package} FALSE CACHE BOOL "" FORCE)
          set(ICUB_USE_${package} FALSE CACHE BOOL "Use package ${package}")
    endif (${package}_FOUND)
    mark_as_advanced(ICUB_HAS_${package})
    
    if (NOT ${package}_FOUND AND ICUB_USE_${package})
        message("Warning: you requested to use the package ${package}, but it is unavailable (or was not found). This might lead to compile errors, we recommend you turn off the ICUB_USE_${package} flag.") 
    endif (NOT ${package}_FOUND AND ICUB_USE_${package})
endmacro (checkandset_dependency)


message(STATUS "Detecting required libraries")
message(STATUS "CMake modules directory: ${CMAKE_MODULE_PATH}")

find_package(GSL)
find_package(GtkMM)
find_package(Qt3)
find_package(GLUT)
find_package(OpenCV)

find_package(OpenGL)
find_package(ODE)
find_package(SDL)

find_package(GtkPlus)
find_package(Gthread)
find_package(ACE)
find_package(IPOPT)
find_package(IPP)

message(STATUS "OpenCV version is ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")

if (OpenCV_FOUND)
  # check version of openCV
  if (OpenCV_VERSION_MAJOR EQUAL 2)
    message(STATUS "OpenCV is at least version 2")
    set(ICUB_OpenCV_LEGACY false CACHE BOOL "Legacy version of OpenCV detected" FORCE)
    mark_as_advanced(ICUB_OpenCV_LEGACY)
  else()
    set(ICUB_OpenCV_LEGACY true CACHE BOOL "Legacy version of OpenCV detected" FORCE)
    message(STATUS "OpenCV is previous 2.0 (some modules will be skipped)")  
    message(STATUS "Setting ICUB_OpenCV_LEGACY true")
  endif()
endif()

if (GtkMM_FOUND)

    message(STATUS "GtkMM version is ${GtkMM_VERSION_MAJOR}.${GtkMM_VERSION_MINOR}")

    if (NOT GtkMM_VERSION_MAJOR)
        message (STATUS "GtkMM version unknown, assuming 2.8")
        set(GtkMM_VERSION_MAJOR 2)
        set(GtkMM_VERSION_MINOR 8)
    endif()

   if (GtkMM_VERSION_MAJOR GREATER 2 OR GtkMM_VERSION_MAJOR EQUAL 2)
    if (GtkMM_VERSION_MINOR GREATER 8 OR GtkMM_VERSION_MINOR EQUAL 8)

        if (GtkMM_VERSION_MINOR LESS 12)
                set(ICUB_GtkMM_LEGACY true CACHE BOOL "Legacy version of GtkMM detected" FORCE)
        else()
                set(ICUB_GtkMM_LEGACY false CACHE BOOL "Legacy version of GtkMM detected" FORCE)
                mark_as_advanced(ICUB_GtkMM_LEGACY)
        endif()
     endif()
    endif()

    # check version of GtkMM
    if (NOT ICUB_GtkMM_LEGACY)
        message(STATUS "GtkMM is at least 2.12")
   else()
        message(STATUS "GtkMM is previous 2.12 (some modules will be skipped)")  
        message(STATUS "Setting ICUB_GtkMM_LEGACY true")
   endif()

endif()

message(STATUS "I have found the following libraries:")

checkandset_dependency(GSL)
checkandset_dependency(Qt3)
checkandset_dependency(GLUT)
checkandset_dependency(OpenGL)
checkandset_dependency(ODE)
checkandset_dependency(SDL)
checkandset_dependency(GtkMM)
checkandset_dependency(GtkPlus)
checkandset_dependency(Gthread)
checkandset_dependency(ACE)
checkandset_dependency(IPOPT)
checkandset_dependency(IPP)
checkandset_dependency(OpenCV)


if (YARP_HAS_LIBMATH)
    set(ICUB_HAS_YARPMATH true)
    message(STATUS "found yarp math library")
endif (YARP_HAS_LIBMATH)





