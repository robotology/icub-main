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
endmacro (checkandset_dependency)


message(STATUS "Detecting required libraries")
message(STATUS "CMake modules directory: ${CMAKE_MODULE_PATH}")

find_package(GSL)
find_package(GtkMM)
find_package(Qt3)
find_package(OpenCV)

find_package(OpenGL)
find_package(ODE)
find_package(SDL)

find_package(GtkPlus)
find_package(ACE)
find_package(IPOPT)
find_package(IPP)
message(STATUS "I have found the following libraries:")

checkandset_dependency(GSL)
checkandset_dependency(GtkMM)
checkandset_dependency(Qt3)
checkandset_dependency(OpenCV)
checkandset_dependency(OpenGL)
checkandset_dependency(ODE)
checkandset_dependency(SDL)
checkandset_dependency(GtkPlus)
checkandset_dependency(ACE)
checkandset_dependency(IPOPT)
checkandset_dependency(IPP)

if (YARP_HAS_LIBMATH)
    set(ICUB_HAS_YARPMATH true)
    message(STATUS "found yarp math library")
endif (YARP_HAS_LIBMATH)



