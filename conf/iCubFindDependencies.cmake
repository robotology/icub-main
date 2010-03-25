# Copyright: (C) 2009 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Based on code from yarp.

macro(checkandset_dependency package)
    if (${package}_FOUND)
        set(ICUB_HAS_${package} true)
        message(STATUS "found ${package}")
    endif (${package}_FOUND)
endmacro (checkandset_dependency)

message(STATUS "Detecting required libraries")

find_package(YARP)
find_package(GSL)
find_package(GtkMM)
find_package(Qt3)
find_package(OpenCV)

find_package(OpenGL)
find_package(ODE)
find_package(SDL)

message(STATUS "I have found the following libraries:")

checkandset_dependency(YARP)
checkandset_dependency(GSL)
checkandset_dependency(GtkMM)
checkandset_dependency(Qt3)
checkandset_dependency(OPENCV)
checkandset_dependency(OpenGL)
checkandset_dependency(ODE)
checkandset_dependency(SDL)

if (YARP_HAS_LIBMATH)
    set(ICUB_HAS_YARPMATH true)
    message(STATUS "found yarp math library")
endif (YARP_HAS_LIBMATH)



