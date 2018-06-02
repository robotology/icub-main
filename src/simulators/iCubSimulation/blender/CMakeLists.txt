
# Copyright (C) 2010 Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Compile reusable parts of the simulator as a library.
# This is particularly targeted at Blender integration.
# Not currently integrated with the ICUB build.
# Sketch of anticipated usage -
# SimulationBundle *bundle = new BlenderSimulationBundle;
# SimulationRun main;
# main.run(bundle,0,NULL);
# - need to replace that with start and stop.

cmake_minimum_required(VERSION 2.6.4)

project(blysim)

set(BASE ${CMAKE_CURRENT_SOURCE_DIR}/..)
add_definitions(-DOMIT_LOGPOLAR)

find_package(YARP REQUIRED)

include_directories(${BASE}/wrapper)
                    ${CMAKE_CURRENT_SOURCE_DIR})

file(GLOB_RECURSE wrapper_source ${BASE}/wrapper/*.cpp)
file(GLOB_RECURSE wrapper_header ${BASE}/wrapper/*.h)

add_library(blysim ${wrapper_source}
                   ${wrapper_header}
                   FakeWorldManager.cpp
                   BlenderSimulationBundle.cpp
                   BlenderLogicalJoints.h
                   BlenderSimulationBundle.h
                   BlenderSimulation.h
  )
target_link_libraries(blysim ${YARP_LIBRARIES})

add_executable(blysim_sanity_check test.cpp)
target_link_libraries(blysim_sanity_check blysim)
