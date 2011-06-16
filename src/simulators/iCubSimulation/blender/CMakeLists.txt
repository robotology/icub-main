
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

cmake_minimum_required(VERSION 2.6)

PROJECT(blysim)

SET(BASE ${CMAKE_CURRENT_SOURCE_DIR}/..)
ADD_DEFINITIONS(-DOMIT_LOGPOLAR)

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${BASE}/wrapper)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

FILE(GLOB_RECURSE wrapper_source ${BASE}/wrapper/*.cpp)
FILE(GLOB_RECURSE wrapper_header ${BASE}/wrapper/*.h)

ADD_LIBRARY(blysim ${wrapper_source} ${wrapper_header}
  FakeWorldManager.cpp
  BlenderSimulationBundle.cpp
  BlenderLogicalJoints.h  BlenderSimulationBundle.h  BlenderSimulation.h
  )
TARGET_LINK_LIBRARIES(blysim ${YARP_LIBRARIES})

ADD_EXECUTABLE(blysim_sanity_check test.cpp)
TARGET_LINK_LIBRARIES(blysim_sanity_check blysim)


