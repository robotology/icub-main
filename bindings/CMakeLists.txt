# Copyright: (C) 2011 IITRBCS
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
link_directories(${YARP_LIBRARY_DIRS})
link_libraries(${YARP_LIBRARIES})

find_package(ICUB REQUIRED)
include_directories(${ICUB_INCLUDE_DIRS})
link_directories(${ICUB_LIBRARY_DIRS})
link_libraries(${ICUB_LIBRARIES})

# Work-around for missing paths to OpenCV libraries
find_package(OpenCV)
if (OpenCV_FOUND)
  link_directories(${OpenCV_LINK_DIRECTORIES} ${OPENCV_LINK_DIRECTORIES})
endif ()

include_directories(${CMAKE_SOURCE_DIR}/..)
include_directories(${CMAKE_SOURCE_DIR}/../src/libraries/icubmod)

# for yarp.i
include_directories(${YARP_MODULE_PATH}/../example/swig)

set(CREATE_PYTHON FALSE CACHE BOOL "Create Python interface")
set(CREATE_RUBY FALSE CACHE BOOL "Create Ruby interface")
set(CREATE_JAVA FALSE CACHE BOOL "Create Java interface")
SET(CREATE_CSHARP FALSE CACHE BOOL "Create CSharp interface")

FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})
MESSAGE(STATUS "${SWIG_USE_FILE}")
SET (MIN_SWIG_VERSION "1.3.29")
IF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")
  MESSAGE( FATAL_ERROR "Please update your SWIG. Current version: ${SWIG_VERSION}. The minimum version needed: ${MIN_SWIG_VERSION}")
ENDIF ("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")

SET_SOURCE_FILES_PROPERTIES(icub.i PROPERTIES CPLUSPLUS ON)

if (CREATE_PYTHON)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;icub;-threads")
  find_package(PythonLibs)
  include_directories(${PYTHON_INCLUDE_PATH})
  swig_add_module(icub python icub.i)
endif (CREATE_PYTHON)

if (CREATE_RUBY)
  find_package(Ruby REQUIRED)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  include_directories(${RUBY_INCLUDE_PATH})
  swig_add_module(icub ruby icub.i)
  swig_link_libraries(icub ${RUBY_LIBRARY})
  set_target_properties(icub PROPERTIES PREFIX "")
endif (CREATE_RUBY)

if (CREATE_JAVA)
  set(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/yarp")
  set(CMAKE_SWIG_FLAGS "-package;yarp;-Wall;-module;icub")
  find_package(JNI REQUIRED)
  include_directories(${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})
  swig_add_module(icub java icub.i)
  #option (CREATE_JAVA_CLASS_FILES "Compile java code with javac" ON)
  #if (CREATE_JAVA_CLASS_FILES)
  #  ADD_CUSTOM_COMMAND(TARGET icub
  #    POST_BUILD
  #    COMMAND javac -source 1.3 -target 1.3 "icub/*.java"
  #    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  #endif ()
  if (APPLE)
    set_target_properties(icub PROPERTIES SUFFIX ".jnilib")
  endif (APPLE)
endif (CREATE_JAVA)

IF (CREATE_CSHARP)
   SET(target_name icub)
   SET(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
   SWIG_ADD_MODULE(icub csharp icub.i)
ENDIF (CREATE_CSHARP)
