# Copyright: (C) 2011 Department of Robotics, Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6.4)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

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
if(OpenCV_FOUND)
    link_directories(${OpenCV_LINK_DIRECTORIES} ${OPENCV_LINK_DIRECTORIES})
endif()

include_directories(${CMAKE_SOURCE_DIR}/..)
include_directories(${CMAKE_SOURCE_DIR}/../src/libraries/icubmod)

# for yarp.i
include_directories(${YARP_BINDINGS})

option(CREATE_PYTHON "Create Python interface" FALSE)
option(CREATE_RUBY   "Create Ruby interface"   FALSE)
option(CREATE_JAVA   "Create Java interface"   FALSE)
option(CREATE_CSHARP "Create CSharp interface" FALSE)
option(CREATE_LUA    "Create LUA interface"    FALSE)

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
message(STATUS "${SWIG_USE_FILE}")
set (MIN_SWIG_VERSION "1.3.29")
if("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")
    message( FATAL_ERROR "Please update your SWIG. Current version: ${SWIG_VERSION}. The minimum version needed: ${MIN_SWIG_VERSION}")
endif("${SWIG_VERSION}" VERSION_LESS "${MIN_SWIG_VERSION}")

set_source_files_properties(icub.i PROPERTIES CPLUSPLUS ON)

if(CREATE_PYTHON)
    set(CMAKE_SWIG_FLAGS "-Wall;-module;icub;-threads")

    set(ICUB_USE_PYTHON_VERSION "" CACHE STRING "Specify python version to use" )

    find_package(PythonInterp ${ICUB_USE_PYTHON_VERSION} REQUIRED)
    set(ICUB_USE_PYTHON_VERSION_ARGS)
    if(NOT ICUB_USE_PYTHON_VERSION)
      set (ICUB_USE_PYTHON_VERSION ${PYTHON_VERSION_STRING})
    endif()
    find_package(PythonLibs ${ICUB_USE_PYTHON_VERSION} EXACT)

    get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
    link_directories(${PYTHON_DIR})

    if(WIN32)
        get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
        link_directories(${PYTHON_DIR})
    endif()

    # message("-->${PYTHON_LIBRARY} ${PYTHON_DIR}")

    include_directories(${PYTHON_INCLUDE_PATH})
    swig_add_module(icub python icub.i)
endif(CREATE_PYTHON)

if(CREATE_RUBY)
    find_package(Ruby REQUIRED)
    set(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
    include_directories(${RUBY_INCLUDE_PATH})
    swig_add_module(icub ruby icub.i)
    swig_link_libraries(icub ${RUBY_LIBRARY})
    set_target_properties(icub PROPERTIES PREFIX "")
endif(CREATE_RUBY)

if(CREATE_JAVA)
    set(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/yarp")
    set(CMAKE_SWIG_FLAGS "-package;yarp;-Wall;-module;icub")
    find_package(JNI REQUIRED)
    include_directories(${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})
    swig_add_module(icub java icub.i)
    #option (CREATE_JAVA_CLASS_FILES "Compile java code with javac" ON)
    #if(CREATE_JAVA_CLASS_FILES)
    #  ADD_CUSTOM_COMMAND(TARGET icub
    #    POST_BUILD
    #    COMMAND javac -source 1.3 -target 1.3 "icub/*.java"
    #    WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    #endif()
    if(APPLE)
        set_target_properties(icub PROPERTIES SUFFIX ".jnilib")
    endif(APPLE)
endif(CREATE_JAVA)

if(CREATE_CSHARP)
    set(target_name icub)
    set(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
    SWIG_ADD_MODULE(icub csharp icub.i)
endif(CREATE_CSHARP)

IF (CREATE_LUA)
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  find_package(Lua51 REQUIRED) 
  INCLUDE_DIRECTORIES(${LUA_INCLUDE_DIR})
  SWIG_ADD_MODULE(icub lua icub.i)
  SWIG_LINK_LIBRARIES(icub ${LUA_LIBRARY})
  IF (UNIX)
    ADD_CUSTOM_COMMAND(TARGET icub
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E create_symlink libicub.so icub.so 
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ENDIF (UNIX)
ENDIF (CREATE_LUA)
