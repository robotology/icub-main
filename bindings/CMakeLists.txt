# Copyright: (C) 2011 Department of Robotics, Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6.4)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} ${CMAKE_MODULE_PATH})

# Find YARP for bindings-only builds
find_package(YARP COMPONENTS conf OS sig dev math gsl REQUIRED)
foreach(_component conf OS sig dev math gsl)
  get_property(YARP_${_component}_INCLUDE_DIRS TARGET YARP::YARP_${_component} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  include_directories(${YARP_${_component}_INCLUDE_DIRS})
endforeach()

find_package(ICUB REQUIRED)
include_directories(${ICUB_INCLUDE_DIRS})
link_directories(${ICUB_LIBRARY_DIRS})
link_libraries(${ICUB_LIBRARIES})

# Work-around for missing paths to OpenCV libraries
find_package(OpenCV)
if(OpenCV_FOUND)
    link_directories(${OpenCV_LINK_DIRECTORIES} ${OPENCV_LINK_DIRECTORIES})
endif()

include_directories(${CMAKE_SOURCE_DIR}/src/libraries/iDyn/include)
include_directories(${CMAKE_SOURCE_DIR}/src/libraries/iKin/include)
include_directories(${CMAKE_SOURCE_DIR}/src/libraries/ctrlLib/include)
include_directories(${CMAKE_SOURCE_DIR}/src/libraries/skinDynLib/include)

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
    swig_add_library(icub_python
                     LANGUAGE python
                     SOURCES icub.i)
    target_include_directories(${SWIG_MODULE_icub_python_REAL_NAME} SYSTEM PRIVATE ${PYTHON_INCLUDE_PATH})
    set_target_properties(${SWIG_MODULE_icub_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_icub"
                                                                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python")

endif(CREATE_PYTHON)

if(CREATE_RUBY)
    find_package(Ruby REQUIRED)
    set(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
    include_directories(${RUBY_INCLUDE_PATH})
    swig_add_library(icub_ruby
                     LANGUAGE ruby
                     SOURCES icub.i)
    swig_link_libraries(icub_ruby ${RUBY_LIBRARY})
    target_include_directories(${SWIG_MODULE_icub_ruby_REAL_NAME} SYSTEM PRIVATE ${RUBY_INCLUDE_PATH})
    set_target_properties(${SWIG_MODULE_icub_ruby_REAL_NAME} PROPERTIES OUTPUT_NAME "icub"
                                                                      LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/ruby")
endif(CREATE_RUBY)

if(CREATE_JAVA)
    set(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_BINARY_DIR}/yarp")
    set(CMAKE_SWIG_FLAGS "-package;yarp;-Wall;-module;icub")
    find_package(Java REQUIRED)
    find_package(JNI REQUIRED)
    include_directories(SYSTEM ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})

    swig_add_library(icub_java
                     LANGUAGE java
                     SOURCES icub.i)

    swig_link_libraries(icub_java ${SWIG_YARP_LIBRARIES})
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
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  swig_add_library(icub_csharp
                     LANGUAGE csharp
                     SOURCES icub.i)

  swig_link_libraries(icub_csharp ${SWIG_YARP_LIBRARIES})
  set_target_properties(${SWIG_MODULE_icub_csharp_REAL_NAME} PROPERTIES OUTPUT_NAME "icub" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/csharp")
endif(CREATE_CSHARP)

IF (CREATE_LUA)
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  find_package(Lua REQUIRED)
  include_directories(SYSTEM ${LUA_INCLUDE_DIR})
  swig_add_library(icub_lua
                   LANGUAGE lua
                   SOURCES icub.i)
  swig_link_libraries(icub_lua ${LUA_LIBRARY})
  set_target_properties(${SWIG_MODULE_icub_lua_REAL_NAME} PROPERTIES OUTPUT_NAME "icub" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/lua")
ENDIF (CREATE_LUA)
