# Copyright: (C) 2011 Department of Robotics, Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Paul Fitzpatrick
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

# Find YARP for bindings-only builds
find_package(YARP COMPONENTS conf os sig dev math gsl REQUIRED)
foreach(_component conf os sig dev math gsl)
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
include_directories(${CMAKE_SOURCE_DIR}/src/libraries/optimization/include)

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
    find_package(Python REQUIRED COMPONENTS Interpreter Development)
    set(CMAKE_SWIG_FLAGS "-Wall;-module;icub;-threads")

    swig_add_library(icub_python
                     LANGUAGE python
                     SOURCES icub.i)
    target_link_libraries(${SWIG_MODULE_icub_python_REAL_NAME} Python::Python)
    set_target_properties(${SWIG_MODULE_icub_python_REAL_NAME} PROPERTIES OUTPUT_NAME "_icub")

    # installation path is determined reliably on most platforms using distutils
    execute_process(COMMAND ${Python_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                    OUTPUT_VARIABLE PYTHON_INSTDIR
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/icub.py DESTINATION ${PYTHON_INSTDIR})

    set(_CMAKE_INSTALL_PYTHONDIR "${PYTHON_INSTDIR}")
    set(CMAKE_INSTALL_PYTHONDIR ${_CMAKE_INSTALL_PYTHONDIR} CACHE PATH "python bindings (${_CMAKE_INSTALL_PYTHONDIR})")
    mark_as_advanced(CMAKE_INSTALL_PYTHONDIR)
    if(NOT IS_ABSOLUTE ${CMAKE_INSTALL_PYTHONDIR})
      set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_PYTHONDIR}")
    else()
      set(CMAKE_INSTALL_FULL_PYTHONDIR "${CMAKE_INSTALL_PYTHONDIR}")
    endif()

    # Update RPATH
    if(NOT CMAKE_SKIP_RPATH AND NOT CMAKE_SKIP_INSTALL_RPATH)
      file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_FULL_PYTHONDIR}" "${CMAKE_INSTALL_FULL_LIBDIR}")
      get_target_property(_current_rpath ${SWIG_MODULE_icub_python_REAL_NAME} INSTALL_RPATH)
      if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        list(APPEND _current_rpath "@loader_path/${_rel_path}")
      else()
        list(APPEND _current_rpath "\$ORIGIN/${_rel_path}")
      endif()
      set_target_properties("${SWIG_MODULE_icub_python_REAL_NAME}" PROPERTIES INSTALL_RPATH "${_current_rpath}")
    endif()

    install(TARGETS ${SWIG_MODULE_icub_python_REAL_NAME} DESTINATION ${PYTHON_INSTDIR})
endif()

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
endif()

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
    if(APPLE)
      set_target_properties(icub PROPERTIES SUFFIX ".jnilib")
    endif(APPLE)
endif()

if(CREATE_CSHARP)
  SET(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  swig_add_library(icub_csharp
                     LANGUAGE csharp
                     SOURCES icub.i)

  swig_link_libraries(icub_csharp ${SWIG_YARP_LIBRARIES})
  set_target_properties(${SWIG_MODULE_icub_csharp_REAL_NAME} PROPERTIES OUTPUT_NAME "icub" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/csharp")
endif()

if(CREATE_LUA)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;icub")
  find_package(Lua REQUIRED)
  include_directories(SYSTEM ${LUA_INCLUDE_DIR})
  swig_add_library(icub_lua
                   LANGUAGE lua
                   SOURCES icub.i)
  swig_link_libraries(icub_lua ${LUA_LIBRARY})
  set_target_properties(${SWIG_MODULE_icub_lua_REAL_NAME} PROPERTIES OUTPUT_NAME "icub" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/lua")
endif()
