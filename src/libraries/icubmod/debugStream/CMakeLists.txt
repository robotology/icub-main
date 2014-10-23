# Copyright: (C) 2010 RobotCub Consortium
# Authors: Alberto Cardellino
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME debugStream)

include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})

if(YARP_IS_SHARED_LIBRARY)
    set(_type SHARED)
else()
    set(_type STATIC)
endif()

add_library(${PROJECTNAME} ${_type}
                           yarp/os/Log.h
                           yarp/os/LogStream.h
                           Log.cpp)
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
# Temporary hack in order to enable dllexport on WIN32 using YARP_OS_API
set_target_properties(${PROJECTNAME} PROPERTIES COMPILE_DEFINITIONS YARP_OS_EXPORTS
                                                RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
                                                LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
                                                ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

icub_export_library(${PROJECTNAME})

add_executable(${PROJECTNAME}_test test.cpp)
target_link_libraries(${PROJECTNAME}_test ${PROJECTNAME})
