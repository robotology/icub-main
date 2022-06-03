# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Luca Tricerri
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

cmake_minimum_required(VERSION 3.5)

include(FetchContent)

#GTEST
FetchContent_Declare(
        googletest
        GIT_REPOSITORY "https://github.com/google/googletest"
        GIT_TAG        release-1.11.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

project(unittest LANGUAGES CXX)

add_executable(${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)

target_sources(${PROJECT_NAME}
    PRIVATE
    gtest_main.cc
    testServiceParserForMultipleFT.cpp
    testDeviceMultipleFTSensors.cpp
  )

target_link_libraries(${PROJECT_NAME}
PRIVATE 
  gtest 
  gmock
  ethResources
  embObjMultipleFTsensorsUT
  YARP::YARP_init
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

#
# Auto test execution during the build
#
#include(GoogleTest)
#gtest_discover_tests(${PROJECT_NAME})

#add_custom_target(run_unit_test ALL
#    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
#    DEPENDS ${PROJECT_NAME})

