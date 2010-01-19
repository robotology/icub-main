cmake_minimum_required(VERSION 2.4)
# replace "..." with the path to your YARP binaries if needed
SET(YARP_DIR "${CMAKE_SOURCE_DIR}/../.." CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP REQUIRED)

ADD_EXECUTABLE(predictiveSP main.cpp)
