PROJECT(test_gyro)

SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp root?")
SET(ICUB_DIR "${PROJECT_SOURCE_DIR}/../../" CACHE LOCATION "where is icub root?")
SET(INSTALL_DIR "${CMAKE_SOURCE_DIR}")

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ICUB)

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(test_gyro ${folder_source} ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
