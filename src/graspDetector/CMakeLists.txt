cmake_minimum_required(VERSION 2.4)

PROJECT(graspDetector)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(graspDetector ${folder_source} ${folder_header})
INSTALL(TARGETS graspDetector DESTINATION ${ICUB_DIR}/bin)
