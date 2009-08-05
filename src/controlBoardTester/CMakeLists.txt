cmake_minimum_required(VERSION 2.6)
PROJECT(controlBoardTester)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(controlBoardTester ${folder_source} ${folder_header})
INSTALL(TARGETS controlBoardTester DESTINATION ${ICUB_DIR}/bin)

