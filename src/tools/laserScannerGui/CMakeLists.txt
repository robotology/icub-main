cmake_minimum_required(VERSION 2.6)

set(MODULENAME laserScannerGui)
project(${MODULENAME})

include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(${MODULENAME} ${folder_source} ${folder_header})

TARGET_LINK_LIBRARIES(${MODULENAME} ${OpenCV_LIBRARIES} icubmod ${YARP_LIBRARIES})