cmake_minimum_required(VERSION 2.4)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
    
PROJECT(graspDetectorConf)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(GSL)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIR})
ADD_EXECUTABLE(graspDetectorConf ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(graspDetectorConf ${GSL_LIBRARIES})
INSTALL(TARGETS graspDetectorConf DESTINATION bin)
