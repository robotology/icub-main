PROJECT(arm_feedfwd_control)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is the yarp build directory?")

SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(arm_feedfwd_control ${folder_source} ${folder_header})


