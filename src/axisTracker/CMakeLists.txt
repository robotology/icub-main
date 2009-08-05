PROJECT(objectspotter)

SET(YARP_DIR "$ENV{YARP_BUILD}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)

FILE(GLOB_RECURSE folder_source1 *.cpp)
FILE(GLOB_RECURSE folder_source2 *.c)
FILE(GLOB_RECURSE folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source1} ${folder_source2})
SOURCE_GROUP("Header Files" FILES ${folder_header})

MESSAGE(STATUS ${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

ADD_EXECUTABLE(ospot ${folder_source1} ${folder_source2} ${folder_header})

