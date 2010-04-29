SET(PROJECTNAME canLoaderLib)
PROJECT(${PROJECTNAME})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

set(${PROJECTNAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE STRING "")

include_directories(${YARP_INCLUDE_DIRS})
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
