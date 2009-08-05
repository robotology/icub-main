SET(PROJECTNAME canLoaderLib)
PROJECT(${PROJECTNAME})

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

FIND_PACKAGE(YARP REQUIRED)

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
