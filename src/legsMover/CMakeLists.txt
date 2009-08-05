PROJECT(legsMover)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

IF (WIN32 AND NOT CYGWIN)
    ADD_EXECUTABLE(legsMover ${folder_source} ${folder_header})
ELSE (WIN32 AND NOT CYGWIN)
    ADD_EXECUTABLE(legsMover ${folder_source} ${folder_header})
ENDIF (WIN32 AND NOT CYGWIN)
