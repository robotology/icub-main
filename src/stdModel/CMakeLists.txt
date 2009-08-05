PROJECT(stdModel)

FIND_PACKAGE(YARP)

FIND_PATH(IPL_INCLUDE_DIR ipl.h)
INCLUDE_DIRECTORIES(${IPL_INCLUDE_DIR})

FIND_LIBRARY(IPL_LIB_DIR ipl.lib)

# Find source code files
FILE(GLOB HEADERS *.h)
FILE(GLOB CODE *.cpp)

# Lay the files out in folders in environments where that makes sense
# This is completely optional
SOURCE_GROUP("Source Files" FILES ${CODE})
SOURCE_GROUP("Header Files" FILES ${HEADERS})

ADD_EXECUTABLE(stdModel ${HEADERS} ${CODE})
TARGET_LINK_LIBRARIES(stdModel ${IPL_LIB_DIR} )