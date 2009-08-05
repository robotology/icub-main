SET(KEYWORD "ihaDebug")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
)

FILE(GLOB source_lib 
    src/debug.cpp
)
FILE(GLOB header_lib
    include/iCub/iha/debug.h
)

LINK_LIBRARIES(
)

# Set up library
ADD_LIBRARY(ihaDebugLib ${source_lib} ${header_lib})
INSTALL_TARGETS(/lib ihaDebugLib)

