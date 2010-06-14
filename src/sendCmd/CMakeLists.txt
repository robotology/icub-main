cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME sendCmd)
PROJECT(${PROJECTNAME})
FIND_PACKAGE(YARP)

FIND_PACKAGE(ICUB QUIET)
IF (NOT ICUB_FOUND)
   MESSAGE (STATUS "ICUB repository not found; assuming distributed build.")
   MESSAGE (STATUS "If this is not what you want, set ICUB_DIR variable.")
ENDIF (NOT ICUB_FOUND)

# Find source code files
FILE(GLOB SENDCMD_INC include/iCub/*.h)

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Header Files" FILES ${SENDCMD_INC})

# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(sendcmd src/bin/sendcmd.cpp)
TARGET_LINK_LIBRARIES(sendcmd ${YARP_LIBRARIES})

