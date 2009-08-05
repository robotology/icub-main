PROJECT(arm_tracking)


FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#FIND_LIBRARY(LIB_ONLINESVR
#  NAMES  libOnlineSVR
#  PATHS ${CMAKE_SOURCE_DIR}/../onlinesvr/lib
#	${ICUB_DIR}/lib)

#FIND_PATH(ONLINESVR_HEADERS OnlineSVR.h  ${CMAKE_SOURCE_DIR}/../onlinesvr/)

#LINK_LIBRARIES(${LIB_ONLINESVR})
#INCLUDE_DIRECTORIES(${ONLINESVR_HEADERS})

## NOT THE RECOMMENDED WAY TO FIND THESE LIBRARIES
## SEE: http://eris.liralab.it/iCub/dox/html/cmake_usage.html
##   -- paulfitz

FIND_PACKAGE(YARPMATH)

LINK_LIBRARIES(${YARPMATH_LIBRARIES})

INCLUDE_DIRECTORIES(${YARPMATH_INCLUDE_DIRS})

MESSAGE(${YARPMATH_INCLUDE_DIRS})

ADD_EXECUTABLE(arm_tracking ${folder_source} ${folder_header})


