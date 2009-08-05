PROJECT(arm_gravity_compensator)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is the yarp build directory?")

SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

FIND_LIBRARY(LIB_ONLINESVR
  NAMES  libOnlineSVR
  PATHS ${CMAKE_SOURCE_DIR}/../onlineSvr/lib
        ${CMAKE_SOURCE_DIR}/../onlineSvr
	${ICUB_ROOT}/lib)

FIND_PATH(ONLINESVR_HEADERS OnlineSVR.h  ${CMAKE_SOURCE_DIR}/../onlineSvr/)

LINK_LIBRARIES(${LIB_ONLINESVR})
INCLUDE_DIRECTORIES(${ONLINESVR_HEADERS})

ADD_EXECUTABLE(arm_gravity_compensator ${folder_source} ${folder_header})


