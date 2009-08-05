PROJECT(colorsegmentation)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
FIND_PACKAGE(OpenCV)

SET(CMAKE_INSTALL_PREFIX "$ENV{ICUB_ROOT}" CACHE STRING "insert here your install prefix" FORCE)

# include ipl

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})


INCLUDE_DIRECTORIES(${OPENCV_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})
LINK_LIBRARIES(${OPENCV_LIBRARIES})

ADD_EXECUTABLE(colorsegmentation ${folder_source} ${folder_header})
INSTALL_TARGETS(/bin colorsegmentation) 
