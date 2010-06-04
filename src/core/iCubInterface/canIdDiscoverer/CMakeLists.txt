PROJECT(canIdDiscoverer)

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
FIND_PACKAGE(CANLOADERLIB)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CANLOADERLIB_INCLUDE_DIRS})
LINK_LIBRARIES(${CANLOADERLIB_LIBRARIES})

ADD_EXECUTABLE(canIdDiscoverer ${folder_source} ${folder_header})
INSTALL_TARGETS(/bin canIdDiscoverer) 
