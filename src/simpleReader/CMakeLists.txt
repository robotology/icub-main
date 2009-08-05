PROJECT(simpleReader)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(simpleReader ${folder_source} ${folder_header})
INSTALL_TARGETS(/bin simpleReader)
