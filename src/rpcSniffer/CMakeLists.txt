PROJECT(rpcSniffer)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)
#FIND_PACKAGE(CQPLIB REQUIRED)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(rpcSniffer ${folder_source} ${folder_header})
INSTALL(TARGETS rpcSniffer DESTINATION ${ICUB_DIR}/bin)

