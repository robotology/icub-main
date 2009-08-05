PROJECT(motorIdentification)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is the yarp build directory?")
SET(ICUB_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")
SET(INSTALL_DIR "$ENV{ICUB_ROOT}")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(motorIdentification ${folder_source} ${folder_header})
INSTALL(TARGETS motorIdentification DESTINATION ${INSTALL_DIR}/bin)