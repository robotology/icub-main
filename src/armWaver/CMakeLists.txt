PROJECT(armWaver)
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")
SET(YARP_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is icub root?")

FIND_PACKAGE(YARP)

FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

ADD_EXECUTABLE(armWaver ${folder_source} ${folder_header})

INSTALL_TARGETS(/bin armWaver) 
