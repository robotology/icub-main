PROJECT(articulator)

SET(YARP_DIR "$ENV{YARP_BUILD}" CACHE LOCATION "where is yarp?")

FIND_PACKAGE(YARP)

FILE(GLOB_RECURSE folder_source1 *.cc)
FILE(GLOB_RECURSE folder_source2 *.c)
FILE(GLOB_RECURSE folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source1} ${folder_source2})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/gnuspeech)

#LINK_LIBRARIES(fftw3)

ADD_EXECUTABLE(articulator ${folder_source1} ${folder_source2} ${folder_header})

