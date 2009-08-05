PROJECT( handMover )
SET(YARP_DIR "$ENV{YARP_ROOT}" CACHE LOCATION "where is yarp?")

SET(CMAKE_BUILD_TYPE distribution)
SET(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3 -DNDEBUG")
SET(CMAKE_C_FLAGS_DISTRIBUTION "-O3 -DNDEBUG")

FIND_PACKAGE(YARP)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
)

# Search for source code.

FILE(GLOB folder_header 
	  ./include/*.h
	  ./include/iCub/*.h
 )

FILE(GLOB folder_source 
	  ./src/*.cpp )

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})


ADD_EXECUTABLE(handMover ${folder_source} ${folder_header}
)

