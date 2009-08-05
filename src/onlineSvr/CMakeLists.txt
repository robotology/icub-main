PROJECT(onlineSVR)

#SET(YARP_DIR "$ENV{YARP_ROOT}/build/vc6" CACHE LOCATION "where is your yarp build directory?")

IF(EXISTS $ENV{ICUB_ROOT})
	SET(INSTALL_DIR "$ENV{ICUB_ROOT}" CACHE LOCATION "where is your icub root directory?")
ELSE (EXISTS $ENV{ICUB_ROOT})
	SET(INSTALL_DIR "${CMAKE_SOURCE_DIR}" CACHE LOCATION "where is your icub root directory?")	
ENDIF(EXISTS $ENV{ICUB_ROOT})

FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR} ${IPL_INCLUDE_DIR})
#LINK_LIBRARIES(${IPL_LIBRARY})

ADD_LIBRARY(libOnlineSVR ${folder_source} ${folder_header})
#INSTALL_TARGETS(/lib libOnlineSVR)
INSTALL(TARGETS libOnlineSVR DESTINATION "${INSTALL_DIR}/lib")

### EXAMPLES
ADD_EXECUTABLE(CommandLine "./examples/Main.cpp")
TARGET_LINK_LIBRARIES(CommandLine debug libOnlineSVR optimized libOnlineSVR)
ADD_EXECUTABLE(example1 "./examples/example1.cpp")
TARGET_LINK_LIBRARIES(example1 debug libOnlineSVR optimized libOnlineSVR)
ADD_EXECUTABLE(test "./examples/test.cpp")
TARGET_LINK_LIBRARIES(test debug libOnlineSVR optimized libOnlineSVR)

ADD_DEPENDENCIES(CommandLine libOnlineSVR)
ADD_DEPENDENCIES(example1 libOnlineSVR)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

SET(OnlineSvr_INCLUDE_DIRS ${PROJECT_SOURCE_DIR})
GET_TARGET_PROPERTY(OnlineSvr_LIBRARIES_RELEASE libOnlineSVR RELEASE_LOCATION)
GET_TARGET_PROPERTY(OnlineSvr_LIBRARIES_DEBUG libOnlineSVR DEBUG_LOCATION)
SET(OnlineSvr_LIBRARIES optimized ${OnlineSvr_LIBRARIES_RELEASE}
		    debug ${OnlineSvr_LIBRARIES_DEBUG})

# Generate configuration information for this library.
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/OnlineSvrConfig.cmake.in
               ${PROJECT_BINARY_DIR}/OnlineSvrConfig.cmake
	       @ONLY IMMEDIATE)

