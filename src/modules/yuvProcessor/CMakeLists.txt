IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(PROJECTNAME YUVProcessor)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP)
FIND_PACKAGE(ICUB)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
FIND_PACKAGE(IPP REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	${IPP_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${IPP_LIBRARIES}
	${YARP_LIBRARIES}
	pthread
)

# Set up the main executable.
IF (folder_header)
    ADD_EXECUTABLE(${PROJECTNAME} 
        ${folder_source} 
        ${folder_header}
    )
    INSTALL_TARGETS(/bin ${PROJECTNAME})
ELSE (folder_header)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_header)



