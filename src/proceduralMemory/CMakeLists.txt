IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(KEYWORD "proceduralMemory")

PROJECT(${KEYWORD})

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(FOURIERVISION REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${FOURIERVISION_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${FOURIERVISION_LIBRARIES}
)


# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)



