IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(PROJECTNAME particleModule)

PROJECT(${PROJECTNAME})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${OpenCV_LIBRARIES}
	${YARP_LIBRARIES}
)

# Set up the main executable.
IF (folder_header)
    ADD_EXECUTABLE(${PROJECTNAME} 
        ${folder_source} 
        ${folder_header}
    )
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)

ELSE (folder_header)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_header)



