IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(KEYWORD "rectification")

PROJECT(${KEYWORD})


#added to allow libraries to be linked
#comment out before commiting this to the repository for 
#a global build  DV 6/9/09
SET(LOGPOLAR_DIR "${ICUB_DIR}/src/logPolar")
SET(FOURIERVISION_DIR "${ICUB_DIR}/src/fourierVision")
SET(FOURIERVISION_LIBRARIES "${ICUB_DIR}/lib/Release/fourierVision.lib")
SET(LOGPOLAR_LIBRARIES "${ICUB_DIR}/lib/Release/logPolar.lib")

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(LOGPOLAR REQUIRED)
FIND_PACKAGE(FOURIERVISION REQUIRED)


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${LOGPOLAR_INCLUDE_DIRS}
    ${FOURIERVISION_INCLUDE_DIRS}
    ${FOURIERVISION_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${FOURIERVISION_LIBRARIES}
	${LOGPOLAR_LIBRARIES}
#      none at present
)


# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(/bin ${KEYWORD})

