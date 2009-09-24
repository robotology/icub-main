IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(KEYWORD "logPolarTransform")

PROJECT(${KEYWORD})

# The following directives are required to allow libraries to be linked
# when running Cmake directly against this CMakeLists.txt file
# Comment out if the make / solution file is built using a global build  
# i.e. if this module is included in $(ICUB_DIR)/src/CmakeLists.txt
# DV 23/9/09
#SET(LOGPOLAR_DIR "${ICUB_DIR}/src/logPolar")
#SET(FOURIERVISION_DIR "${ICUB_DIR}/src/fourierVision")
#SET(FOURIERVISION_LIBRARIES "${ICUB_DIR}/lib/Release/fourierVision.lib")
#SET(LOGPOLAR_LIBRARIES "${ICUB_DIR}/lib/Release/logPolar.lib")


FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(LOGPOLAR REQUIRED)
FIND_PACKAGE(FOURIERVISION REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${LOGPOLAR_INCLUDE_DIRS}
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
)


# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
  INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(STATUS "No source code files found. Please add something")
ENDIF (folder_source)



