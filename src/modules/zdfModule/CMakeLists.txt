IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(PROJECTNAME zdfMod)
PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	${IPP_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} )

TARGET_LINK_LIBRARIES(
    ${PROJECTNAME}
	${IPP_LIBRARIES}
	${YARP_LIBRARIES}
	${OpenCV_LIBRARIES}
	pthread
)






