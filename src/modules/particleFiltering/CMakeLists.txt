SET(PROJECTNAME particleModule)

PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} )

TARGET_LINK_LIBRARIES(
    ${PROJECTNAME} 
	${OpenCV_LIBRARIES}
	${YARP_LIBRARIES}
    ${GSL_LIBRARIES}
)
INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)



