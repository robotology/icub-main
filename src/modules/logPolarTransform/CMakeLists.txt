
SET(KEYWORD "logPolarTransform")

PROJECT(${KEYWORD})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${logPolar_INCLUDE_DIRS}
    ${iCubVis_INCLUDE_DIRS}	
    ${YARP_INCLUDE_DIRS}
    #${FOURIERVISION_INCLUDE_DIRS}
    #${FOURIERVISION_INCLUDE_DIRS}
)

MESSAGE("iCubVis_INCLUDE_DIRS: ${iCubVis_INCLUDE_DIRS}  ")
MESSAGE("iCubVis_LIBRARIES: ${iCubVis_LIBRARIES} ")
MESSAGE("iKin_LIBRARIES: ${iKin_LIBRARIES}")

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
	${YARP_LIBRARIES}
	iCubVis
	logPolar
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



