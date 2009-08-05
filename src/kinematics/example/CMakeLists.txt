SET(KEYWORD "kinematics")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP)
FIND_PACKAGE(GSL)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${GSL_INCLUDE_DIR}
    ${KINEMATICS_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/../include/
)

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
FILE(GLOB kin_header ../include/iCub/kinematics/*.h ../include/iCub/head/*.h ../include/iCub/arm/*.h)
FILE(GLOB kin_source ../src/*.cpp)
SOURCE_GROUP("Source Files" FILES ${folder_source} ${kin_source} )
SOURCE_GROUP("Header Files" FILES ${folder_header} ${kin_header} )

LINK_LIBRARIES(
    ${GSL_LIBRARIES}
    ${GSLCBLAS_LIBRARIES}
)

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
	${kin_header}
	${kin_source}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(${PROJECT_SOURCE_DIR}/bin ${KEYWORD})


