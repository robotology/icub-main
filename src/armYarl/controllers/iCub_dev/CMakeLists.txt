SET(KEYWORD "iCub_dev")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
    $ENV{WEBOTS_HOME}/include
#    ${KINEMATICS_INCLUDE_DIRS}
#    ${GSL_INCLUDE_DIR}
#    ${EGOSPHERELIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
    $ENV{WEBOTS_HOME}/lib/libController.so
#    ${GSL_LIBRARIES}
#    ${GSLCBLAS_LIBRARIES}
#    ${EGOSPHERELIB_LIBRARIES}
)

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

#INSTALL_TARGETS(/bin ${KEYWORD})

