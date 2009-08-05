SET(KEYWORD "controlGaze")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(KINEMATICS REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(EGOSPHERELIB REQUIRED)
FIND_PACKAGE(CAMERA)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${KINEMATICS_INCLUDE_DIRS}
    ${GSL_INCLUDE_DIR}
    ${EGOSPHERELIB_INCLUDE_DIRS}
    ${CAMERA_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_LIBRARIES(
    ${KINEMATICS_LIBRARIES}
    ${GSL_LIBRARIES}
    ${GSLCBLAS_LIBRARIES}
    ${EGOSPHERELIB_LIBRARIES}
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

INSTALL_TARGETS(/bin ${KEYWORD})

