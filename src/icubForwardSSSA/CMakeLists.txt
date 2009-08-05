SET(KEYWORD "servoIcubForward")
FIND_PATH(MATLAB_INCLUDE_DIRS engine.h NO_DEFAULT_PATH)
FIND_PATH(MATLAB_LIBRARIES_DIRS libeng.lib NO_DEFAULT_PATH)

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

LINK_DIRECTORIES(
    ${ICUB_SOURCE_DIR}/lib
)

LINK_LIBRARIES(
    ${OPENCV_LIBRARIES}
    ${MATLAB_LIBRARIES_DIRS}/libeng.lib ${MATLAB_LIBRARIES_DIRS}/libmx.lib ${MATLAB_LIBRARIES_DIRS}/libmex.lib     ${MATLAB_LIBRARIES_DIRS}/libmat.lib)
INCLUDE_DIRECTORIES(${MATLAB_INCLUDE_DIRS})


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

