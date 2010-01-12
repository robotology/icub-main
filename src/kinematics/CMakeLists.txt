SET(KEYWORD "kinematics")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB REQUIRED)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(IKIN REQUIRED)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/kinematics/*.h include/iCub/head/*.h include/iCub/arm/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${GSL_INCLUDE_DIR}
    ${IKIN_INCLUDE_DIRS}
	
)

LINK_LIBRARIES(
    ${GSL_LIBRARIES}
    ${GSLCBLAS_LIBRARIES}
    ${IKIN_LIBRARIES}
)

# Set up our main executable.
IF (folder_source)
  ADD_LIBRARY(${KEYWORD} ${folder_source} ${folder_header})
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

INSTALL_TARGETS(/lib ${KEYWORD})


 
