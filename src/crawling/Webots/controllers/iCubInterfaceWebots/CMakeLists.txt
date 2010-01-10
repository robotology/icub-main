cmake_minimum_required(VERSION 2.6)

SET(KEYWORD "iCubInterfaceWebots")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)

INCLUDE_DIRECTORIES(
    $ENV{WEBOTS_HOME}/include/controller/cpp 
    $ENV{WEBOTS_HOME}/include/controller/c
#    ${KINEMATICS_INCLUDE_DIRS}
#    ${GSL_INCLUDE_DIR}
#    ${EGOSPHERELIB_INCLUDE_DIRS}
)

#Do not change that. Webots controllers have to be put in "controllers" directory for Webots to find them.
IF(WIN32)
	INSTALL(FILES ${CMAKE_BINARY_DIR}/\${BUILD_TYPE}/iCubInterfaceWebots.exe DESTINATION ${PROJECT_SOURCE_DIR})
ELSEIF(WIN32)
	INSTALL(FILES ${CMAKE_BINARY_DIR}/iCubInterfaceWebots DESTINATION ${PROJECT_SOURCE_DIR})
ENDIF (WIN32)

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

link_directories($ENV{WEBOTS_HOME}/lib)

LINK_LIBRARIES(
    Controller
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

