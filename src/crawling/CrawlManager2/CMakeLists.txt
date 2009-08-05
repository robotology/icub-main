SET(KEYWORD "CrawlManager2")

PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)

#INCLUDE_DIRECTORIES(
#	$ENV{WEBOTS_HOME}/include
#)

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#LINK_LIBRARIES(
#    ${KINEMATICS_LIBRARIES}
#    ${GSL_LIBRARIES}
#    ${GSLCBLAS_LIBRARIES}
#    ${EGOSPHERELIB_LIBRARIES}
#)

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

