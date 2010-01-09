SET(KEYWORD "behaviorController")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(ATTENTIONSELECTIONLIB REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ATTENTIONSELECTIONLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	src/*.cpp
)
FILE(GLOB folder_header 
	include/iCub/*.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
TARGET_LINK_LIBRARIES(${KEYWORD}
	${ATTENTIONSELECTIONLIB_LIBRARIES}
)
INSTALL_TARGETS(/bin ${KEYWORD})
