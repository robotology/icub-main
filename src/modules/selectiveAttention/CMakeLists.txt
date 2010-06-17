SET(KEYWORD "attentionSelection")

PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(EGOSPHERELIB REQUIRED)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${EGOSPHERELIB_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
)

# Search for source code.
FILE(GLOB folder_source 
	src/main.cpp
	src/AttentionSelectionModule.cpp
)
FILE(GLOB folder_header 
	include/iCub/AttentionSelectionInterfaces.h
	include/iCub/AttentionSelectionModule.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Source code for remote library
FILE(GLOB source_lib 
    src/RemoteAttentionSelection.cpp
)
FILE(GLOB header_lib
    include/iCub/RemoteAttentionSelection.h
    include/iCub/AttentionSelectionInterfaces.h
)
SOURCE_GROUP("Source Files" FILES ${source_lib})
SOURCE_GROUP("Header Files" FILES ${header_lib})

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
	${EGOSPHERELIB_LIBRARIES}
	${OPENCV_LIBRARIES}
)
INSTALL_TARGETS(/bin ${KEYWORD})

# Set up library
ADD_LIBRARY(attentionSelectionLib ${source_lib} ${header_lib})
TARGET_LINK_LIBRARIES(attentionSelectionLib
)
INSTALL_TARGETS(/lib attentionSelectionLib)
