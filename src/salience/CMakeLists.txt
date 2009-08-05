SET(KEYWORD "salience")

# Start a project.
PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(FASTFILT REQUIRED)

# Search for source code.
FILE(GLOB folder_source 
	# TODO: we should exclude the RemoteSalience.cpp
	src/*.cpp
)
FILE(GLOB folder_header 
	# TODO: we should exclude the RemoteSalience.h
	include/iCub/*.h
)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Source code for salience library
FILE(GLOB source_lib 
    src/RemoteSalience.cpp
    src/IOR.cpp
)
FILE(GLOB header_lib
    include/iCub/RemoteSalience.h
    include/iCub/SalienceInterfaces.h
    IOR.h
)
SOURCE_GROUP("Source Files" FILES ${source_lib})
SOURCE_GROUP("Header Files" FILES ${header_lib})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${OPENCV_INCLUDE_DIR}
    ${FASTFILT_INCLUDE_DIRS}
)

# Set up our main executable.
IF (folder_source)
  ADD_EXECUTABLE(${KEYWORD} ${folder_source} ${folder_header})
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
TARGET_LINK_LIBRARIES(${KEYWORD}
    ${OPENCV_LIBRARIES}
    ${FASTFILT_LIBRARIES}
)
INSTALL_TARGETS(/bin ${KEYWORD})

# Set up salience library
ADD_LIBRARY(salienceLib ${source_lib} ${header_lib})
TARGET_LINK_LIBRARIES(salienceLib
)
INSTALL_TARGETS(/lib salienceLib)



